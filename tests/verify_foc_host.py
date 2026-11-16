"""Exercise GUI backpressure without a PCAN adapter or a Tk window."""
import ast
from pathlib import Path
import queue
import struct
import threading
import time
from types import SimpleNamespace, ModuleType

source_path = Path(__file__).resolve().parents[1] / "scripts/foc_can_host.py"
tree = ast.parse(source_path.read_text(encoding="utf-8-sig"))
# Load the actual application class without importing Tk/PCAN or starting a
# window. This keeps the stress check usable in the bundled headless Python.
tree.body = [node for node in tree.body if
             (isinstance(node, ast.Import) and
              all(alias.name in {"time", "struct", "queue", "threading"}
                  for alias in node.names)) or
             isinstance(node, (ast.ClassDef, ast.Assign))]
host = ModuleType("foc_host")
exec(compile(tree, str(source_path), "exec"), host.__dict__)


class Log:
    def __init__(self):
        self.lines = []
        self.inserts = 0

    def configure(self, **kwargs):
        pass

    def insert(self, where, text):
        self.inserts += 1
        self.lines.extend(text.splitlines())

    def delete(self, start, end):
        del self.lines[:int(end.split(".")[0]) - 1]

    def see(self, where):
        pass


def make_app():
    app = host.FocCanApp.__new__(host.FocCanApp)
    app.rx_queue = queue.Queue(maxsize=host.MAX_EVENT_QUEUE)
    app.rx_lock = threading.Lock()
    app.latest_rx = {}
    app.motor_feedback = {}
    app.rx_counts = {}
    app.rx_total = app.dropped_logs = app.log_lines = 0
    app.log = Log()
    app.autoscroll = SimpleNamespace(get=lambda: True)
    app.scheduled = []
    app.root = SimpleNamespace(after=lambda delay, callback: app.scheduled.append(delay))
    app.status_text = ""
    app.rx_status = SimpleNamespace(config=lambda **kw: setattr(app, "status_text", kw["text"]))
    app.bus = object()
    app.preview_mode = False
    app.rate_time, app.rate_total, app.display_rate = time.monotonic(), 0, 0
    class Label:
        def __init__(self):
            self.options = {}

        def config(self, **kwargs):
            self.options.update(kwargs)
    app.motor_widgets = {mid: {key: Label() for key in
                              ("card", "badge", "angle", "speed", "frames")}
                         for mid in range(1, 9)}
    app.online_summary, app.rate_label = Label(), Label()
    return app


def message(mid, angle=1500):
    return SimpleNamespace(arbitration_id=0x100 + mid,
                           data=struct.pack("<ih", angle, -120) + b"\0\0",
                           is_extended_id=False, is_remote_frame=False,
                           is_error_frame=False)


app = make_app()


def produce(mid):
    for n in range(50000):
        app.record_rx(message(mid, n))


threads = [threading.Thread(target=produce, args=(mid,)) for mid in (1, 6)]
for thread in threads:
    thread.start()
for thread in threads:
    thread.join()
assert app.rx_total == 100000
assert app.rx_counts[(0x101, False, False, False)] == 50000
assert app.rx_counts[(0x106, False, False, False)] == 50000
assert len(app.latest_rx) == 2
app.poll_rx()
assert app.log.inserts == 1 and len(app.log.lines) == 2
assert "ID1:50000" in app.status_text and "ID6:50000" in app.status_text
assert "角度=49.999 rad" in app.log.lines[0]
assert app.scheduled == [100]
assert app.motor_widgets[1]["angle"].options["text"] == "49.999"
assert app.motor_widgets[6]["speed"].options["text"] == "-12.0 rpm"
assert app.online_summary.options["text"] == "2 / 8 在线"
app.update_dashboard(app.motor_feedback, app.rx_counts, app.rx_total, time.monotonic() + 1)
assert app.online_summary.options["text"] == "0 / 8 在线"
assert app.motor_widgets[1]["badge"].options["text"] == "● 离线"

# An error flood is bounded and cannot consume an entire UI turn.
for n in range(1000):
    app.log_msg("error %d" % n)
assert app.rx_queue.qsize() == host.MAX_EVENT_QUEUE
assert app.dropped_logs == 1000 - host.MAX_EVENT_QUEUE
before = app.rx_queue.qsize()
app.poll_rx()
assert before - app.rx_queue.qsize() == host.MAX_EVENTS_PER_REFRESH
assert app.scheduled[-1] == 100

# Even a queue that is replenished on every read cannot starve the event loop.
class ReplenishedQueue:
    def __init__(self):
        self.reads = 0

    def get_nowait(self):
        self.reads += 1
        assert self.reads <= host.MAX_EVENTS_PER_REFRESH, "UI turn never yields"
        return "log", "continuous flood"


app.rx_queue = ReplenishedQueue()
app.poll_rx()
assert app.rx_queue.reads == host.MAX_EVENTS_PER_REFRESH

# Long operation stays within the text history cap.
app.rx_queue = queue.Queue(maxsize=host.MAX_EVENT_QUEUE)
for cycle in range(1000):
    app.record_rx(message(1, cycle))
    app.record_rx(message(6, cycle))
    app.log_msg("status")
    app.poll_rx()
assert len(app.log.lines) == host.MAX_LOG_LINES
assert app.log_lines == host.MAX_LOG_LINES

# Discovery on unrelated traffic has a finite number of retained IDs.
for mid in range(1000):
    app.record_rx(message(mid))
assert len(app.latest_rx) == 64 and len(app.rx_counts) == 64
print("PASS: 100000 frames from ID1/ID6, bounded UI turns, log flood and history limits")
