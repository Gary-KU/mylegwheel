# -*- coding: utf-8 -*-
"""
FOC 电机 CAN 上位机 (PCAN-USB)
功能:
  - 发送电压指令到指定电机 ID
  - 接收并解析反馈(角度/转速)
  - 通过 CAN 修改电机 ID(0x300, 需固件支持)
协议(1Mbps 标准帧):
  命令  0x100(ID1~4) / 0x200(ID5~8), 每 2 字节 = int16 电压(mV) 小端
  反馈  0x100+N, data[0:4]=int32 角度*1000, data[4:6]=int16 转速*10
  改ID  0x300, data[0]=新ID(1~8)
"""
import time
import struct
import queue
import threading
import sys
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

try:
    import can
    HAVE_CAN = True
except ImportError:
    HAVE_CAN = False

UI_PERIOD_MS = 100
TX_PERIOD_MS = 20
MAX_EVENT_QUEUE = 256
MAX_EVENTS_PER_REFRESH = 32
MAX_LOG_LINES = 2000
MOTOR_ONLINE_SECONDS = 0.5


class FocCanApp:
    def __init__(self, root):
        self.root = root
        root.title("FOC 电机 CAN 上位机 (PCAN-USB)")
        root.geometry("1180x860")
        root.minsize(1060, 820)
        root.configure(bg="#eef2f6")
        self.bus = None
        self.rx_thread = None
        self.rx_stop = threading.Event()
        self.rx_queue = queue.Queue(maxsize=MAX_EVENT_QUEUE)
        self.rx_lock = threading.Lock()
        self.latest_rx = {}
        self.rx_counts = {}
        self.rx_total = 0
        self.dropped_logs = 0
        self.log_lines = 0
        self.connection_settings = ("PCAN_USBBUS1", 1000000)
        self.motor_feedback = {}
        self.motor_widgets = {}
        self.rate_time = time.monotonic()
        self.rate_total = 0
        self.display_rate = 0
        self.preview_mode = False
        self.tx_running = False
        self.build_ui()
        self.poll_rx()
        root.protocol("WM_DELETE_WINDOW", self.on_close)

    def build_ui(self):
        self.colors = dict(bg="#eef2f6", card="#ffffff", ink="#17283c",
                           muted="#738196", accent="#087f70", line="#dce4ed")
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("TFrame", background=self.colors["bg"])
        style.configure("Card.TFrame", background="#ffffff")
        style.configure("TLabel", background="#ffffff", foreground="#17283c",
                        font=("Microsoft YaHei UI", 10))
        style.configure("Muted.TLabel", foreground="#738196", font=("Microsoft YaHei UI", 9))
        style.configure("Section.TLabel", font=("Microsoft YaHei UI", 12, "bold"))
        style.configure("TButton", background="#edf2f7", foreground="#263b51",
                        borderwidth=0, padding=(12, 9), font=("Microsoft YaHei UI", 9))
        style.map("TButton", background=[("active", "#dfe8f0")],
                  foreground=[("disabled", "#9aa7b5")])
        style.configure("Primary.TButton", background="#087f70", foreground="white")
        style.map("Primary.TButton", background=[("active", "#07685d"), ("disabled", "#e2e8ee")],
                  foreground=[("disabled", "#9aa7b5")])
        style.configure("Stop.TButton", background="#fee7e5", foreground="#bf3d35",
                        font=("Microsoft YaHei UI", 11, "bold"), padding=(12, 12))
        style.map("Stop.TButton", background=[("active", "#ffd6d2")])
        style.configure("TEntry", fieldbackground="#f7f9fc", foreground="#17283c",
                        bordercolor="#dce4ed", padding=7)
        style.configure("TSpinbox", fieldbackground="#f7f9fc", foreground="#17283c",
                        bordercolor="#dce4ed", padding=7, arrowsize=14)
        style.configure("TCombobox", fieldbackground="#f7f9fc", foreground="#17283c",
                        bordercolor="#dce4ed", padding=7)
        style.configure("TCheckbutton", background="#ffffff", foreground="#738196",
                        font=("Microsoft YaHei UI", 9))

        # A quiet header separates connection status from motor controls.
        header = tk.Frame(self.root, bg="#17283c", padx=24, pady=14)
        header.pack(fill="x")
        tk.Label(header, text="FOC  /  MOTOR CONSOLE", bg="#17283c", fg="#63dcc2",
                 font=("Segoe UI", 10, "bold")).pack(anchor="w")
        title_row = tk.Frame(header, bg="#17283c")
        title_row.pack(fill="x", pady=(5, 0))
        tk.Label(title_row, text="电机调试工作台", bg="#17283c", fg="white",
                 font=("Microsoft YaHei UI", 22, "bold")).pack(side="left")
        self.status = tk.Label(title_row, text="●  未连接", bg="#253c51", fg="#b8c6d4",
                               font=("Microsoft YaHei UI", 10), padx=15, pady=8)
        self.status.pack(side="right")
        tk.Label(header, text="PCAN-USB  ·  实时反馈 / 电压调试 / 驱动配置", bg="#17283c",
                 fg="#91a6ba", font=("Microsoft YaHei UI", 9)).pack(anchor="w", pady=(6, 0))

        connection = ttk.Frame(self.root, style="Card.TFrame", padding=(20, 14))
        connection.pack(fill="x", padx=20, pady=(16, 12))
        ttk.Label(connection, text="总线连接", style="Section.TLabel").pack(side="left", padx=(0, 20))
        ttk.Label(connection, text="通道", style="Muted.TLabel").pack(side="left", padx=(0, 8))
        self.ch_var = tk.StringVar(value="PCAN_USBBUS1")
        ttk.Entry(connection, textvariable=self.ch_var, width=18).pack(side="left")
        ttk.Label(connection, text="速率 / bps", style="Muted.TLabel").pack(side="left", padx=(20, 8))
        self.br_var = tk.StringVar(value="1000000")
        ttk.Combobox(connection, textvariable=self.br_var, values=("1000000", "500000", "250000"),
                     width=11).pack(side="left")
        self.btn_disc = ttk.Button(connection, text="断开", command=self.disconnect, state="disabled")
        self.btn_disc.pack(side="right")
        self.btn_conn = ttk.Button(connection, text="连接总线", command=self.connect, style="Primary.TButton")
        self.btn_conn.pack(side="right", padx=(12, 8))

        workspace = ttk.Frame(self.root)
        workspace.pack(fill="both", expand=True, padx=20)
        workspace.columnconfigure(1, weight=1)
        workspace.rowconfigure(0, weight=1)
        controls = ttk.Frame(workspace, style="Card.TFrame", padding=18, width=280)
        controls.grid(row=0, column=0, sticky="nsew", padx=(0, 14))
        controls.columnconfigure(0, weight=1)
        ttk.Label(controls, text="电压调试", style="Section.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Label(controls, text="选择电机，发送轴电压指令", style="Muted.TLabel").grid(
            row=1, column=0, sticky="w", pady=(4, 16))
        ttk.Label(controls, text="目标电机 ID", style="Muted.TLabel").grid(row=2, column=0, sticky="w")
        self.motor_id = tk.IntVar(value=7)
        ttk.Spinbox(controls, from_=1, to=8, textvariable=self.motor_id, width=20).grid(
            row=3, column=0, sticky="ew", pady=(5, 12))
        ttk.Label(controls, text="输出电压 / mV", style="Muted.TLabel").grid(row=4, column=0, sticky="w")
        self.volt_var = tk.StringVar(value="500")
        ttk.Entry(controls, textvariable=self.volt_var, width=20).grid(
            row=5, column=0, sticky="ew", pady=(5, 10))
        presets = ttk.Frame(controls, style="Card.TFrame")
        presets.grid(row=6, column=0, sticky="ew", pady=(0, 10))
        for index, voltage in enumerate((500, -500, 1000, -1000)):
            presets.columnconfigure(index % 2, weight=1)
            ttk.Button(presets, text="%+d mV" % voltage, width=10,
                       command=lambda v=voltage: self.quick_send(v)).grid(
                           row=index // 2, column=index % 2, sticky="ew", padx=2, pady=2)
        actions = ttk.Frame(controls, style="Card.TFrame")
        actions.grid(row=7, column=0, sticky="ew", pady=(0, 8))
        ttk.Button(actions, text="发送一次", command=self.send_once, style="Primary.TButton").pack(
            side="left", fill="x", expand=True, padx=(0, 6))
        self.btn_cont = ttk.Button(actions, text="持续发送", command=self.toggle_continuous)
        self.btn_cont.pack(side="left", fill="x", expand=True)
        ttk.Button(controls, text="停止输出 · 0 V", command=self.send_stop, style="Stop.TButton").grid(
            row=8, column=0, sticky="ew")
        ttk.Separator(controls).grid(row=9, column=0, sticky="ew", pady=18)
        ttk.Label(controls, text="驱动 ID 配置", style="Section.TLabel").grid(row=10, column=0, sticky="w")
        ttk.Label(controls, text="改 ID 时，总线只连接一块驱动。", style="Muted.TLabel").grid(
            row=11, column=0, sticky="w", pady=(6, 12))
        id_row = ttk.Frame(controls, style="Card.TFrame")
        id_row.grid(row=12, column=0, sticky="ew")
        self.new_id = tk.IntVar(value=7)
        ttk.Spinbox(id_row, from_=1, to=8, textvariable=self.new_id, width=4).pack(side="left", padx=(0, 8))
        ttk.Button(id_row, text="设置并重启", command=self.set_motor_id).pack(side="left", fill="x", expand=True)
        controls.rowconfigure(13, weight=1)

        monitor = ttk.Frame(workspace)
        monitor.grid(row=0, column=1, sticky="nsew")
        monitor.columnconfigure(0, weight=1)
        monitor.rowconfigure(2, weight=1)
        monitor_header = ttk.Frame(monitor, style="Card.TFrame", padding=(16, 12))
        monitor_header.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        ttk.Label(monitor_header, text="实时电机反馈", style="Section.TLabel").pack(side="left")
        self.online_summary = ttk.Label(monitor_header, text="0 / 8 在线", foreground="#087f70")
        self.online_summary.pack(side="right")
        grid = ttk.Frame(monitor)
        grid.grid(row=1, column=0, sticky="ew", pady=(0, 12))
        for column in range(4):
            grid.columnconfigure(column, weight=1, uniform="motors")
        for mid in range(1, 9):
            card = tk.Frame(grid, bg="white", highlightbackground="#dce4ed", highlightthickness=1,
                            padx=12, pady=10)
            card.grid(row=(mid - 1) // 4, column=(mid - 1) % 4, sticky="nsew",
                      padx=(0 if (mid - 1) % 4 == 0 else 4, 0 if mid % 4 == 0 else 4), pady=4)
            heading = tk.Frame(card, bg="white")
            heading.pack(fill="x")
            tk.Label(heading, text="M%02d" % mid, bg="white", fg="#17283c",
                     font=("Segoe UI", 12, "bold")).pack(side="left")
            badge = tk.Label(heading, text="● 等待", bg="white", fg="#9aa7b5",
                             font=("Microsoft YaHei UI", 8))
            badge.pack(side="right")
            angle = tk.Label(card, text="—", bg="white", fg="#253b52",
                             anchor="w", font=("Segoe UI", 22, "bold"))
            angle.pack(fill="x", pady=(9, 0))
            tk.Label(card, text="角度  rad", bg="white", fg="#8b99aa", anchor="w",
                     font=("Microsoft YaHei UI", 8)).pack(fill="x")
            speed = tk.Label(card, text="—  rpm", bg="white", fg="#546b81", anchor="w",
                             font=("Segoe UI", 11))
            speed.pack(fill="x", pady=(7, 0))
            frames = tk.Label(card, text="0 帧", bg="white", fg="#8b99aa", anchor="w",
                              font=("Microsoft YaHei UI", 8))
            frames.pack(fill="x", pady=(5, 0))
            self.motor_widgets[mid] = dict(card=card, badge=badge, angle=angle, speed=speed, frames=frames)

        log_panel = ttk.Frame(monitor, style="Card.TFrame", padding=(14, 12))
        log_panel.grid(row=2, column=0, sticky="nsew")
        bar = ttk.Frame(log_panel, style="Card.TFrame")
        bar.pack(fill="x", pady=(0, 8))
        ttk.Label(bar, text="通信日志", style="Section.TLabel").pack(side="left")
        ttk.Button(bar, text="清空日志", command=self.clear_log).pack(side="right")
        self.autoscroll = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text="自动滚动", variable=self.autoscroll).pack(side="right", padx=10)
        self.log = scrolledtext.ScrolledText(log_panel, height=7, state="disabled", bg="#17283c",
                                            fg="#c5d4e3", insertbackground="#63dcc2", borderwidth=0,
                                            relief="flat", font=("Consolas", 9), padx=12, pady=10,
                                            selectbackground="#35536b", wrap="none")
        self.log.pack(fill="both", expand=True)
        footer = ttk.Frame(self.root, padding=(22, 10))
        footer.pack(fill="x", side="bottom", before=workspace)
        self.rx_status = tk.Label(footer, text="收到 0 帧", bg="#eef2f6", fg="#738196",
                                  font=("Microsoft YaHei UI", 9))
        self.rx_status.pack(side="left")
        self.rate_label = tk.Label(footer, text="0 帧/s", bg="#eef2f6", fg="#087f70",
                                   font=("Segoe UI", 10, "bold"))
        self.rate_label.pack(side="right")

    # ---------- 日志 ----------
    def log_msg(self, s):
        # Error/log floods must never block the receive thread or grow memory.
        try:
            self.rx_queue.put_nowait(("log", s))
        except queue.Full:
            with self.rx_lock:
                self.dropped_logs += 1

    def record_rx(self, message):
        # Count every frame; publish only the latest value for each CAN ID.
        # Tk widgets and variables are accessed exclusively by the UI thread.
        key = (message.arbitration_id, message.is_extended_id,
               message.is_remote_frame, message.is_error_frame)
        with self.rx_lock:
            self.rx_total += 1
            self.rx_counts[key] = self.rx_counts.get(key, 0) + 1
            self.latest_rx[key] = message
            if not any(key[1:]) and 0x101 <= key[0] <= 0x108 and len(message.data) >= 6:
                self.motor_feedback[key[0] - 0x100] = (message, time.monotonic())
            # Retain bounded discovery data even on a noisy/unrelated bus.
            if len(self.latest_rx) > 64:
                self.latest_rx.pop(next(iter(self.latest_rx)))
            if len(self.rx_counts) > 64:
                self.rx_counts.pop(next(iter(self.rx_counts)))

    def poll_rx(self):
        lines = []
        for _ in range(MAX_EVENTS_PER_REFRESH):
            try:
                kind, data = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            if kind == "log":
                lines.append(data)
        with self.rx_lock:
            latest = list(self.latest_rx.values())
            self.latest_rx.clear()
            total = self.rx_total
            counts = dict(self.rx_counts)
            dropped = self.dropped_logs
            feedback = dict(self.motor_feedback)
        lines.extend(self.fmt_frame(m) for m in latest)
        if lines:
            # One widget update per refresh, instead of several per CAN frame.
            self.log.configure(state="normal")
            self.log.insert("end", "\n".join(lines) + "\n")
            self.log_lines += len(lines)
            excess = self.log_lines - MAX_LOG_LINES
            if excess > 0:
                self.log.delete("1.0", "%d.0" % (excess + 1))
                self.log_lines -= excess
            self.log.configure(state="disabled")
            if self.autoscroll.get():
                self.log.see("end")
        ids = ", ".join("ID%d:%d" % (key[0] - 0x100, count)
                        for key, count in sorted(counts.items())
                        if not any(key[1:]) and 0x101 <= key[0] <= 0x108)
        self.rx_status.config(text="收到 %d 帧 | %s | 丢弃日志 %d" %
                              (total, ids or "尚无电机反馈", dropped))
        self.update_dashboard(feedback, counts, total, time.monotonic())
        self.root.after(UI_PERIOD_MS, self.poll_rx)

    def update_dashboard(self, feedback, counts, total, now):
        online_count = 0
        connected = self.bus is not None or self.preview_mode
        for mid, widgets in self.motor_widgets.items():
            value = feedback.get(mid)
            online = connected and value is not None and now - value[1] <= MOTOR_ONLINE_SECONDS
            online_count += bool(online)
            color = "#087f70" if online else "#9aa7b5"
            widgets["badge"].config(text="● 在线" if online else ("● 离线" if value else "● 等待"), fg=color)
            widgets["card"].config(highlightbackground="#79c9b7" if online else "#dce4ed")
            if value:
                data = value[0].data
                angle = struct.unpack("<i", bytes(data[:4]))[0] / 1000.0
                speed = struct.unpack("<h", bytes(data[4:6]))[0] / 10.0
                widgets["angle"].config(text="%.3f" % angle, fg="#087f70" if online else "#8b99aa")
                widgets["speed"].config(text="%+.1f rpm" % speed)
            count = counts.get((0x100 + mid, False, False, False), 0)
            widgets["frames"].config(text="%s 帧" % format(count, ","))
        self.online_summary.config(text="%d / 8 在线" % online_count)
        elapsed = now - self.rate_time
        if elapsed >= 0.5:
            self.display_rate = max(0, (total - self.rate_total) / elapsed)
            self.rate_total, self.rate_time = total, now
        self.rate_label.config(text="%s 帧/s" % format(round(self.display_rate), ","))

    # ---------- 连接 ----------
    def connect(self):
        if not HAVE_CAN:
            messagebox.showerror("错误", "未安装 python-can\n请先执行: pip install python-can")
            return
        try:
            ch = self.ch_var.get().strip() or "PCAN_USBBUS1"
            br = int(self.br_var.get())
            self.bus = can.interface.Bus(interface="pcan", channel=ch, bitrate=br)
            self.connection_settings = (ch, br)
        except Exception as e:
            messagebox.showerror("连接失败", str(e))
            return
        self.status.config(text="●  总线已连接", foreground="#63dcc2")
        self.btn_conn.config(state="disabled")
        self.btn_disc.config(state="normal")
        self.rx_stop.clear()
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()
        self.log_msg("[连接成功] %s @ %d bps" % (ch, int(self.br_var.get())))

    def disconnect(self):
        self.tx_running = False
        self.rx_stop.set()
        if self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None
        self.status.config(text="●  未连接", foreground="#b8c6d4")
        self.btn_conn.config(state="normal")
        self.btn_disc.config(state="disabled")
        self.log_msg("[已断开]")

    def rx_loop(self):
        while not self.rx_stop.is_set():
            if not self.bus:
                time.sleep(0.2)
                continue
            try:
                m = self.bus.recv(timeout=0.2)
            except can.CanError as e:
                s = str(e).lower()
                if 'bus-off' in s or 'bus off' in s:
                    self.log_msg("[RX错误] %s — 自动恢复中..." % e)
                    self.recover_bus()
                else:
                    self.log_msg("[RX错误] %s" % e)
                    self.rx_stop.wait(0.05)
                continue
            except Exception as e:
                self.log_msg("[RX异常] %s" % e)
                self.rx_stop.wait(0.05)
                continue
            if m is None:
                continue
            self.record_rx(m)

    def recover_bus(self):
        if self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None
        if self.rx_stop.wait(0.3):
            return
        try:
            ch, br = self.connection_settings
            self.bus = can.interface.Bus(interface="pcan", channel=ch, bitrate=br)
            self.log_msg("[已恢复] CAN 控制器重新初始化")
        except Exception as e:
            self.log_msg("[恢复失败] %s" % e)
            self.bus = None

    def fmt_frame(self, m):
        aid = m.arbitration_id
        d = list(m.data)
        t = time.strftime("%H:%M:%S")
        if (not m.is_extended_id and not m.is_remote_frame and
                not m.is_error_frame and 0x101 <= aid <= 0x108 and len(d) >= 6):
            ang = struct.unpack("<i", bytes(d[0:4]))[0] / 1000.0
            spd = struct.unpack("<h", bytes(d[4:6]))[0] / 10.0
            return "%s RX 0x%03X (ID=%d): 角度=%.3f rad, 转速=%.1f rpm" % (t, aid, aid - 0x100, ang, spd)
        return "%s RX 0x%03X: %s" % (t, aid, d)

    # ---------- 发送 ----------
    def build_voltage_frame(self, mid, mv):
        v = int(mv)
        if not -32768 <= v <= 32767:
            raise ValueError("电压超出 int16 范围")
        data = [0] * 8
        if 1 <= mid <= 4:
            aid = 0x100
            off = (mid - 1) * 2
        elif 5 <= mid <= 8:
            aid = 0x200
            off = (mid - 5) * 2
        else:
            raise ValueError("ID 必须在 1~8")
        data[off] = v & 0xFF
        data[off + 1] = (v >> 8) & 0xFF
        return aid, bytes(data)

    def send_voltage(self, mv):
        if not self.bus:
            messagebox.showwarning("未连接", "请先连接")
            return False
        try:
            mid = int(self.motor_id.get())
            aid, data = self.build_voltage_frame(mid, mv)
            self.bus.send(can.Message(arbitration_id=aid, data=data, is_extended_id=False))
            self.log_msg("TX 0x%03X (ID=%d): %+d mV" % (aid, mid, int(mv)))
            return True
        except Exception as e:
            messagebox.showerror("发送失败", str(e))
            return False

    def send_once(self):
        try:
            mv = int(self.volt_var.get())
        except ValueError:
            messagebox.showerror("错误", "电压必须是整数")
            return
        self.send_voltage(mv)

    def quick_send(self, v):
        self.volt_var.set(str(v))
        self.send_voltage(v)

    def send_stop(self):
        self.tx_running = False
        timer = getattr(self, 'tx_timer', None)
        if timer is not None:
            self.root.after_cancel(timer)
            self.tx_timer = None
        self.btn_cont.config(text="持续发送")
        self.send_voltage(0)

    def toggle_continuous(self):
        if self.tx_running:
            self.send_stop()
        else:
            self.tx_running = True
            self.btn_cont.config(text="停止持续")
            self.continuous_loop()

    def continuous_loop(self):
        self.tx_timer = None
        if not self.tx_running:
            return
        try:
            mv = int(self.volt_var.get())
        except ValueError:
            mv = 0
        if not self.send_voltage(mv):
            self.tx_running = False
            self.btn_cont.config(text="持续发送")
            return
        self.tx_timer = self.root.after(TX_PERIOD_MS, self.continuous_loop)

    def set_motor_id(self):
        if not self.bus:
            messagebox.showwarning("未连接", "请先连接")
            return
        nid = int(self.new_id.get())
        if not 1 <= nid <= 8:
            messagebox.showerror("错误", "ID 必须在 1~8")
            return
        try:
            self.bus.send(can.Message(arbitration_id=0x300, data=[nid, 0, 0, 0, 0, 0, 0, 0],
                                      is_extended_id=False))
            self.log_msg("TX 0x300: 设置ID=%d (板子保存后自动重启)" % nid)
        except Exception as e:
            messagebox.showerror("发送失败", str(e))

    def clear_log(self):
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")
        self.log_lines = 0

    def on_close(self):
        self.tx_running = False
        self.rx_stop.set()
        if self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass
        self.root.destroy()


def main():
    root = tk.Tk()
    app = FocCanApp(root)
    if "--preview" in sys.argv:
        # Layout preview uses synthetic feedback and never opens a CAN device.
        from types import SimpleNamespace
        app.preview_mode = True
        app.status.config(text="界面预览 · 模拟数据", foreground="#63dcc2")
        app.btn_conn.config(state="disabled")
        def preview_tick():
            for mid, angle, rpm in ((1, 14503, 125), (6, -2815, -80)):
                app.record_rx(SimpleNamespace(arbitration_id=0x100 + mid,
                              data=struct.pack("<ih", angle, rpm) + b"\0\0",
                              is_extended_id=False, is_remote_frame=False, is_error_frame=False))
            root.after(100, preview_tick)
        preview_tick()
        app.log_msg("界面预览：显示模拟数据，未连接 CAN 设备。")
    root.mainloop()


if __name__ == "__main__":
    main()
