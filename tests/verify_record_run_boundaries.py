"""A stopped-tail restart must not be summarized as one continuous trial."""
from pathlib import Path
import contextlib
import io
import json
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from read_lqr_startup_record import active_run_summaries, export, header_decode, MAGIC


def row(index, active, tick=None):
    return dict(active=active, elapsed_ms=20*index,
                tick_ms=20*index if tick is None else tick, phase=3 if active else 0,
                body_pitch_deg=float(index), state=[0, 0, 0, -.1*index, 0, 0],
                position_error_m=.01*index)


class RunBoundaries(unittest.TestCase):
    def test_restart_in_tail_is_separate_and_final_run_is_truncated(self):
        rows = [row(i, i < 2 or i >= 4) for i in range(7)]
        first, second = active_run_summaries(rows, 2)
        self.assertEqual((first['start_index'], first['end_index']), (0, 1))
        self.assertEqual(first['active_records'], 2)
        self.assertTrue(first['stop_observed'])
        self.assertFalse(first['starts_after_trigger'])
        self.assertEqual((second['start_index'], second['end_index']), (4, 6))
        self.assertEqual(second['active_records'], 3)
        self.assertTrue(second['starts_after_trigger'])
        self.assertFalse(second['stop_observed'])
        self.assertEqual(first['max_abs_pitch_deg'], 1)
        self.assertEqual(second['max_abs_pitch_deg'], 6)

    def test_fall_trigger_while_active_is_still_one_run(self):
        rows = [row(i, i < 4) for i in range(7)]
        runs = active_run_summaries(rows, 1)
        self.assertEqual(len(runs), 1)
        self.assertEqual(runs[0]['active_records'], 4)
        self.assertFalse(runs[0]['starts_after_trigger'])

    def test_tick_wrap_and_no_trigger(self):
        rows = [row(0, True, 0xfffffff0), row(1, True, 4)]
        run, = active_run_summaries(rows, 0xffffffff)
        self.assertEqual(run['observed_duration_ms'], 20)
        self.assertFalse(run['starts_after_trigger'])

    def test_no_active_rows(self):
        self.assertEqual(active_run_summaries([], 0), [])
        self.assertEqual(active_run_summaries([row(0, False)], 0), [])

    def test_historical_export_warns_and_preserves_original_rows(self):
        raw_header = bytearray(128)
        struct.pack_into('<12I', raw_header, 0, MAGIC, 2, 2026100710,
                         160, 1536, 3, 1, 1, 0, 40, 20, 1)
        raw_rows = bytearray(3*160)
        for i, active in enumerate((1, 0, 1)):
            struct.pack_into('<6I', raw_rows, i*160, i*20,
                             active | ((3 if active else 0) << 8), 127, 0, 0, 0)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'historical.json'
            with contextlib.redirect_stdout(io.StringIO()):
                export(header_decode(raw_header, require_commit=False), raw_rows,
                       path, 'synthetic historical record')
            report = json.loads(path.read_text(encoding='utf-8'))
        self.assertEqual([r['active'] for r in report['records']], [True, False, True])
        self.assertTrue(report['summary']['multiple_control_runs'])
        self.assertTrue(report['summary']['restart_after_trigger'])
        self.assertEqual(report['summary']['active_run_count'], 2)
        self.assertTrue(report['integrity_warnings'])


if __name__ == '__main__':
    unittest.main()
