import unittest
from stop_evidence import StopEvidence, StopPolicy


class StopChecks(unittest.TestCase):
    def make(self, reviewed=True):
        e = StopEvidence(StopPolicy(position_hold_contract="reviewed PID" if reviewed else ""),
                         10., 100_000_000_000, True)
        e.action_terminal = True
        e.acknowledge(True, 10.2)
        e.mode, e.mode_at = 2, 10.1
        return e

    def state(self, e):
        return e.check(10.21, 100_210_000_000)[0]

    def test_position_hold(self):
        self.assertEqual(self.state(self.make()), "confirmed")

    def test_direct_hover(self):
        e = self.make(False); e.mode = 1
        self.assertEqual(self.state(e), "confirmed")

    def test_unreviewed_position(self):
        self.assertEqual(self.state(self.make(False)), "waiting")

    def test_missing_action_terminal(self):
        e = self.make(); e.action_terminal = False
        self.assertEqual(self.state(e), "waiting")

    def test_stop_rejected(self):
        e = self.make(); e.acknowledge(False, 10.2)
        self.assertEqual(self.state(e), "failed")

    def test_stop_reply_absent(self):
        e = self.make(); e.ack_at = None
        self.assertEqual(self.state(e), "waiting")

    def test_mode_before_stop_is_not_confirmation(self):
        e = self.make(); e.mode_at = 9.9
        self.assertEqual(self.state(e), "waiting")

    def test_trajectory_mode_is_not_hold(self):
        e = self.make(); e.mode = 7
        self.assertEqual(self.state(e), "waiting")

    def test_frozen_ros_clock_timeout(self):
        self.assertEqual(self.make().check(16.01, 100_000_000_000)[0], "failed")

    def test_backward_ros_clock(self):
        self.assertEqual(self.make().check(10.02, 99_000_000_000)[0], "failed")


if __name__ == "__main__":
    unittest.main(verbosity=2)
