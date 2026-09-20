"""Hardware-free checks of the integrated approach, grasp and handoff sequence."""
import contextlib
import io
import subprocess
import unittest
from unittest.mock import Mock, patch

from retriever.mission import Arm, Heard, Seen, main, run_once
from retriever.teleop import ArmRunner


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.voice = Mock()
        self.detector = Mock()
        self.nav = Mock()
        self.arm = Mock()
        self.arm.pick.return_value = (True, 'picked')
        self.arm.handoff.return_value = (True, 'handed over')
        self.heard = Heard('fetch the goose', target='goose')

    def run_mission(self):
        return run_once(self.heard, voice=self.voice, detector=self.detector,
                        nav=self.nav, arm=self.arm)

    def test_complete_mission_requires_fresh_arrival_and_home(self):
        self.detector.locate.side_effect = [Seen(1, 0, 0.1), Seen(0.2, 0, 0.1)]
        self.nav.watch.side_effect = [iter([{'status': 'arrived'}]),
                                     iter([{'status': 'arrived'}])]
        self.assertTrue(self.run_mission())
        self.arm.pick.assert_called_once()
        self.nav.home.assert_called_once()
        self.arm.handoff.assert_called_once()

    def test_stale_or_nonfinite_arrival_does_not_start_arm(self):
        for age in (5.0, float('nan'), -1.0):
            with self.subTest(age=age):
                self.setUp()
                self.detector.locate.side_effect = [Seen(1, 0, 0.1), Seen(0.2, 0, age)]
                self.nav.watch.return_value = iter([{'status': 'arrived'}])
                self.assertFalse(self.run_mission())
                self.arm.pick.assert_not_called()
                self.nav.halt.assert_called_once()

    def test_blocked_nudge_does_not_start_arm(self):
        self.detector.locate.side_effect = [Seen(1, 0, 0.1), Seen(0.5, 0, 0.1)]
        self.nav.watch.side_effect = [iter([{'status': 'arrived'}]),
                                     iter([{'status': 'blocked'}])]
        self.assertFalse(self.run_mission())
        self.arm.pick.assert_not_called()
        self.nav.halt.assert_called_once()

    def test_blocked_return_does_not_handoff(self):
        self.detector.locate.side_effect = [Seen(1, 0, 0.1), Seen(0.2, 0, 0.1)]
        self.nav.watch.side_effect = [iter([{'status': 'arrived'}]),
                                     iter([{'status': 'blocked'}])]
        self.assertFalse(self.run_mission())
        self.arm.pick.assert_called_once()
        self.arm.handoff.assert_not_called()
        self.nav.halt.assert_called_once()

    def test_dry_run_exercises_the_whole_sequence(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(main(['--dry-run', '--once', 'fetch the goose']), 0)
        self.assertIn('POST /home', out.getvalue())
        self.assertIn('Here you go.', out.getvalue())

    def test_handoff_uses_configured_board_interpreter(self):
        arm = Arm('robot@arm', python='/opt/arm/bin/python')
        with patch.object(arm, '_ssh', return_value=(True, '')) as ssh:
            self.assertTrue(arm.handoff()[0])
        self.assertIn('/opt/arm/bin/python ~/arm_poses.py', ssh.call_args.args[0])
        self.assertIn('--hold-gripper', ssh.call_args.args[0])

    def test_dashboard_handoff_uses_its_own_timeout(self):
        # Execute the worker synchronously, with SSH mocked: no thread or board.
        def immediate_thread(*, target, args, **kwargs):
            return Mock(start=lambda: target(*args))
        arm = ArmRunner('robot@arm', timeout_s=90)
        with patch('retriever.teleop.threading.Thread', side_effect=immediate_thread), \
             patch('subprocess.run', return_value=subprocess.CompletedProcess([], 0, '', '')) as run:
            arm.handoff()
        self.assertEqual(run.call_args.kwargs['timeout'], 40.0)
        self.assertFalse(arm.state()['busy'])


if __name__ == '__main__':
    unittest.main()
