from pathlib import Path
import runpy
from types import ModuleType
import unittest
from unittest.mock import Mock, call, patch


SOURCE = Path(__file__).resolve().parents[1] / "record_zed.py"


class ZedRecordingTest(unittest.TestCase):
    def test_recording_cleanup_on_exit(self):
        for outcome in ("interrupt", "failed_status", "exception", "startup_exception"):
            camera = Mock()
            camera.open.return_value = 0
            camera.enable_recording.return_value = 0
            if outcome == "startup_exception":
                camera.enable_recording.side_effect = RuntimeError("Recording startup raised")
            elif outcome == "interrupt":
                camera.grab.side_effect = KeyboardInterrupt
            elif outcome == "failed_status":
                camera.grab.return_value = 1
            else:
                camera.grab.side_effect = RuntimeError("Grab raised")
            package, sdk = ModuleType("pyzed"), ModuleType("pyzed.sl")
            package.sl = sdk
            sdk.Camera = Mock(return_value=camera)
            sdk.InitParameters = Mock()
            sdk.RecordingParameters = Mock()
            sdk.ERROR_CODE = Mock(SUCCESS=0)
            sdk.RESOLUTION = Mock(HD720="HD720")
            sdk.DEPTH_MODE = Mock(NONE="NONE")
            sdk.SVO_COMPRESSION_MODE = Mock(H264="H264")
            with self.subTest(outcome=outcome), \
                    patch.dict("sys.modules", {"pyzed": package, "pyzed.sl": sdk}), \
                    patch("os.makedirs") as mkdir, \
                    patch("time.sleep", side_effect=AssertionError("Unexpected retry")), \
                    patch("builtins.print"):
                if outcome == "startup_exception":
                    with self.assertRaisesRegex(RuntimeError, "Recording startup raised"):
                        runpy.run_path(str(SOURCE), run_name="__main__")
                    camera.close.assert_called_once_with()
                    camera.disable_recording.assert_not_called()
                    camera.grab.assert_not_called()
                    continue
                if outcome == "exception":
                    with self.assertRaisesRegex(RuntimeError, "Grab raised"):
                        runpy.run_path(str(SOURCE), run_name="__main__")
                elif outcome == "failed_status":
                    with self.assertRaisesRegex(RuntimeError, "Grab failed"):
                        runpy.run_path(str(SOURCE), run_name="__main__")
                else:
                    runpy.run_path(str(SOURCE), run_name="__main__")
                camera.disable_recording.assert_called_once_with()
                camera.close.assert_called_once_with()
                self.assertEqual(camera.method_calls[-2:], [call.disable_recording(), call.close()])
                mkdir.assert_called_once()


if __name__ == "__main__":
    unittest.main()
