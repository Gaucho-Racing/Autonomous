from pathlib import Path
import runpy
from types import ModuleType
import unittest
from unittest.mock import Mock, call, patch


SOURCE = Path(__file__).resolve().parents[1] / "record_zed.py"


class ZedRecordingTest(unittest.TestCase):
    def test_startup_failures_close_camera_before_retry(self):
        for phase in ("open", "enable_recording"):
            failed, recovered = Mock(), Mock()
            failed.open.return_value = recovered.open.return_value = 0
            failed.enable_recording.return_value = recovered.enable_recording.return_value = 0
            getattr(failed, phase).return_value = 1
            failed.grab.side_effect = AssertionError("Failed camera must not grab")
            recovered.grab.side_effect = KeyboardInterrupt

            package, sdk = ModuleType("pyzed"), ModuleType("pyzed.sl")
            package.sl = sdk
            sdk.InitParameters = Mock()
            sdk.RecordingParameters = Mock()
            sdk.ERROR_CODE = Mock(SUCCESS=0)
            sdk.RESOLUTION = Mock()
            sdk.DEPTH_MODE = Mock()
            sdk.SVO_COMPRESSION_MODE = Mock()
            events = Mock()
            events.attach_mock(failed, "failed")
            events.attach_mock(recovered, "recovered")
            sdk.Camera = events.camera
            sdk.Camera.side_effect = [failed, recovered]

            with self.subTest(phase=phase), \
                    patch.dict("sys.modules", {"pyzed": package, "pyzed.sl": sdk}), \
                    patch("os.makedirs"), patch("time.sleep", events.sleep), \
                    patch("builtins.print"):
                runpy.run_path(str(SOURCE), run_name="__main__")

            events.assert_has_calls([call.failed.close(), call.sleep(2), call.camera()])
            failed.close.assert_called_once_with()
            failed.grab.assert_not_called()
            failed.disable_recording.assert_not_called()
            if phase == "open":
                failed.enable_recording.assert_not_called()
            recovered.disable_recording.assert_called_once_with()
            recovered.close.assert_called_once_with()

    def test_recording_cleanup_on_exit(self):
        for outcome in ("interrupt", "failed_status", "exception", "startup_exception", "finalization_exception", "open_exception"):
            camera = Mock()
            camera.open.return_value = 0
            camera.enable_recording.return_value = 0
            if outcome == "open_exception":
                camera.open.side_effect = RuntimeError("Camera open raised")
            elif outcome == "startup_exception":
                camera.enable_recording.side_effect = RuntimeError("Recording startup raised")
            elif outcome in ("interrupt", "finalization_exception"):
                camera.grab.side_effect = KeyboardInterrupt
                if outcome == "finalization_exception":
                    camera.disable_recording.side_effect = RuntimeError("Finalization raised")
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
                if outcome in ("open_exception", "startup_exception"):
                    message = "Camera open raised" if outcome == "open_exception" else "Recording startup raised"
                    with self.assertRaisesRegex(RuntimeError, message):
                        runpy.run_path(str(SOURCE), run_name="__main__")
                    camera.close.assert_called_once_with()
                    camera.disable_recording.assert_not_called()
                    camera.grab.assert_not_called()
                    if outcome == "open_exception":
                        camera.enable_recording.assert_not_called()
                    continue
                if outcome == "exception":
                    with self.assertRaisesRegex(RuntimeError, "Grab raised"):
                        runpy.run_path(str(SOURCE), run_name="__main__")
                elif outcome == "finalization_exception":
                    with self.assertRaisesRegex(RuntimeError, "Finalization raised"):
                        runpy.run_path(str(SOURCE), run_name="__main__")
                elif outcome == "failed_status":
                    with self.assertRaisesRegex(RuntimeError, "Grab failed.*1"):
                        runpy.run_path(str(SOURCE), run_name="__main__")
                else:
                    runpy.run_path(str(SOURCE), run_name="__main__")
                camera.disable_recording.assert_called_once_with()
                camera.close.assert_called_once_with()
                self.assertEqual(camera.method_calls[-2:], [call.disable_recording(), call.close()])
                mkdir.assert_called_once()


if __name__ == "__main__":
    unittest.main()
