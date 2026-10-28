import importlib.util
from pathlib import Path
from types import ModuleType
import unittest
from unittest.mock import Mock, patch


SOURCE = Path(__file__).resolve().parents[2] / "Python Detection Code" / "WebcamDetection.py"
SPEC = importlib.util.spec_from_file_location("webcam_detection", SOURCE)
WEBCAM = importlib.util.module_from_spec(SPEC)
CV2_STUB = ModuleType("cv2")
YOLO_STUB = ModuleType("ultralytics")
YOLO_STUB.YOLO = Mock()
with patch.dict("sys.modules", {"cv2": CV2_STUB, "ultralytics": YOLO_STUB}):
    SPEC.loader.exec_module(WEBCAM)


class WebcamDetectionTest(unittest.TestCase):
    def test_explicit_model_directory_is_rejected(self):
        with patch.object(Path, "exists", return_value=True), \
                patch.object(Path, "is_file", return_value=False):
            with self.assertRaises(FileNotFoundError):
                WEBCAM.resolve_model_path("/models/directory")

    def test_fallback_skips_directories(self):
        candidates = [Path("/models/directory"), Path("/models/best.pt")]
        with patch.object(WEBCAM, "DEFAULT_MODEL_CANDIDATES", candidates), \
                patch.object(Path, "exists", return_value=True), \
                patch.object(Path, "is_file", lambda path: path == candidates[1]):
            self.assertEqual(WEBCAM.resolve_model_path(""), candidates[1])

    def test_explicit_model_file_is_selected(self):
        model = Path("/models/best.pt")
        with patch.object(Path, "is_file", return_value=True), \
                patch.object(Path, "exists", return_value=True):
            self.assertEqual(WEBCAM.resolve_model_path(str(model)), model.resolve())

    def test_missing_models_report_searched_paths(self):
        with patch.object(Path, "exists", return_value=False), \
                patch.object(Path, "is_file", return_value=False):
            with self.assertRaisesRegex(FileNotFoundError, "Pass --model"):
                WEBCAM.resolve_model_path("")

    def test_invalid_inference_arguments_stop_before_loading(self):
        arguments = ["--conf=-0.1", "--conf=1.1", "--conf=nan", "--conf=inf",
                     "--conf=-inf", "--imgsz=0", "--imgsz=-1"]
        for argument in arguments:
            with self.subTest(argument=argument), \
                    patch("sys.argv", ["WebcamDetection.py", argument]), \
                    patch("sys.stderr"), \
                    patch.object(WEBCAM, "resolve_model_path", side_effect=AssertionError("Unexpected model lookup")) as resolve, \
                    patch.object(WEBCAM, "YOLO") as yolo, \
                    patch.object(WEBCAM, "cv2") as cv2:
                with self.assertRaises(SystemExit) as result:
                    WEBCAM.main()
                self.assertEqual(result.exception.code, 2)
                resolve.assert_not_called()
                yolo.assert_not_called()
                cv2.VideoCapture.assert_not_called()

    def test_valid_inference_argument_boundaries_reach_model_lookup(self):
        for arguments in ([], ["--conf=0", "--imgsz=1"], ["--conf=1", "--imgsz=640"]):
            with self.subTest(arguments=arguments), \
                    patch("sys.argv", ["WebcamDetection.py"] + arguments), \
                    patch.object(WEBCAM, "resolve_model_path", side_effect=RuntimeError("Reached model lookup")):
                with self.assertRaisesRegex(RuntimeError, "Reached model lookup"):
                    WEBCAM.main()

    def test_camera_cleanup_on_failures(self):
        for failure in ("open", "read", "predict"):
            with self.subTest(failure=failure):
                camera = Mock()
                camera.isOpened.return_value = failure != "open"
                camera.read.return_value = (failure != "read", Mock())
                cv2 = Mock()
                cv2.VideoCapture.return_value = camera
                model = Mock()
                model.predict.side_effect = ValueError("Inference failed")
                with patch.object(WEBCAM, "cv2", cv2), \
                        patch.object(WEBCAM, "YOLO", return_value=model), \
                        patch.object(WEBCAM, "resolve_model_path", return_value=Path("/model.pt")), \
                        patch("sys.argv", ["WebcamDetection.py"]), \
                        patch("builtins.print"):
                    if failure == "open":
                        with self.assertRaisesRegex(RuntimeError, "Could not open webcam"):
                            WEBCAM.main()
                    elif failure == "predict":
                        with self.assertRaisesRegex(ValueError, "Inference failed"):
                            WEBCAM.main()
                    else:
                        WEBCAM.main()
                camera.release.assert_called_once_with()
                cv2.destroyAllWindows.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
