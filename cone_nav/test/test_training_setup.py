import importlib.util
from pathlib import Path
from types import ModuleType
import unittest
from unittest.mock import Mock, patch


SOURCE = Path(__file__).resolve().parents[2] / "Python Detection Code" / "ConeDetection.py"
SPEC = importlib.util.spec_from_file_location("training_setup", SOURCE)
TRAINING = importlib.util.module_from_spec(SPEC)
TORCH_STUB = ModuleType("torch")
YOLO_STUB = ModuleType("ultralytics")
YOLO_STUB.YOLO = Mock()
with patch.dict("sys.modules", {"torch": TORCH_STUB, "ultralytics": YOLO_STUB}):
    SPEC.loader.exec_module(TRAINING)


class TrainingSetupTest(unittest.TestCase):
    def test_checkpoint_selection_requires_files(self):
        weights = TRAINING.OUTPUT_DIR / TRAINING.RUN_NAME / "weights"
        last, best = weights / "last.pt", weights / "best.pt"
        for files, expected in (({last, best}, last), ({best}, best), (set(), None)):
            with self.subTest(files=files), \
                    patch.object(Path, "exists", return_value=True), \
                    patch.object(Path, "is_file", lambda path: path in files):
                self.assertEqual(TRAINING.existing_checkpoint(), expected)

    def test_missing_checkpoints_return_none(self):
        with patch.object(Path, "exists", return_value=False), \
                patch.object(Path, "is_file", return_value=False):
            self.assertIsNone(TRAINING.existing_checkpoint())


    def test_wrong_input_path_types_stop_before_training(self):
        train_images = TRAINING.DATA_DIR / "train" / "images"
        val_images = TRAINING.DATA_DIR / "val" / "images"
        for invalid in (TRAINING.DATA_CONFIG, train_images, val_images):
            def entries(path):
                if path == invalid:
                    raise NotADirectoryError(str(path))
                return iter([Path("image.jpg")])

            with self.subTest(invalid=invalid), \
                    patch.object(TRAINING, "check_device", return_value="cpu"), \
                    patch.object(TRAINING, "YOLO") as yolo, \
                    patch.object(Path, "exists", return_value=True), \
                    patch.object(Path, "is_file", lambda path: path == TRAINING.DATA_CONFIG and path != invalid), \
                    patch.object(Path, "is_dir", lambda path: path in (train_images, val_images) and path != invalid), \
                    patch.object(Path, "iterdir", entries), \
                    patch.object(Path, "unlink", side_effect=AssertionError("Unexpected file removal")) as unlink, \
                    patch("builtins.print"):
                TRAINING.train()
                yolo.assert_not_called()
                unlink.assert_not_called()


if __name__ == "__main__":
    unittest.main()
