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


if __name__ == "__main__":
    unittest.main()
