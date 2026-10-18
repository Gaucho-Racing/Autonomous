import importlib.util
from pathlib import Path
from types import ModuleType
import unittest
from unittest.mock import patch


SOURCE = Path(__file__).resolve().parents[2] / "Python Detection Code" / "ConvertToYOLO.py"
SPEC = importlib.util.spec_from_file_location("dataset_conversion", SOURCE)
CONVERTER = importlib.util.module_from_spec(SPEC)
PIL_STUB = ModuleType("PIL")
PIL_STUB.Image = None
TQDM_STUB = ModuleType("tqdm")
TQDM_STUB.tqdm = None
with patch.dict("sys.modules", {"PIL": PIL_STUB, "tqdm": TQDM_STUB}):
    SPEC.loader.exec_module(CONVERTER)


class DatasetConversionTest(unittest.TestCase):
    def labels_for(self, corners):
        annotation = {"objects": [{"classTitle": "blue_cone", "points": {"exterior": corners}}]}
        return CONVERTER.supervisely_to_yolo(annotation, 100, 100)

    def test_boxes_are_clipped_before_normalization(self):
        self.assertEqual(
            self.labels_for([[-20, 20], [40, 80]]),
            ["0 0.200000 0.500000 0.400000 0.600000"],
        )
        self.assertEqual(
            self.labels_for([[60, 20], [120, 80]]),
            ["0 0.800000 0.500000 0.400000 0.600000"],
        )

    def test_invisible_and_tiny_clipped_boxes_are_skipped(self):
        self.assertEqual(self.labels_for([[-40, 20], [-10, 80]]), [])
        self.assertEqual(self.labels_for([[-20, 20], [0.1, 80]]), [])

    def test_in_bounds_and_reversed_boxes_match(self):
        expected = ["0 0.500000 0.500000 0.600000 0.600000"]
        self.assertEqual(self.labels_for([[20, 20], [80, 80]]), expected)
        self.assertEqual(self.labels_for([[80, 80], [20, 20]]), expected)


if __name__ == "__main__":
    unittest.main()
