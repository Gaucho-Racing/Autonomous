import importlib.util
import fnmatch
from pathlib import Path
from types import ModuleType
import unittest
from unittest.mock import Mock, mock_open, patch


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

    def test_nonfinite_boxes_are_skipped_without_losing_valid_boxes(self):
        valid = {"classTitle": "yellow_cone", "points": {"exterior": [[20, 20], [80, 80]]}}
        for coordinate in range(4):
            for value in (float("nan"), float("inf"), -float("inf")):
                corners = [[20, 20], [80, 80]]
                corners[coordinate // 2][coordinate % 2] = value
                invalid = {"classTitle": "blue_cone", "points": {"exterior": corners}}
                with self.subTest(coordinate=coordinate, value=value):
                    self.assertEqual(
                        CONVERTER.supervisely_to_yolo({"objects": [invalid, valid]}, 100, 100),
                        ["1 0.500000 0.500000 0.600000 0.600000"],
                    )

    def test_conversion_uses_distinct_repeatable_output_names(self):
        root = Path("/dataset")
        images = [
            root / "team_a" / "frame.jpg",
            root / "team_b" / "frame.jpg",
            root / "team_c" / "img" / "frame.jpg",
            root / "team_c" / "img" / "frame.png",
        ]
        samples = [(image, image.with_suffix(".json")) for image in images]
        image_api = Mock()
        image_api.open.return_value.__enter__ = Mock(return_value=Mock(size=(100, 100)))
        image_api.open.return_value.__exit__ = Mock(return_value=False)
        with patch.object(CONVERTER, "FSOCO_DIR", root), \
                patch.object(CONVERTER, "TRAIN_RATIO", 1.0), \
                patch.object(CONVERTER, "reset_output_dirs") as reset, \
                patch.object(CONVERTER, "find_samples", return_value=samples), \
                patch.object(CONVERTER.random, "shuffle"), \
                patch.object(CONVERTER, "tqdm", side_effect=lambda values: values), \
                patch.object(CONVERTER, "Image", image_api), \
                patch.object(Path, "exists", return_value=True), \
                patch.object(Path, "open", mock_open(read_data='{"objects": []}')), \
                patch.object(CONVERTER, "supervisely_to_yolo", return_value=["0 0.5 0.5 0.2 0.2"]), \
                patch.object(CONVERTER.shutil, "copy") as copy, \
                patch.object(Path, "write_text", autospec=True) as write, \
                patch.object(CONVERTER, "write_data_config"), \
                patch("builtins.print"):
            CONVERTER.convert_dataset()
            first_names = [call.args[1] for call in copy.call_args_list]
            self.assertEqual(len(first_names), len(images))
            self.assertEqual(len({name.stem for name in first_names}), len(images))
            self.assertEqual(write.call_count, len(images))
            self.assertEqual(
                [call.args[0].name for call in write.call_args_list],
                [name.with_suffix(".txt").name for name in first_names],
            )
            CONVERTER.convert_dataset()
            self.assertEqual([call.args[1] for call in copy.call_args_list[len(images):]], first_names)
            self.assertEqual(reset.call_count, 2)

    def test_discovery_filters_and_sorts_images(self):
        root = Path("/dataset")
        team_a, team_z = root / "a", root / "z"
        images_a = [team_a / "img" / name for name in (
            "c.PNG", "b.JPEG", "a.jpg", "backup.jpg.tmp", "folder.png", "unpaired.jpg"
        )]
        image_z = team_z / "img" / "z.jpeg"
        listings = {
            root: [team_z, team_a],
            team_a / "img": images_a,
            team_z / "img": [image_z],
        }
        annotations = {
            team_a / "ann" / "a.jpg.json",
            team_a / "ann" / "b.json",
            team_a / "ann" / "c.json",
            team_a / "ann" / "backup.jpg.json",
            team_a / "ann" / "folder.json",
            team_z / "ann" / "z.json",
        }
        directories = {root, team_a, team_z, team_a / "img", team_a / "ann",
                       team_z / "img", team_z / "ann", images_a[4],
                       team_a / "ann" / "a.json", team_a / "ann" / "unpaired.jpg.json"}
        with patch.object(CONVERTER, "FSOCO_DIR", root), \
                patch.object(Path, "iterdir", lambda path: iter(listings[path])), \
                patch.object(Path, "glob", lambda path, pattern: (
                    item for item in listings[path] if fnmatch.fnmatchcase(item.name, pattern)
                )), \
                patch.object(Path, "is_dir", lambda path: path in directories), \
                patch.object(Path, "is_file", lambda path: (path in annotations or path in images_a or path == image_z) and path not in directories), \
                patch.object(Path, "exists", lambda path: path in directories or path in annotations):
            samples = CONVERTER.find_samples()
        self.assertEqual(samples, [
            (images_a[2], team_a / "ann" / "a.jpg.json"),
            (images_a[1], team_a / "ann" / "b.json"),
            (images_a[0], team_a / "ann" / "c.json"),
            (image_z, team_z / "ann" / "z.json"),
        ])


if __name__ == "__main__":
    unittest.main()
