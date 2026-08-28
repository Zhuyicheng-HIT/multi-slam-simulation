import csv
import importlib.util
from pathlib import Path


MODULE_PATH = Path(__file__).parents[1] / "tools" / "evaluate_dynamic_observer_csv.py"
SPEC = importlib.util.spec_from_file_location("dynamic_evaluator", MODULE_PATH)
EVALUATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EVALUATOR)


def write_rows(path, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["truth_label", "predicted_label"])
        writer.writeheader()
        writer.writerows(rows)


def test_pure_static_dynamic_metrics_are_not_applicable(tmp_path):
    path = tmp_path / "static.csv"
    write_rows(
        path,
        [
            {"truth_label": "static", "predicted_label": "static"},
            {"truth_label": "static", "predicted_label": "unknown"},
        ],
    )
    result = EVALUATOR.evaluate(path)
    assert result["dynamic_metrics_applicable"] is False
    assert result["dynamic_precision"] is None
    assert result["dynamic_recall"] is None
    assert result["dynamic_f1"] is None
    assert result["static_preservation_rate"] == 1.0


def test_positive_class_with_no_detection_is_zero_not_perfect(tmp_path):
    path = tmp_path / "miss.csv"
    write_rows(
        path,
        [
            {"truth_label": "dynamic", "predicted_label": "unknown"},
            {"truth_label": "dynamic", "predicted_label": "static"},
        ],
    )
    result = EVALUATOR.evaluate(path)
    assert result["dynamic_metrics_applicable"] is True
    assert result["dynamic_precision"] == 0.0
    assert result["dynamic_recall"] == 0.0
    assert result["dynamic_f1"] == 0.0
    assert result["static_map_contamination"] == 0.5


def test_dynamic_precision_counts_static_false_positives(tmp_path):
    path = tmp_path / "mixed.csv"
    write_rows(
        path,
        [
            {"truth_label": "dynamic", "predicted_label": "dynamic"},
            {"truth_label": "static", "predicted_label": "dynamic"},
        ],
    )
    result = EVALUATOR.evaluate(path)
    assert result["dynamic_precision"] == 0.5
    assert result["dynamic_recall"] == 1.0
    assert result["false_dynamic_ratio"] == 1.0
