import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "evaluate_hybridfusion_runs.py"
SPEC = importlib.util.spec_from_file_location("evaluate_hybridfusion_runs", SCRIPT)
EVALUATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EVALUATOR)


def test_missing_result_is_retained_as_failed_run(tmp_path):
    matrix = tmp_path / "run_matrix.tsv"
    matrix.write_text(
        "run\tmethod\texit_code\tresult\n"
        "run_01\thybrid\t139\trun_01/hybrid/result.json\n",
        encoding="utf-8",
    )
    rows = EVALUATOR.collect(tmp_path)
    assert len(rows) == 1
    assert rows[0]["converged"] is False
    assert rows[0]["failure_reason"] == "process_exit_139_without_result"
    summary = EVALUATOR.summarize(rows)
    assert summary["methods"]["hybrid"]["failed"] == 1
    output = tmp_path / "summary.md"
    EVALUATOR.write_markdown(output, summary)
    assert "n/a" in output.read_text(encoding="utf-8")
