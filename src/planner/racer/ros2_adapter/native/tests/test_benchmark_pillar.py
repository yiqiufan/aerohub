import csv
import importlib.util
import unittest
from pathlib import Path


def load_benchmark_module():
    module_path = Path(__file__).resolve().parents[1] / "tools" / "benchmark_pillar.py"
    spec = importlib.util.spec_from_file_location("benchmark_pillar", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class BenchmarkPillarCoverageTest(unittest.TestCase):
    def test_run_log_tracks_latest_coverage_by_drone(self):
        benchmark = load_benchmark_module()
        run_log = benchmark.RunLog()

        run_log.feed(
            "[exploration_node_1]: Map coverage percent: 70.000 known_cells=499800 total_cells=714000"
        )
        run_log.feed(
            "[exploration_node_2]: Map coverage percent: 77.500 known_cells=553350 total_cells=714000"
        )
        run_log.feed(
            "[exploration_node_1]: Map coverage percent: 75.000 known_cells=535500 total_cells=714000"
        )

        self.assertEqual(run_log.coverage[1].percent, 75.0)
        self.assertEqual(run_log.coverage[1].known_cells, 535500)
        self.assertEqual(run_log.coverage[1].total_cells, 714000)
        self.assertEqual(run_log.coverage_summary(), (76.25, 75.0, 77.5))

    def test_write_csv_includes_coverage_columns(self):
        benchmark = load_benchmark_module()
        output = Path(self._testMethodName + ".csv")
        self.addCleanup(output.unlink, missing_ok=True)

        benchmark.write_csv(
            output,
            [
                {
                    "run": 1,
                    "exploration_time_sec": 40.0,
                    "total_path_length_m": 150.0,
                    "wall_time_sec": 45.0,
                    "completion_mode": "no_grid",
                    "coverage_percent": 82.5,
                    "coverage_min_percent": 81.0,
                    "coverage_max_percent": 84.0,
                    "coverage_drone_1_percent": 81.0,
                    "coverage_drone_2_percent": 82.0,
                    "coverage_drone_3_percent": 83.0,
                    "coverage_drone_4_percent": 84.0,
                }
            ],
        )

        with output.open(newline="") as handle:
            rows = list(csv.DictReader(handle))

        self.assertEqual(rows[0]["coverage_percent"], "82.5")
        self.assertEqual(rows[0]["coverage_drone_4_percent"], "84.0")
