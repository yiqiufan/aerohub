import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = REPO_ROOT / "src" / "exploration_manager" / "launch" / "racer_pillar.launch.py"
FSM_SOURCE = (
    REPO_ROOT / "src" / "exploration_manager" / "src" / "fast_exploration_fsm.cpp"
)


class RacerPillarLaunchTest(unittest.TestCase):
    def test_grid_text_markers_are_disabled_by_default_and_opt_in(self):
        source = LAUNCH_FILE.read_text()
        compact_source = " ".join(source.split())

        self.assertIn("def planner_params(", source)
        self.assertIn("show_grid_text", source)
        self.assertIn(
            '"visualization.show_grid_text": ParameterValue(show_grid_text, value_type=bool),',
            source,
        )
        self.assertIn('DeclareLaunchArgument(\n            "show_grid_text"', source)
        self.assertIn('default_value="false"', source)
        self.assertIn(
            "drone_nodes(drone_id, init_x, init_y, coverage_metrics, "
            "coverage_report_interval_sec, show_grid_text)",
            compact_source,
        )

    def test_grid_text_drawing_is_guarded_by_parameter(self):
        source = FSM_SOURCE.read_text()

        self.assertIn('getParam<bool>(node_, "visualization.show_grid_text", false)', source)
        self.assertIn("if (show_grid_text_)", source)
