"""Selection checks use temporary files; no ROS, Docker or flight startup."""
import json
import tempfile
import unittest
from pathlib import Path

from check_complete import selected_errors, sha


class CompleteSelectionTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.path = self.root / 'local.json'
        self.local = {'completion_wall_budget_s': 300}
        self.lock = {'completion_wall_budget_s': 300, 'selected_configs': {}}
        for key in ('fleet_config', 'flight_config'):
            file = self.root / (key + '.json')
            file.write_text(json.dumps({'case': key}))
            self.local[key] = file.name
            self.lock['selected_configs'][key] = sha(file)

    def errors(self):
        return selected_errors(self.path, self.local, self.lock)

    def test_relative_paths_resolve_beside_local_without_creating_run(self):
        self.assertEqual(self.errors(), [])

    def test_absolute_paths_accepted(self):
        self.local['fleet_config'] = str(self.root / self.local['fleet_config'])
        self.assertEqual(self.errors(), [])

    def test_default_short_configuration_is_not_silently_selected(self):
        del self.local['fleet_config']
        self.assertIn('explicit configuration required: fleet_config', self.errors())

    def test_changed_flight_configuration_rejected(self):
        (self.root / self.local['flight_config']).write_text('{}')
        self.assertIn('selected configuration differs: flight_config', self.errors())

    def test_missing_configuration_rejected(self):
        (self.root / self.local['fleet_config']).unlink()
        self.assertIn('selected configuration differs: fleet_config', self.errors())

    def test_changed_or_invalid_budget_rejected(self):
        for value in (None, True, 20, 301, '300', float('nan')):
            with self.subTest(value=value):
                self.local['completion_wall_budget_s'] = value
                self.assertIn('completion budget differs from validated case', self.errors())


if __name__ == '__main__':
    unittest.main(verbosity=2)
