"""Offline selection tests: no containers or vehicle services are started."""
import ast,importlib.util,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('selected_flight',HERE/'run.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class SelectionTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name).resolve()
        self.local=dict(run_root=str(self.root/'run'),release=str(self.root/'release'),project='test-config')
    def create(self):
        p=self.root/'local.json';p.write_text(json.dumps(self.local));return module.FlightDeployment(p)
    def selected(self):
        self.fleet=json.loads((HERE.parent/'racer_multi_ground/fleet.json').read_text())
        self.flight=json.loads((HERE/'flight.json').read_text())
        self.fleet['planning_box_enu'][0][1]=7;self.flight['planning_box_enu'][0][1]=7
        self.local.update(fleet_config='fleet.json',flight_config='flight.json',completion_wall_budget_s=300)
        (self.root/'fleet.json').write_text(json.dumps(self.fleet));(self.root/'flight.json').write_text(json.dumps(self.flight))
    def test_default_selection_unchanged(self):
        d=self.create();self.assertEqual(d.flight_config,json.loads((HERE/'flight.json').read_text()))
        self.assertEqual(d.fleet,json.loads((HERE.parent/'racer_multi_ground/fleet.json').read_text()))
    def test_relative_paths_resolved_at_local_file(self):
        self.selected();d=self.create()
        self.assertEqual(d.fleet_path,self.root/'fleet.json');self.assertEqual(d.flight_path,self.root/'flight.json')
        self.assertEqual(d.fleet['planning_box_enu'][0][1],7);self.assertEqual(d.validate_completion_config(),300)
        self.assertTrue(Path(d.local['fleet_config']).is_absolute());self.assertTrue(Path(d.local['flight_config']).is_absolute())
    def test_mismatched_boxes_rejected_before_start(self):
        self.selected();self.flight['planning_box_enu'][0][1]=4
        (self.root/'flight.json').write_text(json.dumps(self.flight))
        with self.assertRaisesRegex(ValueError,'boxes differ'):self.create()
        self.assertFalse((self.root/'run').exists())
    def test_changed_flight_rejected_before_base_up(self):
        self.selected();d=self.create();(self.root/'flight.json').write_text('{}')
        with patch.object(module.Deployment,'up') as launch:
            with self.assertRaisesRegex(ValueError,'flight configuration changed'):d.up()
            launch.assert_not_called()
    def test_changed_fleet_rejected_without_run_directory(self):
        self.selected();d=self.create();(self.root/'fleet.json').write_text('{}')
        with self.assertRaisesRegex(ValueError,'fleet configuration changed'):module.Deployment.up(d)
        self.assertFalse(d.root.exists())
    def test_selected_flight_and_budget_are_written(self):
        self.selected();d=self.create()
        def base_up(obj):
            (obj.root/'configuration').mkdir(parents=True)
            (obj.root/'configuration-sources.json').write_text('{}')
        with patch.object(module.Deployment,'up',base_up):d.up()
        data=json.loads((d.root/'configuration/flight.json').read_text())
        self.assertEqual(data['planning_box_enu'][0][1],7);self.assertEqual(data['completion_wall_budget_s'],300)
        self.assertEqual(json.loads((d.root/'configuration-sources.json').read_text())['flight']['sha256'],d.flight_source_sha)
    def test_renderer_uses_selected_fleet(self):
        self.selected();root=self.root
        (root/'fleet-input.json').write_text(json.dumps(self.fleet))
        tree=ast.parse((HERE.parent/'racer_multi_ground/prepare_config.py').read_text())
        assignments=[n for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id in ('fleet_source','fleet') for t in n.targets)]
        env=dict(root=root,Path=Path,json=json);exec(compile(ast.Module(body=assignments,type_ignores=[]),'renderer','exec'),env)
        self.assertEqual(env['fleet'],self.fleet)

if __name__=='__main__':unittest.main()
