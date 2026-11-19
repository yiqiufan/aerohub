import io,json,tempfile,unittest,time
from pathlib import Path
from unittest.mock import patch
import rclpy
from geometry_msgs.msg import Point
from bspline.msg import Bspline
from rosidl_runtime_py.convert import message_to_ordereddict
from flight_adapter import FlightAdapter
from common import VEHICLE,NS
from online_adapter import OnlineAdapter
from trajectory import Plan
from base_mission import Mission

class ExecutionContract(unittest.TestCase):
    def setUp(self):rclpy.init()
    def tearDown(self):rclpy.try_shutdown()
    def test_real_adapter_namespace_translation_and_enable_gate(self):
        log=io.StringIO();node=FlightAdapter(log)
        try:
            msg=Bspline();msg.drone_id=VEHICLE['racer_id'];msg.traj_id=1;msg.order=3
            msg.pos_pts=[Point(x=0.,y=-3.,z=.75) for _ in range(5)];msg.knots=[float(x) for x in range(9)]
            msg.yaw_pts=[0.]*5;msg.yaw_dt=1.;msg.start_time.sec=10
            node.registration={'offset':[0.,-3.,0.]};node.input_rx=time.monotonic()
            with patch.object(OnlineAdapter,'on_plan') as downstream:
                node.on_plan(msg);downstream.assert_not_called()
                node.phase='tracking';node.on_plan(msg)
                converted=downstream.call_args.args[0]
            self.assertEqual(converted.pos_pts[0].y,0.);self.assertEqual(msg.pos_pts[0].y,-3.)
            self.assertEqual(converted.knots,msg.knots);self.assertEqual(converted.start_time,msg.start_time)
            plan=Plan.parse(dict(vehicle=VEHICLE['racer_id'],received_ns=0,message=message_to_ordereddict(converted)))
            goal=node.goal_for(plan)
            self.assertEqual(goal.path.vehicle_id,NS);self.assertEqual(goal.path.frame_id,NS+'/odom')
            self.assertEqual(goal.path.valid_from.sec,10)
            self.assertIsNone(node.pending);self.assertIsNone(node.goal_handle)
        finally:node.destroy_node()
    def test_workstation_clients_target_configured_vehicle(self):
        with tempfile.TemporaryDirectory() as root:
            n=Mission(Path(root),False,NS,VEHICLE['airsim'],VEHICLE['px4_system_id'],'drone1' if NS=='drone0' else 'drone0','/racer_multi/'+NS+'/adapter')
            try:
                self.assertEqual(n.direct_land.topic_name,'/'+NS+'/fmu/in/vehicle_command')
                self.assertEqual(n.adapter.request.topic_name,'/racer_multi/'+NS+'/adapter/command')
                topics={s.topic_name for s in n.subscriptions}
                self.assertIn('/'+NS+'/fmu/out/vehicle_status_v4',topics)
                self.assertIn('/'+NS+'/navigation/odom',topics)
            finally:n.e.shutdown();n.log.close();n.destroy_node()
if __name__=='__main__':unittest.main()
