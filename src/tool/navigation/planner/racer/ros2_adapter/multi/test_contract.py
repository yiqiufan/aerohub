import io,unittest
import rclpy
from geometry_msgs.msg import Point
from bspline.msg import Bspline
from rosidl_runtime_py.convert import message_to_ordereddict
from camera_pose import CameraPoseAdapter
from common import translation,to_local_message
from trajectory import Plan

class Contract(unittest.TestCase):
    def test_nonzero_translation_preserves_curve_derivatives_time_yaw(self):
        msg=Bspline();msg.drone_id=3;msg.traj_id=7;msg.order=3
        msg.pos_pts=[Point(x=float(i),y=float(i*i),z=1.) for i in range(5)]
        msg.knots=[float(i) for i in range(9)];msg.yaw_pts=[.1,.2,.3,.4,.5];msg.yaw_dt=1.
        msg.start_time.sec=123;msg.start_time.nanosec=456
        offset=translation([2.,3.,4.],[-1.,5.,0.]);local=to_local_message(msg,offset)
        def parse(m):return Plan.parse({'vehicle':3,'received_ns':0,'message':message_to_ordereddict(m)})
        world,converted=parse(msg),parse(local)
        for t in [0.,.2,.7,1.4,2.]:
            a,b=world.sample(t),converted.sample(t)
            for axis in range(3):
                self.assertAlmostEqual(a[0][axis]-offset[axis],b[0][axis])
                self.assertAlmostEqual(a[1][axis],b[1][axis]);self.assertAlmostEqual(a[2][axis],b[2][axis])
            self.assertAlmostEqual(a[3],b[3])
        self.assertEqual(world.start,converted.start);self.assertEqual(local.drone_id,3)
        self.assertEqual(msg.pos_pts[0].x,0.)
    def test_default_and_other_vehicle_camera_contract(self):
        rclpy.init()
        nodes=[]
        try:
            old=CameraPoseAdapter(io.StringIO(),mapping_path='/single/config/mapper.yaml');nodes.append(old)
            new=CameraPoseAdapter(io.StringIO(),'drone3','/racer_multi/drone3','/single/config/mapper.yaml');nodes.append(new)
            self.assertEqual(old.odom_frame,'drone0/odom');self.assertEqual(old.depth_out.topic_name,'/racer_ground/depth')
            self.assertEqual(new.optical_frame,'drone3/front_center_optical');self.assertEqual(new.pose_out.topic_name,'/racer_multi/drone3/camera_pose')
        finally:
            for n in nodes:n.destroy_node()
            rclpy.try_shutdown()
    def test_real_shadow_stop_subscription_and_input_clock(self):
        from shadow import Shadow
        from inputs import Inputs
        from std_msgs.msg import Empty
        import time
        rclpy.init()
        node=Shadow(io.StringIO());inputs=Inputs(io.StringIO())
        try:
            self.assertTrue(inputs.get_parameter('use_sim_time').value)
            pub=node.create_publisher(Empty,'/racer_multi/drone0/operator_stop',10)
            deadline=time.monotonic()+3
            while not node.stopped and time.monotonic()<deadline:
                pub.publish(Empty());rclpy.spin_once(node,timeout_sec=.05)
            self.assertTrue(node.stopped)
            self.assertTrue(node.disabled)
            self.assertIsNone(node.goal_handle)
            self.assertIsNone(node.pending)
            from exploration_manager.msg import TrajectoryExecution
            statuses=[]
            node.report=lambda identity,status,detail:statuses.append(status)
            node.on_stop(Empty())
            self.assertEqual(statuses,[TrajectoryExecution.OPERATOR_STOPPED])
        finally:
            node.destroy_node();inputs.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':unittest.main()
