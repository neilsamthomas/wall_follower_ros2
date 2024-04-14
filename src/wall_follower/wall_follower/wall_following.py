import rclpy
import numpy as np
import time

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from new_interface.srv import FindWall
from new_interface.action import OdomRecord

class ClientSync(Node):
    def __init__(self):
        super().__init__('find_wall_client')
        self.client = self.create_client(FindWall,'find_wall')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available')
        self.req = FindWall.Request()

    def send_request(self):
        self.future = self.client.call_async(self.req)



class WallFollower(Node):

    def __init__(self):
       
        super().__init__('wall_follower')

        #rclpy.logging.set_logger_level('wall_follower', rclpy.logging.LoggingSeverity.DEBUG)

        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = MutuallyExclusiveCallbackGroup()


        # Wall follower functionality
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser_subscriber_ = self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=self.group1)
        
        self.srv_client = self.create_client(FindWall,'find_wall')
        while not self.srv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available')
        self.req = FindWall.Request()

        self.action_client = ActionClient(self, OdomRecord, 'record_odom',callback_group=self.group3)
        
        self.timer_period = 0.5
        self.publisher_timer = self.create_timer(self.timer_period, self.motion, callback_group=self.group2)


    def send_goal(self):
        goal_msg = OdomRecord.Goal()
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: "%s"'%str(result.list_of_odoms))
        self.cmd.linear.x= 0.0
        self.cmd.angular.z= 0.0
        self.publisher_.publish(self.cmd)
        self.destroy_node()
        self.destroy.action_client()
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info("===================================")
        self.get_logger().info('Received feedback, Current Total: {0}'.format(feedback.current_total))
        self.get_logger().info("===================================")



    def motion(self):

        #self.get_logger().info('I receive: "%s"'%str(self.laser_right))
        #self.get_logger().info('I front "%s"'%str(self.laser_front))
        self.cmd= Twist()

        if self.laser_front < 0.4:
            self.cmd.linear.x=0.05
            self.cmd.angular.z= 0.8
            self.get_logger().info('Making Turn')

        else:

            if self.laser_right > 0.3 :
                self.cmd.linear.x=0.05
                self.cmd.angular.z=  -0.09
                self.get_logger().info('Moving Towards Wall')

            elif self.laser_right < 0.2:
                self.cmd.linear.x=0.05
                self.cmd.angular.z= 0.09
                self.get_logger().info('Moving Away Wall')

            else:
                self.cmd.linear.x=0.03
                self.cmd.angular.z = 0.00
                    
                self.get_logger().info('Straight')
            
        
        self.publisher_.publish(self.cmd)

        
    def laser_callback(self, msg):
        #self.laser_right = msg.ranges[270]
        self.laser_right = msg.ranges[180] #for remote robot
        self.laser_front = msg.ranges[359]


    

def main(args=None):
    rclpy.init(args=args)
    client = ClientSync()
    client.send_request()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response=client.future.result()
            except Exception as e:
                client.get_logger().info('Service call failed %r' %(e,))
            else:
                client.get_logger().info("Robot is ready to follow the wall")
            break
    client.destroy_node()

    executor = MultiThreadedExecutor(num_threads=4)
    wall_follow= WallFollower()
    executor.add_node(wall_follow)
    wall_follow.send_goal()

    try:
        executor.spin()
    finally:
        executor.shutdown()
        wall_follow.destroy_node()
    #rclpy.shutdown()
    

if __name__ == '__main__':
    main()