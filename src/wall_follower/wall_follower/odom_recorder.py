import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from new_interface.action import OdomRecord
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import time
import math
import copy

class OdomAction(Node):
    def __init__(self):
        super().__init__('odom_recorder_node')
        self.group1= MutuallyExclusiveCallbackGroup()
        self.group2= MutuallyExclusiveCallbackGroup()
        self.action_server = ActionServer(self, OdomRecord, 'record_odom', self.execute_callback, callback_group=self.group1)

        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE), callback_group=self.group2)

        self.laser_data = LaserScan()

        self.last_odom = Point()
        self.first_odom = Point()
        self.lap_status = False

        self.dist = 0.0
        self.distance_from_start = 0.0
        self.total_distance = 0.0
        self.odom_record = []

        self.last_x = 0.0
        self.last_y = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        a=0

    def odom_callback(self, msg):
        self.position_x = msg.pose.pose.position.x
        self.position_y = msg.pose.pose.position.y
        self.orientation_theta = msg.pose.pose.orientation.z
        self.position = msg.pose.pose.position

    def get_position(self):
        return self.position
    
    def get_orientation(self):
        return self.orientation_theta

    def get_current_dist(self):
        return self.current_dist

    def execute_callback(self, goal_handle):
        feedback_msg = OdomRecord.Feedback()
        self.get_orientation()
        self.get_position()

        self.first_odom.x = self.position_x
        self.first_odom.y = self.position_y
        self.first_odom.z = self.orientation_theta
        self.odom_record.append(copy.copy(self.first_odom))   #copy.copy
        a=0

        self.get_logger().info('First Point has been added : "%s"'% str(self.first_odom))
        

        while self.lap_status == False:
            time.sleep(1)
            result = OdomRecord.Result()
            self.get_position()
            self.get_orientation()
            self.last_odom.x = self.position_x
            self.last_odom.y = self.position_y
            self.last_odom.z = self.orientation_theta
            self.odom_record.append(copy.copy(self.last_odom)) # copy copy

            self.get_logger().info('Next point:"%s"'%str(self.last_odom))
            if a==0:
                self.last_x= self.first_odom.x
                self.last_y= self.first_odom.y
            a+= 1
            self.new_distance = math.sqrt((self.last_odom.x - self.last_x)**2 + (self.last_odom.y - self.last_y)**2)
            self.total_distance += self.new_distance
            feedback_msg.current_total = self.total_distance

            goal_handle.publish_feedback(feedback_msg)

            self.distance_from_start = math.sqrt((self.last_odom.x - self.first_odom.x)**2 + (self.last_odom.y - self.first_odom.y)**2)
            self.get_logger().info('Distance from starting point : "%s"'% str(self.distance_from_start))

            if self.total_distance>= 6 and self.distance_from_start <= 0.5:
                self.lap_status = True
                break

            self.last_x = self.last_odom.x
            self.last_y = self.last_odom.y

        goal_handle.succeed()
        result = OdomRecord.Result() 
        result.list_of_odoms = self.odom_record
        return result

def main(args=None):
    rclpy.init(args=args)
    actionsrv = OdomAction()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(actionsrv)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        actionsrv.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
