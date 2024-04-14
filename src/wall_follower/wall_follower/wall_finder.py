from geometry_msgs.msg import Twist
from new_interface.srv import FindWall
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import math

class WallFinder(Node):

    def __init__(self):
        super().__init__('wall_finding_server')


        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        

        self.wall_find_srv = self.create_service(FindWall, 'find_wall', self.empty_callback, callback_group=self.group1)

        self.mov_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.laser_sub= self.create_subscription(LaserScan, '/scan', self.laser_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=self.group2)
        
        self.timer_period= 0.5
        self.laser_front= 0
        self.laser_right= 0
        self.min_range= 100
        self.min_index= 0
        self.turn_complete= False
        self.parallel_complete = False
        self.approach_complete = False
        
        
        self.cmd= Twist()



    def laser_callback(self,msg):
        self.laser_right =  msg.ranges[180]
        #270 for sim 180 for actual robot
        self.laser_front = msg.ranges[359]
        ranges= msg.ranges
        self.min_range = min(ranges)
        self.min_index = ranges.index(self.min_range)

    def turn_to_wall(self):
        while self.turn_complete==False:
            self.get_logger().info('Turning towards wall "%s"' % str(self.min_index))

            if 362<= self.min_index <=720:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.3
                self.get_logger().info('Turning')
            elif 0 < self.min_index < 355:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = -0.3
                self.get_logger().info('Turning')
            elif 357 < self.min_index < 362:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.turn_complete = True
                self.get_logger().info('Stop')
                break

            self.mov_pub.publish(self.cmd)
            time.sleep(0.1)

    def parallel_wall(self):
        while self.parallel_complete == False:
            self.get_logger().info('turning parallel to wall: "%s"' % str(self.min_index))
            if 0 < self.min_index < 200 or 220 < self.min_index < 540:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.3
            if 540 < self.min_index < 720:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = -0.3
            elif 200 <= self.min_index <= 220:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.parallel_complete = True
                break

            
            self.mov_pub.publish(self.cmd)
            time.sleep(0.1)
    
    def stop(self):
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.mov_pub.publish(self.cmd)
        time.sleep(0.1)

    def approach_wall(self):
        while self.approach_complete == False:
            self.get_logger().info('Distance between robot and wall: "%s"' %str(self.laser_front))
            if self.laser_front > 0.28 :
                self.cmd.linear.x = 0.03
                self.cmd.angular.z = 0.0
            elif 90 < self.laser_front <= 0.27:
                self.cmd.linear.x = -0.01
                self.cmd.angular.z = 0.0
            else:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.mov_pub.publish(self.cmd)
                self.parallel_complete =False
                break
            self.mov_pub.publish(self.cmd)
            time.sleep(0.1)


    def empty_callback(self, request, response):
        self.get_logger().info('Front distance: "%s"' % str(self.laser_front))
        self.get_logger().info('Min range"%s"' % str(self.min_range))
        self.get_logger().info('Min_index "%s"' % str(self.min_index))
        self.get_logger().info('Robot is now moving')
        
        self.turn_to_wall()
        self.approach_wall()
        self.parallel_wall()
        self.stop()
        response.wallfound= True
        self.get_logger().info('Mission complete status: "%s"' %str(response.wallfound))
        return response

def main(args=None):
    rclpy.init(args=args)
    wall_finder_node = WallFinder()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(wall_finder_node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        wall_finder_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()