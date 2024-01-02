import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from assessment_interfaces.msg import ItemList, ItemHolder, ItemHolders
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

from tf_transformations import euler_from_quaternion
import angles

from enum import Enum
import random
import math

LINEAR_VELOCITY  = 0.3 # Metres per second
ANGULAR_VELOCITY = 0.5 # Radians per second

TURN_LEFT = 1 # Postive angular velocity turns left
TURN_RIGHT = -1 # Negative angular velocity turns right

SCAN_THRESHOLD = 0.5 # Metres per second
 # Array indexes for sensor sectors
SCAN_FRONT = 0
SCAN_LEFT = 1
SCAN_BACK = 2
SCAN_RIGHT = 3

class State(Enum):
    FORWARD = 0
    TURNING = 1
    COLLECTING = 2
    RETURNING = 3
    SET_GOAL = 4
    NAVIGATING = 5

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value

        self.state = State.FORWARD
        self.pose = Pose()

        self.navigator = BasicNavigator()
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = -3.5
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(initial_pose)

        self.navigator.waitUntilNav2Active()


        self.cmd_vel_publisher = self.create_publisher(Twist, '/robot1/cmd_vel', 10)

        self.items = ItemList()
        self.item_holders = ItemHolders()

        self.item_subscriber = self.create_subscription(
            ItemList,
            '/robot1/items',
            self.item_callback,
            10)
        
        self.item_holders_subscriber = self.create_subscription(
            ItemHolders,
            '/item_holders',
            self.item_holders_callback,
            10)

        # set initial pose for the navigator
        # get the inital pose of the robot

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def item_callback(self, msg):
        self.items = msg
    
    def item_holders_callback(self, msg):
        self.item_holders = msg

    def control_loop(self):
        self.get_logger().info(f"Initial pose - x: {self.initial_x}, y: {self.initial_y}, yaw: {self.initial_yaw}")
        self.get_logger().info(f"item holders: {self.item_holders.data}")
        #setting goal

        match self.state:
            case State.FORWARD:
                if len(self.items.data) > 0:
                    self.state = State.COLLECTING
                    self.get_logger().info("Transitioning to COLLECTING state")
                    return
            case State.COLLECTING:
                
                if self.item_holders.data:
                    self.previous_pose = self.pose
                    self.state = State.SET_GOAL
                    self.get_logger().info("Transitioning to SET_GOAL state")
                    return

                item = self.items.data[0]
                estimated_distance = 69.0 * float(item.diameter) ** -0.89

                msg = Twist()
                msg.linear.x = 0.25 * estimated_distance
                msg.angular.z = item.x / 320.0

                self.cmd_vel_publisher.publish(msg)
                
            case State.SET_GOAL:

                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.get_clock().now().to_msg()
                goal_pose.pose.position.x = 0.0
                goal_pose.pose.position.y = -3.5
                goal_pose.pose.orientation.w = 1.0

                self.navigator.goToPose(goal_pose)

                self.get_logger().info("Setting goal pose:")
                self.get_logger().info(f"Goal pose: {goal_pose}")

                self.state = State.NAVIGATING

            case State.NAVIGATING:
        
                if not self.navigator.isTaskComplete():
                    feedback = self.navigator.getFeedback()
                    print('Estimated time of arrival: ' + '{0:.0f}'.format(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds.')
                else:

                    result = self.navigator.getResult()

                    if result == TaskResult.SUCCEEDED:
                        print('Goal succeeded!')
                    elif result == TaskResult.CANCELED:
                        print('Goal was canceled!')
                    elif result == TaskResult.FAILED:
                        print('Goal failed!')
                    else:
                        print('Goal has an invalid return status!')

    def destroy_node(self):
        super().destroy_node()


def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = RobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
