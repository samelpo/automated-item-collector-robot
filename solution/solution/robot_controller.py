import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from rclpy.qos import QoSPresetProfiles

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from assessment_interfaces.msg import ItemList, ItemHolder, ItemHolders, HomeZone
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
    FINDING = 0
    TURNING = 1
    COLLECTING = 2
    SET_GOAL = 3
    NAVIGATING = 4
    RETURN_HOME = 5
    DRIVE = 6

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value

        self.state = State.DRIVE
        self.pose = Pose()

        self.previous_pose = Pose() # Store a snapshot of the pose for comparison against future poses
        self.yaw = 0.0 # Angle the robot is facing (rotation around the Z axis, in radians), relative to the odom reference frame
        self.previous_yaw = 0.0 # Snapshot of the angle for comparison against future angles
        self.turn_angle = 0.0 # Relative angle to turn to in the TURNING state
        self.turn_direction = TURN_LEFT # Direction to turn in the TURNING state
        self.goal_distance = random.uniform(1.0, 2.0) # Goal distance to travel in FORWARD state
        self.scan_triggered = [False] * 4 # Boolean value for each of the 4 LiDAR sensor sectors. True if obstacle detected within SCAN_THRESHOLD

        # set initial pose for the navigator
        # get the inital pose of the robot

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
        self.item_holder = ItemHolder()
        self.home_zone_sensor = HomeZone()

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
        
        self.home_zone_subscriber = self.create_subscription(
            HomeZone,
            '/robot1/home_zone',
            self.home_zone_callback,
            10)
        
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/robot1/odom',
            self.odom_callback,
            10)

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/robot1/scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value)


        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)
    
    def stop_robot(self):
        stop_command = Twist()
        self.cmd_vel_publisher.publish(stop_command)
        self.get_logger().info('robot stopped')

    def item_callback(self, msg):
        self.items = msg
    
    def item_holders_callback(self, msg):

        self.item_holders = msg
    
    def home_zone_callback(self, msg):
        self.home_zone_sensor = msg
    
    def odom_callback(self, msg):
        self.pose = msg.pose.pose

        (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x,
                                                    self.pose.orientation.y,
                                                    self.pose.orientation.z,
                                                    self.pose.orientation.w])
        
        self.yaw = yaw

    def scan_callback(self, msg):
        # Group scan ranges into 4 segments
        # Front, left, and right segments are each 60 degrees
        # Back segment is 180 degrees
        front_ranges = msg.ranges[331:359] + msg.ranges[0:30] # 30 to 331 degrees (30 to -30 degrees)
        left_ranges  = msg.ranges[31:90] # 31 to 90 degrees (31 to 90 degrees)
        back_ranges  = msg.ranges[91:270] # 91 to 270 degrees (91 to -90 degrees)
        right_ranges = msg.ranges[271:330] # 271 to 330 degrees (-30 to -91 degrees)

        # Store True/False values for each sensor segment, based on whether the nearest detected obstacle is closer than SCAN_THRESHOLD
        self.scan_triggered[SCAN_FRONT] = min(front_ranges) < SCAN_THRESHOLD 
        self.scan_triggered[SCAN_LEFT]  = min(left_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_BACK]  = min(back_ranges)  < SCAN_THRESHOLD
        self.scan_triggered[SCAN_RIGHT] = min(right_ranges) < SCAN_THRESHOLD
    


    def control_loop(self):
        #self.get_logger().info(f"Initial pose - x: {self.initial_x}, y: {self.initial_y}, yaw: {self.initial_yaw}")
        #self.get_logger().info(f"item: {self.items.data}")
        match self.state:
            case State.DRIVE:
                #self.get_logger().info(f"item_holders: {self.item_holders.data}")
                
                # several different paths for the robot to take
                self.get_logger().info("in drive state")

                path_y = [-2.0, 0.0, 2.0]
                path = []

                # 1st choice of path
                goal_pose_1 = PoseStamped()
                goal_pose_1.header.frame_id = 'map'
                goal_pose_1.header.stamp = self.get_clock().now().to_msg()
                goal_pose_1.pose.position.x = -1.0
                goal_pose_1.pose.position.y = random.choice(path_y)
                goal_pose_1.pose.orientation.w = 1.0

                path.append(goal_pose_1)

                # 2nd choice of path
                goal_pose_2 = PoseStamped()
                goal_pose_2.header.frame_id = 'map'
                goal_pose_2.header.stamp = self.get_clock().now().to_msg()
                goal_pose_2.pose.position.x = 1.0
                goal_pose_2.pose.position.y = random.choice(path_y)
                goal_pose_2.pose.orientation.w = 1.0

                path.append(goal_pose_2)

                self.navigator.goThroughPoses(path)

                #self.get_logger().info("Setting goal pose:")
                #self.get_logger().info(f"Goal pose: {goal_pose}")

                self.state = State.NAVIGATING


            case State.FINDING:

                # If camera sees an item, go for the first one
                if len(self.items.data) > 0:
                    self.state = State.COLLECTING
                    self.get_logger().info("Transitioning to COLLECTING state")
                    return
                
                # turn to find more items
                if len(self.items.data) == 0:
                    self.previous_pose = self.pose
                    self.state = State.TURNING
                    return
            
            case State.COLLECTING:

                if self.scan_triggered[SCAN_FRONT]:
                    self.previous_yaw = self.yaw
                    self.state = State.DRIVE
                    self.turn_angle = random.uniform(150, 170)
                    self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                    self.get_logger().info("Detected obstacle in front, turning " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" by {self.turn_angle:.2f} degrees")
                    return
                
                if self.scan_triggered[SCAN_LEFT] or self.scan_triggered[SCAN_RIGHT]:
                    self.previous_yaw = self.yaw
                    self.state = State.DRIVE
                    self.turn_angle = 45

                    if self.scan_triggered[SCAN_LEFT] and self.scan_triggered[SCAN_RIGHT]:
                        self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
                        self.get_logger().info("Detected obstacle to both the left and right, turning " + ("left" if self.turn_direction == TURN_LEFT else "right") + f" by {self.turn_angle:.2f} degrees")
                    elif self.scan_triggered[SCAN_LEFT]:
                        self.turn_direction = TURN_RIGHT
                        self.get_logger().info(f"Detected obstacle to the left, turning right by {self.turn_angle} degrees")
                    else: # self.scan_triggered[SCAN_RIGHT]
                        self.turn_direction = TURN_LEFT
                        self.get_logger().info(f"Detected obstacle to the right, turning left by {self.turn_angle} degrees")
                    return
                self.get_logger().info(f"item_holders: {self.item_holders.data}")

                item_held = self.item_holders.data[0]

                # if it is holding an item go into the turning state
                if item_held.holding_item == True:
                    self.previous_pose = self.pose
                    self.state = State.RETURN_HOME
                    self.get_logger().info("Transitioning to RETURN_HOME state")
                    return
                
                # if it doesnt see any items go into a finding state
                if len(self.items.data) == 0:
                    self.previous_pose = self.pose
                    self.state = State.FINDING
                    return

                # once it sees an item go towards it to pick it up
                item = self.items.data[0]
                estimated_distance = 69.0 * float(item.diameter) ** -0.89

                msg = Twist()
                msg.linear.x = LINEAR_VELOCITY #0.25 * estimated_distance
                msg.angular.z = item.x / 320.0

                self.cmd_vel_publisher.publish(msg)
            
            case State.TURNING:

                if len(self.items.data) > 0:
                    self.state = State.COLLECTING
                    return

                msg = Twist()
                msg.angular.z = self.turn_direction * ANGULAR_VELOCITY
                self.cmd_vel_publisher.publish(msg)
            
            case State.RETURN_HOME:
                
                item_held = self.item_holders.data[0]
                if item_held.holding_item == False:
                    self.navigator.cancelTask()
                    self.state = State.DRIVE
                    self.get_logger().info("Transitioning to DRIVE state")
                    return

                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.get_clock().now().to_msg()
                goal_pose.pose.position.x = -3.5
                goal_pose.pose.position.y = 0.0
                goal_pose.pose.orientation.w = 1.0

                self.navigator.goToPose(goal_pose)

                self.get_logger().info("Setting goal pose:")
                self.get_logger().info(f"Goal pose: {goal_pose}")

            case State.NAVIGATING:

                try:
                    item_held = self.item_holders.data[0]
                    if item_held.holding_item == True:
                        self.navigator.cancelTask()
                        self.state = State.RETURN_HOME
                        self.get_logger().info("Transitioning to RETURN_HOME state")
                        return
                except IndexError:
                    self.get_logger().info("index is out of range")


                if not self.navigator.isTaskComplete():
                    feedback = self.navigator.getFeedback()
                    print('Estimated time of arrival: ' + '{0:.0f}'.format(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds.')
                else:

                    result = self.navigator.getResult()

                    if result == TaskResult.SUCCEEDED:
                        print('Goal succeeded!')
                        self.state = State.FINDING
                        return
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
