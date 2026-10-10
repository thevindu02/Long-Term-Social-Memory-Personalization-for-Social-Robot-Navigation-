import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf2_ros

class NavTester(Node):
    def __init__(self):
        super().__init__('nav_tester')
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.last_print_dist = -1.0
        self.initial_pose = None

    def odom_callback(self, msg):
        if self.initial_pose is None:
            self.initial_pose = msg.pose.pose

    def wait_for_clock(self):
        self.get_logger().info('Waiting for /clock...')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            t = self.get_clock().now().seconds_nanoseconds()
            if t[0] > 0 or t[1] > 0:
                self.get_logger().info('Clock received.')
                break

    def wait_for_odom(self):
        self.get_logger().info('Waiting for /odom...')
        while rclpy.ok() and self.initial_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'Odom received: x={self.initial_pose.position.x:.2f}, y={self.initial_pose.position.y:.2f}')

    def set_initial_pose(self):
        self.wait_for_clock()
        self.wait_for_odom()

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose = self.initial_pose
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068

        self.get_logger().info('Publishing initial pose repeatedly until TF map->base_link exists...')
        
        while rclpy.ok():
            msg.header.stamp = self.get_clock().now().to_msg()
            self.init_pose_pub.publish(msg)
            
            try:
                self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                self.get_logger().info('TF map->base_link found!')
                break
            except Exception as e:
                self.get_logger().info(f'Waiting for TF... ({str(e)})')
            
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info('Sleeping for 2s to allow AMCL to process...')
        start_time = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - start_time).nanoseconds > 2e9:
                break

    def send_goal(self, x, y):
        # Update clock
        rclpy.spin_once(self, timeout_sec=0.1)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Sending goal to x={x}, y={y}...')
        self.last_print_dist = -1.0
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, self.send_goal_future)
        
        goal_handle = self.send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return False
            
        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.get_result_future)
        
        status = self.get_result_future.result().status
        self.get_logger().info(f'Goal finished with status: {status} (4=SUCCEEDED)')
        return status == 4

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        curr = feedback.current_pose.pose.position
        dist = feedback.distance_remaining
        
        # Only print every ~0.5m of progress to keep logs clean
        if self.last_print_dist < 0 or abs(self.last_print_dist - dist) > 0.5:
            self.get_logger().info(f'Progress: At ({curr.x:.2f}, {curr.y:.2f}) | Remaining: {dist:.2f}m')
            self.last_print_dist = dist

def main():
    rclpy.init()
    tester = NavTester()
    tester.set_initial_pose()
    
    # 1. Corridor Goal
    print("\n\n--- GOAL 1: North Corridor (0.0, 10.0) ---")
    success1 = tester.send_goal(0.0, 10.0)
    
    if success1:
        # 2. NE Room Goal (through the 2m door gap at X=8, Y=2)
        print("\n\n--- GOAL 2: NE Room Interior (10.5, 6.0) ---")
        tester.send_goal(10.5, 6.0)
    
    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
