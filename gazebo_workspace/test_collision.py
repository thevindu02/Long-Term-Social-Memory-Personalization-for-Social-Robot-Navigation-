import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class CollisionTester(Node):
    def __init__(self):
        super().__init__('collision_tester')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time.time()
        self.last_y = 0.0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 2.0 # Drive fast forward (Wait, if robot is oriented East (+X), I should turn it to North (+Y) or just drive +X into the East wall? East wall is at X=5.0)
        self.pub.publish(msg)
        if time.time() - self.start_time > 10.0:
            self.get_logger().info(f"Final X position: {self.last_x}")
            rclpy.shutdown()

    def odom_callback(self, msg):
        self.last_x = msg.pose.pose.position.x
        self.get_logger().info(f"Odom X: {self.last_x:.2f}")

def main():
    rclpy.init()
    tester = CollisionTester()
    rclpy.spin(tester)

if __name__ == '__main__':
    main()
