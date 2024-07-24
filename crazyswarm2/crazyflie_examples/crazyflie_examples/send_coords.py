import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.5):
        super().__init__("minimal_publisher")
        self.x = x
        self.y = y
        self.z = z
        self.publisher_ = self.create_publisher(Odometry, "odom", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Odometry()
        msg.pose = PoseWithCovariance()
        msg.pose.pose = Pose()
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = self.z
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "x={msg.pose.pose.position.x} - y={msg.pose.pose.position.y} - z={msg.pose.pose.position.z}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    
    x = input("x : ")
    y = input("y : ")
    z = input("z : ")

    minimal_publisher = MinimalPublisher(float(x), float(y), float(z))

    input(f"{x},{y},{z}\nPublish ?")
    
    rclpy.spin_once(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
