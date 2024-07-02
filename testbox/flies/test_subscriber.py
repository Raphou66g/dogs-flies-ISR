import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3

from position import Position


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.pos = Position()
        self.twistL: Vector3 = None
        self.twistA: Vector3 = None
        self.orient: Quaternion = None
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg:Odometry):
        # self.get_logger().info(f'I heard:\n{msg}\n\n------------------------------\n\n')
        poseWC: PoseWithCovariance = msg.pose
        pose: Pose = poseWC.pose
        self.pos.x, self.pos.y, self.pos.z = pose.position.x, pose.position.y, pose.position.z
        self.orient = pose.orientation
        twistWC: TwistWithCovariance = msg.twist
        twist: Twist = twistWC.twist
        self.twistL = twist.linear
        self.twistA = twist.angular
        print(self.pos, self.orient, self.twistL, self.twistA)
        

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
