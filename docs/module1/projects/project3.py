import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joint_pos = 0.0
        self.direction = 1

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['base_to_arm']
        msg.position = [self.joint_pos]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{self.joint_pos}"')

        self.joint_pos += self.direction * 0.05
        if self.joint_pos > 1.57 or self.joint_pos < -1.57:
            self.direction *= -1

def main(args=None):
    rclpy.init(args=args)
    joint_publisher = JointPublisher()
    rclpy.spin(joint_publisher)
    joint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()