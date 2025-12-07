import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.publisher_ = self.create_publisher(Float64, '/humanoid/joint_controller/commands', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.position = 0.0
        self.direction = 1

    def timer_callback(self):
        msg = Float64()
        msg.data = self.position
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing joint command: "{msg.data}"')

        self.position += self.direction * 0.1
        if self.position > 1.0 or self.position < -1.0:
            self.direction *= -1

def main(args=None):
    rclpy.init(args=args)
    joint_controller = JointController()
    rclpy.spin(joint_controller)
    joint_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()