import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class RobotStatusPublisher(Node):
    def __init__(self):
        super().__init__('robot_status_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'robot_running', 10)
        timer_period = 0.5  # 每0.5秒发布一次
        self.timer = self.create_timer(timer_period, self.publish_status)

    def publish_status(self):
        msg = Float32MultiArray()
        msg.data = [1.0, 2.0, 3.0]  # 示例数据
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
