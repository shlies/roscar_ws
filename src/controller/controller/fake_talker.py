import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class FakeTalker(Node):
    def __init__(self):
        super().__init__('fake_talker')
        self.publisher = self.create_publisher(String, 'coordinate_topic', 10)
        timer_period = 1.0  # 发布频率为1秒
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # 固定的坐标数据，以字符串格式发布
        coordinates_str = '{"target1": [1.0, 2.0, 3.0], "target2": [4.0, 5.0, 6.0], "target3": [7.0, 8.0, 9.0]}'
        msg = String(data=coordinates_str)
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {coordinates_str}')

def main(args=None):
    rclpy.init(args=args)
    fake_talker = FakeTalker()
    rclpy.spin(fake_talker)
    fake_talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

//[{'class': 'A', 'confidence': 0.8883844017982483, 'coordinates': {'center': [249, 213], 'pixel_to_camera': [-0.0, -0.0, 0.0], 'calculated_3d': [-44.637304364675, -18.9066722459125, 335.1136034747884]}},
   {'class': 'A', 'confidence': 0.8883844017982483, 'coordinates': {'center': [249, 213], 'pixel_to_camera': [-0.0, -0.0, 0.0], 'calculated_3d': [-44.637304364675, -18.9066722459125, 335.1136034747884]}}]
