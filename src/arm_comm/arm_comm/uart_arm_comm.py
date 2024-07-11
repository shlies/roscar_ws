import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # 导入标准的String消息类型
import json  # 导入JSON库，用于解析和生成JSON数据
import serial  # 导入pyserial库，用于串口通信
import time

class StringSerialCommNode(Node):
    def __init__(self):
        super().__init__('string_serial_comm')  # 初始化节点，节点名为'string_serial_comm'
        self.get_logger().info('uart_arm_comm_node started')
        self.subscription = self.create_subscription(
            String,
            'string_topic',  # 订阅的话题名为'string_topic'
            self.callback,  # 指定回调函数
            10  # 消息队列长度
        )
        self.serial_port = serial.Serial('/dev/ttyUSB2', 115200)  # 初始化串口通信，根据实际情况修改端口和波特率

    def callback(self, msg):
        json_data_str = msg.data  # 获取收到的字符串消息
        json_data = json.loads(json_data_str)  # 解析JSON字符串为Python字典
        json_data['T'] = 1041  # 添加新的键值对 'T': 1041
        self.send_serial_data(json_data)  # 调用发送串口数据的方法
 
    def send_serial_data(self, data):
        serialized_data = json.dumps(data)  # 将Python字典转换为JSON格式的字符串
        self.get_logger().info(serialized_data)
        serialized_data += '\n'  # 添加换行符作为结束符
        self.serial_port.write(serialized_data.encode('utf-8'))  # 发送数据到串口
        time.sleep(0.1)  # 等待一段时间确保数据发送完整

def main(args=None):
    rclpy.init(args=args)  # 初始化ROS 2客户端库
    node = StringSerialCommNode()  # 创建StringSerialCommNode节点对象
    rclpy.spin(node)  # 进入ROS 2节点的事件循环，等待消息到达
    node.destroy_node()  # 销毁节点对象
    rclpy.shutdown()  # 关闭ROS 2客户端库

if __name__ == '__main__':
    main()
