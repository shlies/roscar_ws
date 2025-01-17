#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def float_to_hex_bytes(float_num):
    # 将浮点数乘以缩放因子 (100) 并转换为整数
    scaled_int = int(float_num * 100)
    
    # 将整数分解为两个字节（大端字节序）
    byte1 = (scaled_int >> 8) & 0xFF
    byte2 = scaled_int & 0xFF
    
    # 将字节转换为16进制字符串
    
    return byte1, byte2

class SerialNode(Node):
    def __init__(self):
        super().__init__('uart_comm_test')
        self.get_logger().info('uart_comm_node started')

        # 创建订阅者，订阅名为“robot_running”的话题
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'robot_running',
            self.listener_callback,
            10
        )

        

    def listener_callback(self, msg):
        data = msg.data  
        self.get_logger().info(f'Received: {data}')
        self.received_data = [float(x) for x in data]  
        
        byte_list = [0xAA, 0xBB]


        for number in self.received_data:
            hex_byte1, hex_byte2 = float_to_hex_bytes(number)
            byte_list.append(hex_byte1)
            byte_list.append(hex_byte2)

            self.get_logger().info(f'Sent: {hex_byte1} and {hex_byte2}')
        
        print(byte_list)
        self.received_data = []                     # 清空已发送的数据

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()