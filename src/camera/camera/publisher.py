import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import json
import select

class SocketToROSPublisher(Node):
    def __init__(self):
        super().__init__('socket_to_ros_publisher')
        self.publisher_ = self.create_publisher(String, 'socket_data', 10)
        
    def start_server_old(self):
        host = '127.0.0.1'
        port = 65432
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((host, port))
            s.listen()
            self.get_logger().info("Server started, waiting for connections...")
            while True:
                conn, addr = s.accept()
                self.get_logger().info(f"Connected by {addr}")
                self.handle_client(conn)
    def start_server(self):
        host = '127.0.0.1'
        port = 65432
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((host, port))
            s.listen()
            s.setblocking(False)  # 设置socket为非阻塞模式
            self.get_logger().info("Server started, waiting for connections...")
            while True:
                # 使用select来检查是否有连接尝试，设置超时时间为5秒
                ready, _, _ = select.select([s], [], [], 2.5)
                if ready:
                    conn, addr = s.accept()
                    self.get_logger().info(f"Connected by {addr}")
                    self.handle_client(conn)
                else:
                    # 在这里处理超时逻辑，例如打印日志
                    self.get_logger().info("No connections in the last 2.5 seconds...")
                    # msg = String()
                    # 发布u{'class': 'A', 'confidence': 0, 'coordinates': {'center': [0, 0],'calculated_3d': [10.0, 0.0, 1.0]}}
                    # msg.data = json.dumps([{'class': 'A', 'confidence': 0, 'coordinates': {'center': [0, 0],'calculated_3d': [10.0, 0.0, 1.0]}}])
                    # self.publisher_.publish(msg)

    def handle_client(self, conn):
        with conn:
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                received_objects = json.loads(data.decode('utf-8'))
                self.get_logger().info(f"Received data: {received_objects}")
                msg = String()
                msg.data = json.dumps(received_objects)
                self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    socket_to_ros_publisher = SocketToROSPublisher()
    socket_to_ros_publisher.start_server()
    rclpy.spin(socket_to_ros_publisher)
    socket_to_ros_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
