import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import json

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.publisher_arm_back = self.create_publisher(String, 'arm_back', 10)
        self.subscription = self.create_subscription(
            String,
            'arm',
            self.arm_callback,
            10)
        self.serial_port = None
        self.timer = self.create_timer(0.1, self.serial_reader)  # 使用定时器替代线程

    def arm_callback(self, msg):
        try:
            arm_data = json.loads(msg.data)
            #self.get_logger().info(f"{arm_data}")
            target_coordinate = [arm_data['x'], arm_data['y'], arm_data['z']]
            ox = oy = oz = 0  # Define your offsets here if needed
            target = {
                "T": 1041,  # Example value for T
                "x": target_coordinate[0] + ox,
                "y": target_coordinate[1] + oy,
                "z": target_coordinate[2] + oz,
                "t": 3.14
            }
            json_target = json.dumps(target)
            if self.serial_port:
                self.get_logger().info(f"{json_target}")
                self.serial_port.write(json_target.encode() + b'\n')
        except Exception as e:
            self.get_logger().error(f"Error in arm_callback: {e}")

    def serial_reader(self):
        if self.serial_port and self.serial_port.in_waiting > 0:
            data = self.serial_port.readline().decode('utf-8').strip()
            if data:
                self.get_logger().info(f"Received from serial: {data}")
                msg = String()
                msg.data = data
                self.publisher_arm_back.publish(msg)

    def start_serial(self, port):
        try:
            self.serial_port = serial.Serial(port, baudrate=115200)
            self.serial_port.setRTS(False)
            self.serial_port.setDTR(False)
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            self.serial_port = None

    def stop_serial(self):
        if self.serial_port:
            self.serial_port.close()

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    port = '/dev/ttyUSB0'  # Replace with your serial port
    serial_node.start_serial(port)

    try:
        rclpy.spin(serial_node)
    except KeyboardInterrupt:
        pass

    serial_node.stop_serial()
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
