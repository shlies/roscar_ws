import rclpy  # ROS 2的Python接口
from rclpy.node import Node  # ROS 2的节点类
from std_msgs.msg import String, Float32MultiArray, Bool  # ROS标准消息类型
import json  # 用于解析和生成JSON字符串
import numpy as np  # 用于数值计算的库
import time  # 用于处理时间相关的功能

class PIDController:
    def __init__(self, Kp, Ki, Kd, max_integral=1.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_integral = max_integral
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max_integral), -self.max_integral)
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

class CoordinateListener(Node):
    def __init__(self):
        super().__init__('coordinate_listener')
        self.subscription = self.create_subscription(
            String,
            'coordinate',
            self.listener_callback,
            10)
        self.publisher_velocity = self.create_publisher(Float32MultiArray, 'robot_running', 10)
        self.publisher_arm = self.create_publisher(String, 'arm', 10)
        self.publisher_in_fetch = self.create_publisher(Bool, 'in_fetch', 10)  # 新增发布者
        self.publisher_grab_success = self.create_publisher(Bool, 'grab_success', 10)  # 抓取成功的发布者

        self.pid_v = PIDController(Kp=0.003, Ki=0.0, Kd=0.0)
        self.pid_angular = PIDController(Kp=4.0, Ki=0.0, Kd=0.0)
        self.last_time = self.get_clock().now().nanoseconds / 1e9
        self.dis_err = 0.0  # 初始化距离误差

        self.in_fetch_time = False
        self.fetch_start_time = 0.0  # 初始化抓取开始时间
        self.fetch_now = False
        self.grab_success = False  # 抓取成功标志
        self.target_location = [0.0, 0.0, 0.0]  # 目标位置

    def listener_callback(self, data):
        location = json.loads(data.data)
        x = location['x']
        y = location['y']
        z = location['z']
        self.distance = (x**2 + y**2 + z**2)**0.5
        self.get_logger().info(f"{x},{y},{z} dist: {self.distance}")
        self.target_location = [x, y, z]
        self.plan_and_publish_velocity([x, z])
        self.fetch()  # 每次接收到数据时调用fetch

    def fetch(self):
        if self.dis_err < 200 and not self.grab_success:
            if not self.in_fetch_time:
                self.fetch_start_time = time.time()
                self.in_fetch_time = True
            if time.time() - self.fetch_start_time > 5:
                self.fetch_now = True
                self.publish_in_fetch(True)  # 发布in_fetch消息
        else:
            self.in_fetch_time = False
            self.fetch_start_time = time.time()
            self.publish_in_fetch(False)  # 发布in_fetch消息

        if self.fetch_now and self.dis_err < 50:
            self.grab_success = True
            self.publish_grab_success(True)  # 发布抓取成功消息
            self.fetch_now = False

    def plan_and_publish_velocity(self, target_location):
        dis = (target_location[0]**2 + target_location[1]**2)**0.5
        target_angle = np.arctan2(target_location[0], target_location[1])
        self.dis_err = dis - 300
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time

        vel = self.pid_v.compute(self.dis_err, dt)
        angular_vel = self.pid_angular.compute(target_angle, dt)
        self.get_logger().info(f"NOT angle_only, angle={target_angle}")
        vx = vel * (target_location[0] / dis)
        vz = vel * (target_location[1] / dis)
        velocity = [vz, -vx, -angular_vel, self.fetch_now]

        self.publish_velocity(velocity)

        if self.grab_success and self.dis_err < 50:
            self.cancel_grab()

    def publish_velocity(self, velocity):
        vel_msg = Float32MultiArray(data=velocity)
        self.publisher_velocity.publish(vel_msg)

    def publish_in_fetch(self, fetch_status):
        fetch_msg = Bool(data=fetch_status)
        self.publisher_in_fetch.publish(fetch_msg)

    def publish_grab_success(self, success_status):
        success_msg = Bool(data=success_status)
        self.publisher_grab_success.publish(success_msg)

    def cancel_grab(self):
        self.grab_success = False
        self.publish_grab_success(False)
        self.get_logger().info("Grab cancelled, moving to destination")

def main(args=None):
    rclpy.init(args=args)
    coordinate_listener = CoordinateListener()
    rclpy.spin(coordinate_listener)
    coordinate_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
