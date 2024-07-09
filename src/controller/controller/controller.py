import rclpy  # ROS 2的Python接口
from rclpy.node import Node  # ROS 2的节点类
from std_msgs.msg import String, Float32MultiArray  # ROS标准消息类型
import json  # 用于解析和生成JSON字符串
import numpy as np  # 用于数值计算的库
import scipy.signal as signal  # 用于信号处理的库

class IIRFilter:
    def __init__(self, b, a):
        self.b = b
        self.a = a
        self.zi = np.zeros(max(len(a), len(b)) - 1)

    def apply(self, data):
        data_filtered, self.zi = signal.lfilter(self.b, self.a, data, zi=self.zi)
        return data_filtered

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
            'coordinate_topic',
            self.listener_callback,
            10)
        self.publisher_velocity = self.create_publisher(Float32MultiArray, 'velocity_topic', 10)
        self.publisher_coordinate = self.create_publisher(String, 'coordinate', 10)
        self.filters=[]
        self.positions=[]
        self.state=False

    def listener_callback(self, data):
        self.process_coordinates(data.data)

    def process_coordinates(self, data):
        coordinates_dict = json.loads(data)

        filtered_coordinates = []

        for i, (key, value) in enumerate(coordinates_dict.items()):
            coordinate = np.array([value[0], value[1], value[2]])  # 提取xyz平面坐标
            filtered_coord = self.iir_filters[i].apply(coordinate)
            filtered_coordinates.append((key, filtered_coord))

        distances = []
        for coord in filtered_coordinates:
            dist = np.linalg.norm(coord[1])  # 计算欧几里得范数
            distances.append(dist)

        closest_target_idx = np.argmin(distances)
        closest_target = filtered_coordinates[closest_target_idx]

        distance_to_target = distances[closest_target_idx]

        if distance_to_target <= 0.2:  # 20 cm
            self.reached_target = True
            filtered_target = json.dumps({
                "x": closest_target[1][0],  # 输出包含x坐标
                "y": closest_target[1][1],
                "z": closest_target[1][2]
            })
            self.publisher_coordinate.publish(String(data=filtered_target))
            self.plan_and_publish_velocity(closest_target[1], align_only=True)
        else:
            self.reached_target = False
            self.publisher_coordinate.publish(String(data=json.dumps({"x": 0, "y": 0, "z": 0})))
            self.plan_and_publish_velocity(closest_target[1])

    def plan_and_publish_velocity(self, target_coordinate, align_only=False):
        current_position = self.get_current_position()
        error = target_coordinate[1:] - current_position  # 仅考虑y和z坐标

        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time

        if align_only:
            target_angle = np.arctan2(error[1], error[0])
            angular_velocity = self.pid_angular.compute(target_angle, dt)
            velocity = [0.0, 0.0, angular_velocity]
        else:
            y_velocity = self.pid_y.compute(error[0], dt)
            z_velocity = self.pid_z.compute(error[1], dt)
            target_angle = np.arctan2(error[1], error[0])
            angular_velocity = self.pid_angular.compute(target_angle, dt)
            velocity = [0.0, y_velocity, z_velocity, angular_velocity]

        self.publish_velocity(velocity)

    def get_current_position(self):
        return np.array([0, 0])  # 仅考虑y和z坐标

    def publish_velocity(self, velocity):
        vel_msg = Float32MultiArray(data=velocity)
        self.publisher_velocity.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    coordinate_listener = CoordinateListener()
    rclpy.spin(coordinate_listener)
    coordinate_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
