import rclpy  # ROS 2的Python接口
from rclpy.node import Node  # ROS 2的节点类
from std_msgs.msg import String, Float32MultiArray  # ROS标准消息类型
import json  # 用于解析和生成JSON字符串
import numpy as np  # 用于数值计算的库
import time

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
        self.publisher_infetch = self.create_publisher(bool, 'infetch', 10)

        self.pid_v = PIDController(Kp=0.003, Ki=0.0, Kd=0.0)#0.03
        self.pid_angular = PIDController(Kp=4.0, Ki=0.0, Kd=0.0)
        self.last_time = self.get_clock().now().nanoseconds / 1e9
        self.dis_err

        self.in_fetch_time = False
        self.fectch_start_time

        self.cancle_fetch_time = False
        self.cancle_fetch_start_time
        self.fetchnow = False
        

    def listener_callback(self, msg):      
        location = json.loads(msg.data)
      
        x=location['x'];y=location['y'];z=location['z']

        self.distance = (x**2+y**2+z**2)**0.5
        self.get_logger().info(f"{x},{y},{z} dist: {self.distance}")
        if not self.fetchnow:
            self.fetch()
        else:
            self.fetch_cancle()
        self.plan_and_publish_velocity([x,z])


    def fetch(self):
        if (self.dis_err <200):
            if(not self.in_fetch_time):
                self.fetch_start_time = time.time()
                self.in_fetch_time = True
            if(time.time()-self.fetch_start_time>5):
                self.fetchnow = True
                self.publisher_infetch.publish(True)
        else:
            self.in_fetch_time = False
            self.fetch_start_time = time.time()
            self.publisher_infetch.publish(False)


    def fetch_cancle(self):
        if (self.dis_err <200):
            if(not self.cancle_fetch_time):
                self.fetch_start_time = time.time()
                self.cancle_fetch_time = True
            if(time.time()-self.fetch_start_time>5):
                self.fetchnow = False
                self.publisher_infetch.publish(False)
        else:
            self.cancle_fetch_time = False
            self.fetch_start_time = time.time()
            self.publisher_infetch.publish(True)


    def plan_and_publish_velocity(self, target_location):
        dis=(target_location[0]**2+target_location[1]**2)**0.5
        target_angle = np.arctan2(target_location[0], target_location[1])
        self.dis_err=dis-300
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time

        vel=self.pid_v.compute(self.dis_err,dt)
        angular_vel = self.pid_angular.compute(target_angle, dt)
        self.get_logger().info(f"NOT angle_only ,angle={target_angle}")
        vx=vel*(target_location[0]/dis)
        vz=vel*(target_location[1]/dis)
        velocity = [vz,-vx, -angular_vel,self.fetchnow]
        # velocity = [0,0, -angular_vel]

        self.publish_velocity(velocity)

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
