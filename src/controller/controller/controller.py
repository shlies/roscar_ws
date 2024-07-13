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
        self.publisher_arm = self.create_publisher(String, 'arm', 10)

        self.pid_v = PIDController(Kp=0.0007, Ki=0.0, Kd=0.0)#0.03
        self.pid_angular = PIDController(Kp=0.1, Ki=0.0, Kd=0.0)
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        self.target_location=[0.0,0.0,0.0]#罐子坐标
        self.destination_location=[0.0,0.0,0.0]#终点坐标
        self.current_location=[0.0,0.0,0.0]#目前前往的坐标

        self.dis_err = 1000#与目标距离的差
        self.distance_ref=300#期望的距离

        self.mode='SearchCan'#当前行为模式 SearchCan GetCan GoDes PutCan Back 

        self.task_finish=100 #当前目标完成标志
        self.fitch_flag=0

    def listener_callback(self, data):      
        location = json.loads(data.data)
      
        self.target_location=[location['x'],location['y'],location['z']]
        self.destination_location=[location['xt'],location['yt'],location['zt']]

        if self.mode=='SearchCan':
            self.current_location=self.target_location
            self.distance_ref=160
        elif self.mode=='GoDes':
            self.current_location=self.destination_location
            self.distance_ref=300
        elif self.mode=='Back':
            self.current_location=self.destination_location
            self.distance_ref=500
        else:
            self.current_location=[1.0,0.0,10.0]

        x=self.current_location[0]
        y=self.current_location[1]
        z=self.current_location[2]
        # self.distance = (x**2+y**2+z**2)**0.5
        self.get_logger().info(f"{x},{y},{z}")
        self.plan_and_publish_velocity([x,z])
        self.switchMode()

    def refresh(self):
        self.dis_err = 1000
        self.task_finish=80
    
    def switchMode(self):
        self.get_logger().warning(f"Mode: {self.mode},Task: {self.task_finish}")
        if self.mode=='SearchCan':
            if (self.dis_err <20):
                self.mode='GetCan'
                self.refresh()
                return
        elif self.mode=='GetCan':
            if(self.task_finish<=0):
                self.mode='GoDes'
                self.refresh()
                return
            self.fitch_flag=1.0-self.task_finish/80.0
            self.task_finish=self.task_finish-1
        elif self.mode=='GoDes':
            if (self.dis_err <20):
                self.mode='PutCan'
                self.refresh()
                return
        elif self.mode=='PutCan':
            if(self.task_finish<=0):
                self.mode='Back'
                self.refresh()
                return
            self.fitch_flag=self.task_finish/80.0
            self.task_finish=self.task_finish-1
        elif self.mode=='Back':
            if (self.dis_err <20):
                self.mode='PutCan'
                self.refresh()
                return
           
    def restrict(self,number,limit):
        if number<-limit:
            return -limit
        if number>limit:
            return limit
        return number

    def plan_and_publish_velocity(self, target_location):
        dis=(target_location[0]**2+target_location[1]**2)**0.5
        target_angle = np.arctan2(target_location[0], target_location[1])
        if target_location!=[10.0,1.0] and target_location!=[1.0,10.0]:
            self.dis_err=dis-self.distance_ref
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time

        vel=self.pid_v.compute(self.dis_err,dt)
        angular_vel = self.pid_angular.compute(target_angle, dt)
        # self.get_logger().info(f"\nv={vel}\nangle_v={angular_vel}")
        vx=vel*(target_location[0]/dis)
        vz=vel*(target_location[1]/dis)
        vx=self.restrict(vx,0.1)
        vz=self.restrict(vz,0.1)
        angular_vel=self.restrict(angular_vel,0.5)
        if self.fitch_flag<0:
            self.fitch_flag=0
        velocity = [vz,-vx, -angular_vel,self.fitch_flag]
        self.get_logger().info(f"\n\n{self.fitch_flag},{abs(target_location[0]-1.0)+abs(target_location[1]-10.0)}\n\n")
        if abs(target_location[0]-10.0)+abs(target_location[1]-1.0)<0.0001:
            velocity = [0.0,0.0, -0.15,self.fitch_flag]
        elif abs(target_location[0]-1.0)+abs(target_location[1]-10.0)<0.0001:
            velocity = [0.0,0.0, 0.0,self.fitch_flag]

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
