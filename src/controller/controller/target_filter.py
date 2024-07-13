import rclpy  # ROS 2的Python接口
from rclpy.node import Node  # ROS 2的节点类
from std_msgs.msg import String, Float32MultiArray  # ROS标准消息类型
import json  # 用于解析和生成JSON字符串
import math

class Target:
    def __init__(self,position,confidence,class_name,target_type):
        self.position=position
        self.filted_position=position
        self.confidence=confidence
        self.continual_miss=0
        self.updated_flag=1
        self.preference=0.0
        self.aimed=False
        self.class_name=class_name
        self.type=target_type
    def update(self,position,confidence,class_name):
        self.position=position
        self.confidence=confidence
        self.updated_flag=1
        self.class_name=class_name
        self.filted_position[0]=0.5*self.filted_position[0]+0.5*position[0]
        self.filted_position[1]=0.5*self.filted_position[1]+0.5*position[1]
        self.filted_position[2]=0.5*self.filted_position[2]+0.5*position[2]
        distance=math.sqrt(self.filted_position[0]**2+self.filted_position[2]**2)
        self.preference=2000.0/distance+confidence-0.2*self.continual_miss
        if self.aimed and self.continual_miss<5:
            self.preference=100

class CoordinateListener(Node):
    def __init__(self):
        super().__init__('coordinate_listener')
        self.subscription = self.create_subscription(
            String,
            'socket_data',
            self.listener_callback,
            10)
        self.publisher_coordinate = self.create_publisher(String, 'coordinate', 10)
        # self.subscription_fetch = self.create_subscription(
        #     bool,
        #     'infetch',
        #     self.listener_callback_fetch,
        #     10)
        self.targets=[]
        self.current_target=Target([0,0,0],0,'','')
        self.current_destination=Target([0,0,0],0,'','')
        # self.state=False
        # self.infetch
        # self.target_type=["can","destination"]

    def listener_callback(self, data):
        self.process_coordinates(data.data)
    def process_coordinates(self, data):
        coordinates_dict = json.loads(data)
        # 发布u{'class': 'A', 'confidence': 0, 'coordinates': {'center': [0, 0],'calculated_3d': [10.0, 0.0, 1.0]}}
        #添加坐标
        for raw in coordinates_dict:
            distance_min=1000
            index_min=0
            target_type='can'
            if(raw['class']=='d'):
                target_type='destination'

            for index, target in enumerate(self.targets):
                x=raw['coordinates']['calculated_3d'][0]
                z=raw['coordinates']['calculated_3d'][2]
                distance=math.sqrt((x-target.position[0])**2+(z-target.position[2])**2)
                if distance<distance_min and target_type==target.type:
                    distance_min=distance
                    index_min=index
            if distance_min<50:
                self.targets[index_min].update([raw['coordinates']['calculated_3d'][0],raw['coordinates']['calculated_3d'][1],raw['coordinates']['calculated_3d'][2]],raw['confidence'],raw['class'])
            else:
                self.targets.append(Target([raw['coordinates']['calculated_3d'][0],raw['coordinates']['calculated_3d'][1],raw['coordinates']['calculated_3d'][2]],raw['confidence'],raw['class'],target_type) )
        #检测并删除过时坐标
        for target in self.targets:
            if target.updated_flag==0:
                target.continual_miss+=1
                if target.continual_miss>10:
                    self.targets.remove(target)
            else:
                target.updated_flag=0
                target.continual_miss=0
        #确定当前追踪目标
        index_target=0
        min_target_preference=0
        index_destination=0
        min_destination_preference=0

        for index, target in enumerate(self.targets):
            if target.type == 'can' and target.preference>min_target_preference:
                index_target=index
                min_target_preference=target.preference
            if target.type == 'destination' and target.preference>min_destination_preference:
                index_destination-index
                min_destination_preference=target.preference

        if min_target_preference != 0:  
            self.targets[index_target].aimed=True
            self.current_target=self.targets[index_target]
        else:
            self.current_target=Target([10,0,1],0,'','')

        if min_destination_preference !=0:
            self.targets[index_target].aimed=True
            self.current_destination=self.targets[index_target]
        else:
            self.current_destination=Target([10,0,1],0,'','')

        
        self.get_logger().info(f'Aimed target: {self.current_target.filted_position} Aimed destination: {self.current_destination.filted_position}')
        arget = json.dumps({
                "x": self.current_target.filted_position[0],
                "y": self.current_target.filted_position[1],
                "z": self.current_target.filted_position[2],
                "xt": self.current_destination.filted_position[0],
                "yt": self.current_destination.filted_position[1],
                "zt": self.current_destination.filted_position[2],
            })
        self.publisher_coordinate.publish(String(data=arget))

    def listener_callback_fetch(self,data):
        self.infetch = data
        


def main(args=None):
    rclpy.init(args=args)
    coordinate_listener = CoordinateListener()
    rclpy.spin(coordinate_listener)
    coordinate_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
