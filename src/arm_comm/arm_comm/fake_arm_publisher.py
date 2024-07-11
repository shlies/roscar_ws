import rospy
import threading
from std_msgs.msg import String
import json

class ArmPublisher:
    def __init__(self):
        rospy.init_node('arm_publisher_node', anonymous=True)
        self.publisher_arm = rospy.Publisher('arm', String, queue_size=10)
        
    def send_arm_message(self, target_coordinate, ox, oy, oz):
        target = json.dumps({
            "x": target_coordinate[0] + ox,
            "y": target_coordinate[1] + oy,
            "z": target_coordinate[2] + oz,
            "t": 3.14
        })
        self.publisher_arm.publish(String(data=target))
        
if __name__ == '__main__':
    try:
        arm_publisher = ArmPublisher()
        target_coordinate = [1.0, 2.0, 3.0]  # Example coordinates
        ox, oy, oz = 0.1, 0.2, 0.3  # Example offsets
        arm_publisher.send_arm_message(target_coordinate, ox, oy, oz)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
