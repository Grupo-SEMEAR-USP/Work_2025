#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32

class MoveToTarget:
    def __init__(self):
        rospy.init_node('rs_distance', anonymous=True)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)

    def depth_callback(self, data):

        height = data.height
        width = data.width
        depth_data = np.frombuffer(data.data, dtype=np.uint16).reshape(height, width)
        center_distance = depth_data[height // 2, width // 2] / 1000.0
        rospy.loginfo("Distância do centro: %.2f metros" % center_distance)

if __name__ == '__main__':
    try:
        move_to_target = MoveToTarget()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
