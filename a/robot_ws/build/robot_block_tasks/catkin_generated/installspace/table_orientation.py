#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String
import numpy as np

class AlignWithTable:
    def __init__(self):
        rospy.init_node('align_with_table', anonymous=True)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.feedback_pub = rospy.Publisher('/table_orientation/feedback', Int32, queue_size=10)
        self.start_sub = rospy.Subscriber('table_orientation', String, self.start_callback)
        self.target_distance_diff = 0.004
        self.align_speed = 0.07
        self.alignment_count = 0
        self.alignment_active = False

    def start_callback(self, msg):
        if msg.data == "start":
            self.alignment_active = True
            self.alignment_count = 0

    def depth_callback(self, msg):
        if not self.alignment_active:
            return

        depth_image = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        h, w = depth_image.shape
        left_region = depth_image[h//2-10:h//2+10, :w//2]
        right_region = depth_image[h//2-10:h//2+10, w//2:]
        left_distance = np.nanmean(left_region) / 1000.0
        right_distance = np.nanmean(right_region) / 1000.0

        rospy.loginfo(f"Média do lado esquerdo: {left_distance:.4f} m, Média do lado direito: {right_distance:.4f} m")
        self.align_robot(left_distance, right_distance)

    def align_robot(self, left_distance, right_distance):
        cmd = Twist()
        distance_diff = left_distance - right_distance

        if abs(distance_diff) > self.target_distance_diff:
            cmd.linear.y = -self.align_speed * (distance_diff / abs(distance_diff))
            self.cmd_vel_pub.publish(cmd)
        else:
            cmd.linear.y = 0
            self.cmd_vel_pub.publish(cmd)
            self.alignment_count += 1
            rospy.loginfo(f"Alinhamento bem-sucedido #{self.alignment_count}")

            if self.alignment_count >= 3:
                rospy.loginfo("Robô alinhado 3 vezes. Publicando feedback.")
                self.feedback_pub.publish(1)
                self.alignment_active = False

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        aligner = AlignWithTable()
        aligner.run()
    except rospy.ROSInterruptException:
        pass
