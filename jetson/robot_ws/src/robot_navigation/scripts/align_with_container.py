
#!/usr/bin/env python3

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32, Float32
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import os

class SearchForContainer:
    def __init__(self):
        rospy.init_node('align_container', anonymous=True)

        self.detected_blocks = {}
        self.camera_image = None
        self.image_received = False
        self.ocioso = True
        self.detect_enabled = False  # Variável de controle para iniciar a detecção
        self.target_color = None
        self.found_color = False
        self.ultrasound_right = 0

        # Limites HSV para as cores
        self.blue_lower_hsv = np.array([90, 21, 0])
        self.blue_upper_hsv = np.array([122, 185, 123])
        self.red_lower_hsv1 = np.array([0, 188, 32])
        self.red_upper_hsv1 = np.array([179, 255, 151])

        #(hMin = 90 , sMin = 21, vMin = 0), (hMax = 122 , sMax = 185, vMax = 123)
        #(hMin = 0 , sMin = 188, vMin = 32), (hMax = 179 , sMax = 255, vMax = 151)



        self.image_save_dir = "/home/rmajetson/Work_2024/"
        os.makedirs(self.image_save_dir, exist_ok=True)

        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_detections_callback)
        rospy.Subscriber('/usb_cam/image_raw', Image, self.camera_callback)
        rospy.Subscriber('/goal_find_container', String, self.goal_callback)
        rospy.Subscriber("/ultrasound_sensor_left", Float32, self.ultrasound_left_callback)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_move_time = rospy.Publisher('/move_time', String, queue_size=10)
        self.block_color_feedback = rospy.Publisher('/find_container_feedback', Int32, queue_size=10)

        self.close_to_container = False

    def goal_callback(self, msg):
        self.target_color = msg.data.lower()
        self.detect_enabled = True
        self.ocioso = False
        rospy.loginfo(self.target_color)
        self.detected_blocks.clear()


    def ultrasound_left_callback(self, data):
        if not self.ocioso:
            self.ultrasound_right = data.data / 100.0  # Acessa o valor com data.data

            if self.ultrasound_right <= 0.15:  # Corrigido para comparar com a distância correta
                self.close_to_container = True


    def create_mask(self, image, left_limit, right_limit):
        h, w = image.shape[:2]
        left_mask_points = np.array([(0, 0), (left_limit, 0), (left_limit, h), (0, h)])
        right_mask_points = np.array([(right_limit, 0), (w, 0), (w, h), (right_limit, h)])
        
        mask = np.ones_like(image) * 255
        cv2.fillPoly(mask, [left_mask_points], (0, 0, 0))
        cv2.fillPoly(mask, [right_mask_points], (0, 0, 0))
        
        return cv2.bitwise_and(image, mask)

    def find_container_color(self, roi, tag_id):
        hsv_image = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        red_mask1 = cv2.inRange(hsv_image, self.red_lower_hsv1, self.red_upper_hsv1)
        blue_mask = cv2.inRange(hsv_image, self.blue_lower_hsv, self.blue_upper_hsv)

        red_pixels = cv2.countNonZero(red_mask1)
        blue_pixels = cv2.countNonZero(blue_mask)

        if red_pixels > blue_pixels:
            return "red"
        elif blue_pixels > red_pixels:
            return "blue"
        return "unknown"

    def camera_callback(self, img_msg):
        if not self.ocioso:
            try:
                if img_msg.encoding != "rgb8":
                    rospy.logwarn(f"Formato de imagem inesperado: {img_msg.encoding}")
                    return
                self.camera_image = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, 3)
                self.camera_image = cv2.cvtColor(self.camera_image, cv2.COLOR_RGB2BGR)
                self.image_received = True
            except Exception as e:
                rospy.logerr(f"Erro ao converter imagem: {e}")
                self.image_received = False

    def tag_detections_callback(self, data):
        if not self.detect_enabled or self.ocioso:
            return
        if len(data.detections) == 0 or not self.image_received or self.camera_image is None:
            return

        masked_image = self.create_mask(self.camera_image, 170, 470)
        for detection in data.detections:
            tag_id = detection.id[0]
            
            position = detection.pose.pose.pose.position
            x, y = int(position.x * 1000) + 220, int(position.y * 1000) + 190
            w, h = 180, 180

            x = max(0, min(masked_image.shape[1] - w, x))
            y = max(0, min(masked_image.shape[0] - h, y))
            
            rospy.loginfo(self.detected_blocks)

            if 170 <= x <= 470 and abs(position.x * 1000) < 100:
                roi = masked_image[y:y+h, x:x+w]
                color = self.find_container_color(roi, tag_id)
                
                
                if color == self.target_color:
                    self.detected_blocks[tag_id] = color
                    rospy.loginfo(f"Bloco ID {tag_id} identificado como {color}")
                    self.block_color_feedback.publish(1)
                    self.pub_move_time.publish("esquerda,0.5")
                    rospy.sleep(0.5)
                    self.ocioso = True
                    self.found_color = True
                    self.stop_movement()
                    rospy.loginfo("Detecção completa")
                    return

    def stop_movement(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.detect_enabled and not self.close_to_container and not self.ocioso:
                twist = Twist()
                twist.angular.z = 0.5
                self.cmd_vel_pub.publish(twist)
            elif self.close_to_container or self.found_color:
                self.found_color = False
                self.close_to_container = False
                self.stop_movement()
                self.detect_enabled = False
                self.ocioso = True
                rospy.loginfo("Perto o suficiente do contêiner")

            rate.sleep()

if __name__ == '__main__':
    try:
        color_detect = SearchForContainer()
        color_detect.run()
    except rospy.ROSInterruptException:
        pass
