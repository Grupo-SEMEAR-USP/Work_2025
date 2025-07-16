#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import os
import datetime

class ImageSaver:
    def __init__(self):
        rospy.init_node('image_saver', anonymous=True)

        # Tópico da Realsense
        self.image_sub = rospy.Subscriber(
            "/usb_cam/image_raw",
            Image,
            self.image_callback
        )

        # Caminho correto da pasta
        self.image_dir = os.path.expanduser("~/Documents/Work_2025/jetson/robot_ws/src/robot_utils/imgs")
        os.makedirs(self.image_dir, exist_ok=True)

        self.saved_once = False
        self.bridge = CvBridge()

    def image_callback(self, data):
        if self.saved_once:
            return

        try:
            # Converter mensagem ROS para OpenCV image (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

            # Nome do arquivo com timestamp
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"image_{timestamp}.png"
            filepath = os.path.join(self.image_dir, filename)

            # Salvar com OpenCV
            import cv2
            cv2.imwrite(filepath, cv_image)

            rospy.loginfo(f"Imagem salva em {filepath}")

            self.saved_once = True
            rospy.signal_shutdown("Imagem salva. Encerrando o script.")
        except Exception as e:
            rospy.logerr(f"Erro ao processar imagem: {e}")

    def start(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        image_saver = ImageSaver()
        image_saver.start()
    except rospy.ROSInterruptException:
        pass
