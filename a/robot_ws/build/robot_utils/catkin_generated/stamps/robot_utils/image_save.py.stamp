#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import numpy as np
from PIL import Image as PILImage
import os

class ImageSaver:
    def __init__(self):
        rospy.init_node('image_saver', anonymous=True)

        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)

        self.image_path = os.path.join(os.getcwd(), "Work_2024/jetson/catkin_ws/src/robot_utils/imgs/imagem_salva.png")
        
        self.saved_once = False

    def image_callback(self, data):
        if self.saved_once:
            return

        image_array = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)

        image = PILImage.fromarray(image_array)

        image.save(self.image_path)
        rospy.loginfo(f"Imagem salva em {self.image_path}")

        self.saved_once = True
        rospy.signal_shutdown("Imagem salva. Encerrando o script.")

    def start(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        image_saver = ImageSaver()
        image_saver.start()
    except rospy.ROSInterruptException:
        pass
