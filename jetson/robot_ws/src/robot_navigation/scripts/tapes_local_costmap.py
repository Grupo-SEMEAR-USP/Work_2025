#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters

# --- Filtros HSV para amarelo e preto
Y_LOWER = np.array([12, 94, 77],  dtype=np.uint8)
Y_UPPER = np.array([51, 255, 255], dtype=np.uint8)
B_LOWER = np.array([0, 35, 0],     dtype=np.uint8)
B_UPPER = np.array([119, 255, 54], dtype=np.uint8)
PERCENT_THRESHOLD = 0.05  # agora mais sensível

# --- Parâmetros intrínsecos da câmera (D435i padrão 640x480)
fx, fy = 617.0, 617.0
cx, cy = 319.5, 239.5

class TapeDetector:
    def __init__(self):
        self.bridge = CvBridge()

        # Sync RGB + Depth
        image_topic = rospy.get_param("~rgb_topic", "/camera/color/image_raw")
        depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_rect_raw")

        rgb_sub   = message_filters.Subscriber(image_topic, Image)
        depth_sub = message_filters.Subscriber(depth_topic, Image)

        ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.synced_callback)

        # Publicadores auxiliares
        self.mask_pub = rospy.Publisher("~tape_mask", Image, queue_size=1)

    def synced_callback(self, rgb_msg, depth_msg):
        try:
            bgr   = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as err:
            rospy.logerr(f"Erro ao converter imagens: {err}")
            return

        # Segmentação HSV
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask_y = cv2.inRange(hsv, Y_LOWER, Y_UPPER)
        mask_b = cv2.inRange(hsv, B_LOWER, B_UPPER)
        mask   = cv2.bitwise_or(mask_y, mask_b)

        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))

        # Publica máscara para visualização
        self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, encoding="mono8"))

        # Verifica se há fita suficiente
        ratio = np.count_nonzero(mask) / float(mask.size)
        if ratio < PERCENT_THRESHOLD:
            return

        rospy.loginfo("Fita detectada - {:.2f}% da imagem".format(ratio * 100))

        # Coleta pixels válidos
        ys, xs = np.where(mask > 0)
        for u, v in zip(xs, ys):
            z = depth[v, u] * 0.001  # converte para metros se 16UC1
            if z == 0 or z > 3.0:
                continue

            # Converte de pixel (u,v) para coordenada 3D na câmera
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            rospy.logdebug(f"Fita em: x={x:.2f} y={y:.2f} z={z:.2f}")

            # Aqui você pode transformar (x,y,z) para `odom` usando tf
            # E publicar como `OccupancyGrid`, `PointCloud2` ou `Costmap Layer`

if __name__ == "__main__":
    rospy.init_node("tape_detector_3d")
    rospy.loginfo("Detector de fita com profundidade inicializado.")
    TapeDetector()
    rospy.spin()
