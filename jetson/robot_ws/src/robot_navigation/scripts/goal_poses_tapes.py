#- Quando o limite de segmentação for atendido, quero analisar tambem se o goal publicado em comparação com a posição atual e verificar se vou passar pela tape
#Provavlemnte vou ter que verificar isso por meio da analise do path gerado pelo move_base e checar isso.
#Caso acontecer, preciso marcar como ocupado


#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
import cv2
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# ------------------- PARAMETROS DE DETECÇÃO DA FITA -------------------
# Amarelo (HSV)
Y_LOWER = np.array([12, 94, 77],  dtype=np.uint8)
Y_UPPER = np.array([51, 255, 255], dtype=np.uint8)
# Preto (HSV)  – intervalo amplo de H para pegar preto com matiz indefinida
B_LOWER = np.array([0, 35, 0],     dtype=np.uint8)
B_UPPER = np.array([119, 255, 54], dtype=np.uint8)
PERCENT_THRESHOLD = 0.50           # >50 % da imagem

# ------------------------------- WAYPOINTS ----------------------------
WAYPOINTS = [
    ((-1.977, -1.021, 0.0), (0.0, 0.0,  0.7215208310173468, 0.6923927284482683)),
    ((-1.916,  1.927, 0.0), (0.0, 0.0, -0.00397220280374852, 0.999992110771323)),
    (( 1.060,  0.954, 0.0), (0.0, 0.0, -0.7217794186514265, 0.692123161591352)),
    (( 1.800, -1.790, 0.0), (0.0, 0.0, -0.9933720755682208, 0.11494311410991477)),
    ((-0.478, -0.916, 0.0), (0.0, 0.0, -0.997827524800359, 0.06588042767612359)),
    (( 1.932,  1.303, 0.0), (0.0, 0.0,  0.6088056800085474, 0.7933193833440162))
]

def build_movebase_goal(pose_tuple):
    """Converte a tupla (pos, quat) num MoveBaseGoal."""
    pos, quat = pose_tuple
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z = pos
    goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, \
    goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w = quat
    return goal


class PatrolNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.blocked_by_tape = False             # define se a fita foi vista
        self.current_wp = 0                      # índice do waypoint corrente

        # --- move_base action client
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Aguardando move_base...")
        self.client.wait_for_server()

        # --- assinatura da câmera
        cam_topic = rospy.get_param("~camera_topic", "/camera/rgb/image_raw")
        self.img_sub = rospy.Subscriber(cam_topic, Image, self.image_cb, queue_size=1)

    # ----------------------- CALLBACK DE IMAGEM -----------------------
    def image_cb(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as err:
            rospy.logerr(err)
            return

        # Converte para HSV e segmenta
        hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask_y = cv2.inRange(hsv, Y_LOWER, Y_UPPER)
        mask_b = cv2.inRange(hsv, B_LOWER, B_UPPER)
        mask   = cv2.bitwise_or(mask_y, mask_b)

        # ** opcional: aplica operações morfológicas para reduzir ruído **
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))

        # Razão de pixels segmentados
        ratio = np.count_nonzero(mask) / float(mask.size)

        if ratio > PERCENT_THRESHOLD and not self.blocked_by_tape:
            rospy.logwarn("Fita amarela-preta detectada! ({} % da imagem)".format(round(ratio*100, 1)))
            self.client.cancel_goal()           # cancela o goal atual
            self.blocked_by_tape = True         # sinaliza que precisa pular waypoint

    # ----------------------------- LOOP -------------------------------
    def start_patrol(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.current_wp < len(WAYPOINTS):
            # Se não há goal ativo, manda o próximo
            if not self.client.simple_state in [actionlib.SimpleGoalState.ACTIVE,
                                                actionlib.SimpleGoalState.PENDING]:
                goal = build_movebase_goal(WAYPOINTS[self.current_wp])
                rospy.loginfo("Enviando waypoint {} / {}".format(self.current_wp + 1, len(WAYPOINTS)))
                self.client.send_goal(goal)

            # Se fita vista, avança para o próximo waypoint
            if self.blocked_by_tape:
                rospy.loginfo("Waypoint {} descartado devido à fita.".format(self.current_wp + 1))
                self.current_wp += 1
                self.blocked_by_tape = False
                # cancela ficou acima; basta pular sem `wait_for_result`
                continue

            # Se o goal foi concluído normalmente, avança
            if self.client.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo("Waypoint {} concluído.".format(self.current_wp + 1))
                self.current_wp += 1

            rate.sleep()

        rospy.loginfo("Patrulha terminada.")


if __name__ == "__main__":
    rospy.init_node("patrol_with_tape_stop")
    PatrolNode().start_patrol()
