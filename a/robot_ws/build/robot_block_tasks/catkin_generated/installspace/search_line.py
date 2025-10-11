#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Float32


class BlockSearcher:
    def __init__(self):
        rospy.init_node('block_searcher')

        # Variáveis de controle
        self.is_executing = False
        self.ultrasound_right = float('inf')
        self.ultrasound_left = float('inf')
        self.detected_ids = []
        self.valid_detections_count = 0

        # Publicadores e subscritores
        self.search_feedback_pub = rospy.Publisher("/search_id_decision_feedback", Int32, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.search_sub = rospy.Subscriber("/search_id_decision", String, self.search_callback)
        self.aruco_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.aruco_callback)
        self.ultrasound_right_sub = rospy.Subscriber("ultrasound_right", Float32, self.ultrasound_right_callback)
        self.ultrasound_left_sub = rospy.Subscriber("ultrasound_left", Float32, self.ultrasound_left_callback)

        self.rate = rospy.Rate(10)
        rospy.loginfo("Sistema em standby aguardando comandos.")
        self.standby_loop()

    def standby_loop(self):
        while not rospy.is_shutdown():
            if not self.is_executing:
                rospy.sleep(0.1)
            else:
                rospy.loginfo("Busca em execução.")
                self.rate.sleep()

    def search_callback(self, msg):
        if self.is_executing:
            rospy.loginfo("O sistema já está executando uma busca.")
            return

        self.is_executing = True
        self.valid_detections_count = 0  
        rospy.loginfo(f"Recebido: {msg.data}")

        try:
            pos1, pos2, element_id = map(int, msg.data.strip("()").split(","))
            rospy.loginfo(f"Parsed: pos1={pos1}, pos2={pos2}, element_id={element_id}")
        except ValueError:
            rospy.logerr("Erro ao interpretar a mensagem recebida. Formato esperado: '(pos1, pos2, element_id)'")
            self.is_executing = False
            return

        # Decide a busca
        if pos1 == -1 and pos2 == -1:
            self.search_left(element_id)
        else:
            self.search_for_specific_id(pos1, pos2, element_id)

        # Publica feedback após término
        self.search_feedback_pub.publish(Int32(1))
        rospy.loginfo("Busca concluída, retornando ao estado standby.")
        self.is_executing = False

    def aruco_callback(self, msg):
        if not self.is_executing:
            return
        self.detected_ids = [detection.id[0] for detection in msg.detections]

    def ultrasound_right_callback(self, data):
        if not self.is_executing:
            return
        self.ultrasound_right = data.data / 100.0  # Converte para metros

    def ultrasound_left_callback(self, data):
        if not self.is_executing:
            return
        self.ultrasound_left = data.data / 100.0  # Converte para metros

    def search_for_specific_id(self, pos1, pos2, element_id):
        rospy.loginfo(f"Iniciando busca para o ID {element_id} com pos1={pos1} e pos2={pos2}.")
        if pos1 < pos2:
            self.search_right(element_id)
        elif pos1 > pos2:
            self.search_left(element_id)
        else:
            rospy.loginfo("Posições iguais. Nenhuma busca necessária.")

    def search_left(self, element_id):
        rospy.loginfo(f"Iniciando busca para a esquerda pelo ID {element_id}.")
        twist = Twist()
        twist.angular.z = 0.5  # Gira para a esquerda

        while not rospy.is_shutdown() and self.is_executing:
            if self.ultrasound_left < 0.1:  # Limite do ultrassom à esquerda
                rospy.loginfo("Limite do ultrassom esquerdo alcançado.")
                break

            if element_id in self.detected_ids:
                self.valid_detections_count += 1
                rospy.loginfo(f"Leitura válida #{self.valid_detections_count} para o ID {element_id} à esquerda.")
                if self.valid_detections_count >= 5:
                    rospy.loginfo("Busca concluída com 5 leituras válidas seguidas à esquerda.")
                    break
            else:
                # Reinicia o contador se a leitura não for válida
                self.valid_detections_count = 0
                rospy.loginfo("Leitura inválida. Reiniciando contador.")

            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def search_right(self, element_id):
        rospy.loginfo(f"Iniciando busca para a direita pelo ID {element_id}.")
        twist = Twist()
        twist.angular.z = -0.5  # Gira para a direita

        while not rospy.is_shutdown() and self.is_executing:
            if self.ultrasound_right < 0.1:  # Limite do ultrassom à direita
                rospy.loginfo("Limite do ultrassom direito alcançado.")
                break

            if element_id in self.detected_ids:
                self.valid_detections_count += 1
                rospy.loginfo(f"Leitura válida #{self.valid_detections_count} para o ID {element_id} à direita.")
                if self.valid_detections_count >= 10:
                    rospy.loginfo("Busca concluída com 5 leituras válidas seguidas à direita.")
                    break
            else:
                # Reinicia o contador se a leitura não for válida
                self.valid_detections_count = 0
                rospy.loginfo("Leitura inválida. Reiniciando contador.")

            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)


if __name__ == '__main__':
    try:
        BlockSearcher()
    except rospy.ROSInterruptException:
        pass
