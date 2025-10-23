#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32, String, Float32
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist


class BlockSorter:
    def __init__(self):
        rospy.init_node('block_sorter')

        # Inicialização dos vetores
        self.original_order = []
        self.ordered_ids = []

        # Flags de controle
        self.mapping_active = False
        self.ultrasound_right = float('inf')

        # Subscritores
        self.mapping_trigger_sub = rospy.Subscriber("/mapping_ids", Int32, self.mapping_trigger_callback)
        self.aruco_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.aruco_callback)
        self.ultrasound_right_sub = rospy.Subscriber("ultrasound_sensor_right", Float32, self.ultrasound_right_callback)

        # Publicadores
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.mapped_data_pub = rospy.Publisher("/mapped_data", String, queue_size=10)

        self.rate = rospy.Rate(10)

    def mapping_trigger_callback(self, msg):
        if msg.data == 1:
            rospy.loginfo("Comando de mapeamento recebido.")
            self.mapping_active = True
            self.mapping_process()

    def aruco_callback(self, msg):
        if not self.mapping_active:
            return

        # Apenas os IDs são salvos se a posição do ArUco estiver no intervalo desejado
        for marker in msg.detections:
            aruco_id = marker.id[0]
            position = marker.pose.pose.pose.position

            # Considera a posição no plano da câmera
            x = position.x
            y = position.y

            # Define os limites do intervalo permitido (ajuste conforme necessário)
            x_center_min = -0.05  # Limite mínimo em x
            x_center_max = 0.05   # Limite máximo em x
            y_center_min = -0.05  # Limite mínimo em y
            y_center_max = 0.05   # Limite máximo em y

            # Verifica se o marcador está dentro dos limites definidos
            if x_center_min <= x <= x_center_max and y_center_min <= y <= y_center_max:
                if aruco_id not in self.original_order:
                    self.original_order.append(aruco_id)
                    rospy.loginfo(f"ArUco ID {aruco_id} detectado dentro do intervalo e salvo.")
            else:
                rospy.loginfo(f"ArUco ID {aruco_id} fora do intervalo. Não será salvo.")

    def ultrasound_right_callback(self, data):
        self.ultrasound_right = data.data / 100.0  # Converte de cm para metros

    def mapping_process(self):
        rospy.loginfo("Iniciando o mapeamento dos blocos.")
        twist = Twist()
        twist.angular.z = -0.5  # Rotaciona para buscar os ArUcos

        while not rospy.is_shutdown():
            if not self.mapping_active:
                rospy.loginfo("Mapeamento inativo. Aguardando novo comando.")
                return

            # Interrompe o movimento se o ultrassom direito detectar um obstáculo
            if self.ultrasound_right < 0.04:
                rospy.loginfo("Limite do ultrassom direito alcançado. Parando mapeamento.")
                break

            # Sai do loop se todos os ArUcos forem detectados
            if len(self.original_order) >= 5:
                rospy.loginfo("Todos os 5 ArUcos foram mapeados.")
                break

            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

        # Ordena os IDs
        self.ordered_ids = sorted(self.original_order)

        # Publica os dados mapeados
        self.publish_mapped_data()

        # Reseta o estado para aguardar próximo comando
        self.mapping_active = False
        self.original_order = []

    def publish_mapped_data(self):
        rospy.loginfo("Publicando dados mapeados.")
        # aux = self.original_order[0]
        # self.original_order[0] = self.original_order[1]
        # self.original_order[1] = aux

        # Convertendo os vetores de IDs para strings
        # Para o robô
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

        original_order_str = ",".join(map(str, self.original_order))
        ordered_ids_str = ",".join(map(str, self.ordered_ids))

        # Publicando os IDs como strings concatenadas
        self.mapped_data_pub.publish(f"{original_order_str};{ordered_ids_str}")
        rospy.loginfo("Dados publicados com sucesso.")


if __name__ == "__main__":
    try:
        sorter = BlockSorter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
