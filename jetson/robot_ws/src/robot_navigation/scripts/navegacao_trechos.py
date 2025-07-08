#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from std_msgs.msg import Header, Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

# ————————————— CONFIGURAÇÃO DE WAYPOINTS POR TRECHO —————————————
# Aqui você define um dicionário onde a chave é o número do trecho
# e o valor é a lista de waypoints (tuplas de posição e orientação).
TRECHOS = {
    1: [
        ((-1.977, -1.021, 0.0),
         (0.0, 0.0,  0.7215208310173468, 0.6923927284482683)),
        ((-1.916,  1.927, 0.0),
         (0.0, 0.0, -0.00397220280374852, 0.999992110771323)),
    ],
    2: [
        (( 1.06,  0.954, 0.0),
         (0.0, 0.0, -0.7217794186514265, 0.692123161591352)),
        (( 1.80, -1.79,  0.0),
         (0.0, 0.0, -0.9933720755682208, 0.11494311410991477)),
    ],
    # adicione quantos trechos precisar…
}

# Tópico onde chega a mensagem do scheduler:
REQUEST_TOPIC = "/scheduler_topic"
END_TOPIC = "/scheduler_patrol_done"

class SchedulerPatrol:
    def __init__(self):
        rospy.loginfo("Inicializando nó de patrulha por scheduler…")

        # Action client para o move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("Conectado ao move_base.")

        # Fila de trechos a processar
        self.queue = []
        self.busy = False

        # Subscriber ao scheduler
        self.sub = rospy.Subscriber(REQUEST_TOPIC, Header, self.request_cb, queue_size=1)

    def request_cb(self, msg: Header):
        """
        Callback chamado sempre que o scheduler publica.
        Espera algo em msg.frame_id como "paraele3".
        """
        name = msg.frame_id or ""
        prefix = "paraele"
        if name.startswith(prefix):
            try:
                trecho_id = int(name[len(prefix):])
            except ValueError:
                rospy.logwarn(f"Scheduler enviou nome inválido: '{name}'")
                return

            if trecho_id not in TRECHOS:
                rospy.logwarn(f"Trecho {trecho_id} não configurado em TRECHOS.")
                return

            rospy.loginfo(f"Agendado trecho {trecho_id}.")
            self.queue.append(trecho_id)

    def build_goal(self, pose):
        """Constrói um MoveBaseGoal a partir da tupla (pos, quat)."""
        pos, quat = pose
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = pos[0]
        goal.target_pose.pose.position.y = pos[1]
        goal.target_pose.pose.position.z = pos[2]
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]
        return goal

    def process_trecho(self, trecho_id: int):
        """
        Envia cada waypoint do trecho para move_base e,
        ao final, publica True em /trecho_<n>/done.
        """
        rospy.loginfo(f"Iniciando processamento do trecho {trecho_id}…")
        waypoints = TRECHOS[trecho_id]

        for idx, wp in enumerate(waypoints, start=1):
            goal = self.build_goal(wp)
            self.client.send_goal(goal)
            rospy.loginfo(f"  Enviado waypoint {idx}/{len(waypoints)} do trecho {trecho_id}")
            self.client.wait_for_result()
            status = self.client.get_state()

            if status != GoalStatus.SUCCEEDED:
                rospy.logwarn(f"  waypoint {idx} do trecho {trecho_id} falhou (status={status}). Abortando trecho.")
                return

        # publicando conclusão
        pub_done = rospy.Publisher(END_TOPIC, Bool, queue_size=1, latch=True)
        rospy.sleep(0.5)  # garante conexão do publisher
        pub_done.publish(Bool(data=True))
        rospy.loginfo(f"Trecho {trecho_id} concluído. Publicado em '{END_TOPIC}'.")

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.busy and self.queue:
                self.busy = True
                trecho_id = self.queue.pop(0)
                self.process_trecho(trecho_id)
                self.busy = False
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("scheduler_patrol")
    node = SchedulerPatrol()
    node.spin()
