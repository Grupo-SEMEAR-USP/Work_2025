#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Scheduler-Patrol – versão com identificação por ID, nome do nó e payload.

Mensagens recebidas em /scheduler_topic devem ser do tipo
`scheduler_msgs/TaskRequest`, definido com:

    int32  id
    string node_name
    string payload

Regras implementadas
--------------------
1.  O nó só reage quando `msg.node_name` é igual ao seu próprio nome
    (parâmetro ~node_name, padrão "scheduler_patrol").
2.  Se chegar uma mensagem com **o mesmo id** de uma já processada
    (executada ou ainda na fila), ela é ignorada.
3.  Enquanto um trecho está em execução, novas mensagens **com o mesmo id**
    também são descartadas (outras ids podem ser enfileiradas normalmente).

O payload mantém o formato “paraeleN”, indicando o número do trecho a percorrer.
"""

import rospy
import actionlib
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from scheduler_msgs.msg import TaskRequest        # <-- mensagem custom

# ─────────────────── CONFIGURAÇÃO DE WAYPOINTS ────────────────────
TRECHOS = {
    1: [
        ((-1.977, -1.021, 0.0),
         (0.0, 0.0,  0.7215208310173468, 0.6923927284482683)),
        ((-1.916,  1.927, 0.0),
         (0.0, 0.0, -0.00397220280374852, 0.999992110771323)),
    ],
    2: [
        (( 1.060,  0.954, 0.0),
         (0.0, 0.0, -0.7217794186514265, 0.692123161591352)),
        (( 1.800, -1.790, 0.0),
         (0.0, 0.0, -0.9933720755682208, 0.11494311410991477)),
    ],
    # … adicione quantos trechos quiser
}

REQUEST_TOPIC = "/scheduler/command"
END_TOPIC     = "/scheduler/feedback"

class SchedulerPatrol:
    def __init__(self):
        # Nome que este nó deve reconhecer nas mensagens recebidas
        self.node_name = rospy.get_param("~node_name", "scheduler_patrol")

        rospy.loginfo(f"[{self.node_name}] inicializando…")

        # Action client → move_base
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("Conectado ao move_base.")

        # Fila de (msg_id, trecho_id) a executar
        self.queue      = []
        self.busy       = False
        self.seen_ids   = set()     # para ignorar duplicatas

        # Subscriber no tópico de agendamento
        self.sub = rospy.Subscriber(
            REQUEST_TOPIC,
            TaskRequest,
            self.request_cb,
            queue_size=10,
        )

        # Publisher latchado para avisar conclusão
        self.pub_done = rospy.Publisher(END_TOPIC, Bool, queue_size=1, latch=True)

    # ───────────────────────── CALLBACK ──────────────────────────
    def request_cb(self, msg: TaskRequest):
        """
        Recebe TaskRequest, valida e põe na fila.

        Ignora se:
          • node_name diferente do esperado
          • id já visto
          • payload fora do formato "paraeleN"
          • trecho N não existe em TRECHOS
        """
        if msg.node_name != self.node_name:
            return                     # não é para mim

        if msg.id in self.seen_ids:
            rospy.logdebug(f"ID {msg.id} já recebido – ignorando.")
            return                     # duplicado

        payload = msg.payload or ""
        prefix  = "paraele"
        if not payload.startswith(prefix):
            rospy.logwarn(f"Payload inesperado: '{payload}'.")
            return

        try:
            trecho_id = int(payload[len(prefix):])
        except ValueError:
            rospy.logwarn(f"Falha ao converter '{payload}' em inteiro.")
            return

        if trecho_id not in TRECHOS:
            rospy.logwarn(f"Trecho {trecho_id} não está configurado.")
            return

        # Tudo certo → registrar e enfileirar
        self.seen_ids.add(msg.id)
        self.queue.append((msg.id, trecho_id))
        rospy.loginfo(f"Trecho {trecho_id} agendado (msg id={msg.id}).")

    # ────────────────────── AUXILIARES DE NAVEGAÇÃO ───────────────────────
    @staticmethod
    def build_goal(pose):
        """Converte tupla (pos, quat) em MoveBaseGoal."""
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
        """Percorre todos os waypoints do trecho via move_base."""
        rospy.loginfo(f"Iniciando trecho {trecho_id}…")
        waypoints = TRECHOS[trecho_id]

        for idx, wp in enumerate(waypoints, start=1):
            goal = self.build_goal(wp)
            self.client.send_goal(goal)
            rospy.loginfo(f"  Waypoint {idx}/{len(waypoints)} enviado.")
            self.client.wait_for_result()
            status = self.client.get_state()

            if status != GoalStatus.SUCCEEDED:
                rospy.logwarn(f"  Waypoint {idx} falhou (status={status}). Abortando trecho.")
                return

        # Todos os waypoints concluídos
        self.pub_done.publish(Bool(data=True))
        rospy.loginfo(f"Trecho {trecho_id} concluído; publicado em {END_TOPIC}.")

    # ───────────────────────── LOOP PRINCIPAL ────────────────────────────
    def spin(self):
        rate = rospy.Rate(10)          # 10 Hz
        while not rospy.is_shutdown():
            if not self.busy and self.queue:
                self.busy = True
                _msg_id, trecho_id = self.queue.pop(0)
                self.process_trecho(trecho_id)
                self.busy = False
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("scheduler_patrol")
    SchedulerPatrol().spin()
