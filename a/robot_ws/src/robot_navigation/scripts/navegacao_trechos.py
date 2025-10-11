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
from robot_scheduler.msg import SchedulerCommand        # <-- mensagem custom

# ─────────────────── CONFIGURAÇÃO DE WAYPOINTS ────────────────────


# pose: 
#   position: 
#     x: -12.550085067749023
#     y: -13.902693748474121
#     z: 0.0
#   orientation: 
#     x: 0.0
#     y: 0.0
#     z: 0.5995163251618421
#     w: 0.8003625277737836

# "header:
#   seq: 0
#   stamp: {secs: 0, nsecs: 0}
#   frame_id: ''
# uid: 0
# target: 'navigation'
# payload: 'TESTE'
# need_ack: false" 


TRECHOS = {
    "TESTE": [
        ((-12.55, -13.9, 0.0), (0, 0, 0.59, 0.8)),
    ],
    "Start": [
        ((0.0, 0.0, 0.0), (0, 0, 0, 1)),
    ],
    "SH01": [
        ((1.0, 1.0, 0.0), (0, 0, 0, 1)),
    ],
    "SH02": [
        ((2.0, 2.0, 0.0), (0, 0, 0, 1)),
    ],
    "RT01": [
        ((3.0, 3.0, 0.0), (0, 0, 0, 1)),
    ],
    "PP01": [
        ((4.0, 4.0, 0.0), (0, 0, 0, 1)),
    ],
    "WS01": [
        ((5.0, 5.0, 0.0), (0, 0, 0, 1)),
    ],
    "WS02": [
        ((6.0, 6.0, 0.0), (0, 0, 0, 1)),
    ],
    "WS03": [
        ((7.0, 7.0, 0.0), (0, 0, 0, 1)),
    ],
    "WS04": [
        ((8.0, 8.0, 0.0), (0, 0, 0, 1)),
    ],
    "WS05": [
        ((9.0, 9.0, 0.0), (0, 0, 0, 1)),
    ],
    "WS06": [
        ((10.0, 10.0, 0.0), (0, 0, 0, 1)),
    ],
    "WS07": [
        ((11.0, 11.0, 0.0), (0, 0, 0, 1)),
    ],
    "WS08": [
        ((12.0, 12.0, 0.0), (0, 0, 0, 1)),
    ],
    "WS09": [
        ((13.0, 13.0, 0.0), (0, 0, 0, 1)),
    ],
    "WS10": [
        ((14.0, 14.0, 0.0), (0, 0, 0, 1)),
    ],
    "WS11": [
        ((15.0, 15.0, 0.0), (0, 0, 0, 1)),
    ],
    "WS12": [
        ((16.0, 16.0, 0.0), (0, 0, 0, 1)),
    ],
    "WS13": [
        ((17.0, 17.0, 0.0), (0, 0, 0, 1)),
    ],
    "WS14": [
        ((18.0, 18.0, 0.0), (0, 0, 0, 1)),
    ],
}


REQUEST_TOPIC = "/scheduler/command"
END_TOPIC     = "/scheduler/feedback"

class SchedulerPatrol:
    def __init__(self):
        self.node_name = rospy.get_param("~node_name", "scheduler_patrol")

        rospy.loginfo(f"[{self.node_name}] inicializando…")

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("Conectado ao move_base.")

        self.queue      = []
        self.busy       = False
        self.seen_ids   = set()

        self.sub = rospy.Subscriber(
            REQUEST_TOPIC,
            SchedulerCommand,
            self.request_cb,
            queue_size=10,
        )

        self.pub_done = rospy.Publisher(END_TOPIC, Bool, queue_size=1, latch=True)

    def request_cb(self, msg: SchedulerCommand):
        """
        Recebe SchedulerCommand, valida e põe na fila.

        Ignora se:
          • node_name diferente do esperado
          • id já visto
          • payload fora do formato "paraeleN"
          • trecho N não existe em TRECHOS
        """
        if msg.node_name != self.node_name:
            return

        if msg.id in self.seen_ids:
            rospy.logdebug(f"ID {msg.id} já recebido – ignorando.")
            return

        trecho_name = (msg.payload or "").strip()
        if not trecho_name:
            rospy.logwarn("Payload vazio.")
            return

        if trecho_name not in TRECHOS:
            rospy.logwarn(f"Trecho '{trecho_name}' não está configurado.")
            return

        self.seen_ids.add(msg.id)
        self.queue.append((msg.id, trecho_name))
        rospy.loginfo(f"Trecho '{trecho_name}' agendado (msg id={msg.id}).")

    @staticmethod
    def build_goal(pose):
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

    def process_trecho(self, trecho_name: str):
        rospy.loginfo(f"Iniciando trecho '{trecho_name}'…")
        waypoints = TRECHOS[trecho_name]

        for idx, wp in enumerate(waypoints, start=1):
            goal = self.build_goal(wp)
            self.client.send_goal(goal)
            rospy.loginfo(f"  Waypoint {idx}/{len(waypoints)} enviado.")
            self.client.wait_for_result()
            status = self.client.get_state()

            if status != GoalStatus.SUCCEEDED:
                rospy.logwarn(f"  Waypoint {idx} falhou (status={status}). Abortando trecho.")
                return

        self.pub_done.publish(Bool(data=True))
        rospy.loginfo(f"Trecho '{trecho_name}' concluído; publicado em {END_TOPIC}.")

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.busy and self.queue:
                self.busy = True
                _msg_id, trecho_name = self.queue.pop(0)
                self.process_trecho(trecho_name)
                self.busy = False
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("scheduler_patrol")
    SchedulerPatrol().spin()

'''
Pontos importantes:
    O nó SchedulerPatrol funciona como um cliente (SimpleActionClient) do servidor move_base, responsável 
    por executar metas de navegação.
    As mensagens chegam pelo tópico /scheduler/command e são processadas pela callback request_cb, que valida 
    node_name, id e payload. Cada requisição válida é enfileirada para execução.
    O ciclo principal (spin) verifica se há trechos pendentes e, quando livre, chama process_trecho.
    Nessa função, cada waypoint é enviado ao servidor (send_goal), aguardado (wait_for_result) e checando (get_state).
    Se todos os pontos forem concluídos com sucesso, o nó publica uma confirmação (Bool) em /scheduler/feedback.    
'''