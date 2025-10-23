#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ajusta automaticamente o pulso da garra com base no objeto detectado
quando o nome do objeto no tópico /identificacao for igual a "Allen".
Publica comandos de ajuste de pulso via /scheduler/commands no formato definido.
"""

import rospy
from robot_perception.msg import Identify
from robot_scheduler.msg  import SchedulerCommand

BURST_REPEATS = 8          # igual ao seu Scheduler
BURST_RATE_HZ = 10

class WristAdjuster:
    def __init__(self):
        rospy.init_node("wrist_adjuster")

        # --- parâmetros configuráveis pelo rosparam ---
        self.target_class = rospy.get_param("~target_class", "Allen")  # Nome do objeto a ser ajustado
        self.need_ack     = bool(rospy.get_param("~need_ack", True))   # Se precisa de ACK
        self.debounce_s   = float(rospy.get_param("~debounce_seconds", 0.5))  # Debounce para evitar múltiplos comandos

        # --- publishers / subscribers ---
        self.pub_cmd = rospy.Publisher("/scheduler/commands", SchedulerCommand, queue_size=30)
        rospy.Subscriber("/identificacao", Identify, self.cb_identify, queue_size=5)

        # estado interno
        self._last_time = rospy.Time(0)
        self._burst_rate = rospy.Rate(BURST_RATE_HZ)

        rospy.loginfo("[wrist_adjuster] pronto ‒ esperando detecções ...")

    # ------------------------------------------------------------------
    # CALLBACK PRINCIPAL
    # ------------------------------------------------------------------
    def cb_identify(self, msg: Identify):
        # há a classe de interesse (objeto "Allen")?
        if self.target_class not in msg.classes:
            return

        # encontra o índice do objeto "Allen" no payload da identificação
        idx = msg.classes.index(self.target_class)
        
        # --- Publicar o comando de ajuste do pulso
        self._send_wrist_adjust_cmd()

    # ------------------------------------------------------------------
    # ENVIA COMANDO DE AJUSTE DE PULSO
    # ------------------------------------------------------------------
    def _send_wrist_adjust_cmd(self):
        uid = int(rospy.Time.now().to_nsec())
        payload = f"wrist_adjust,object={self.target_class}"
        cmd = SchedulerCommand(uid=uid, target="manipulation", payload=payload, need_ack=self.need_ack)

        for _ in range(BURST_REPEATS):  # mesmo padrão de envio com burst
            self.pub_cmd.publish(cmd)
            self._burst_rate.sleep()

        rospy.loginfo(f"[wrist_adjuster] → {payload} (uid={uid})")

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        WristAdjuster().spin()
    except rospy.ROSInterruptException:
        pass
