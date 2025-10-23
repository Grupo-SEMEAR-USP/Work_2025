#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from robot_scheduler.msg import SchedulerCommand

TARGET_NAME = "manipulation"

arm_vals = [0.0, 0.0] #[base, arm]
ee_vals = [0.0, 0.0] #[wrist, gripper]

ACTION_MAP = {
    "gripper": {
        "open": 65.0, #270 é o max da garra (tava 160.0)
        "close": 15.0 #0 é o min da garra (tava 10.0 - valor com o bloco)
    },
    "wrist": {
        "center": 80.0,
    },
    "rotatory_base": {
        "front": -1.0,
        "back": -1.0,
        
        "front_to_deposit1_left": 3700.0,
        "front_to_deposit2_left": 4000.0,
        "front_to_deposit3_left": 4300.0,

        "front_to_deposit1_right": -3700.0,
        "front_to_deposit2_right": -4000.0,
        "front_to_deposit3_right": -4300.0,

        "deposit1_to_front_right": 3700.0,
        "deposit2_to_front_right": 4000.0,
        "deposit3_to_front_right": 4300.0,

        "deposit1_to_front_left": -3700.0,
        "deposit2_to_front_left": -4000.0,
        "deposit3_to_front_left": -4300.0,

        "deposit2_to_deposit1": -300.0,
        "deposit1_to_deposit2": 300.0,
        "deposit1_to_deposit3": 600.0,
        "deposit3_to_deposit1": -600.0,
        "deposit2_to_deposit3": 300.0,
        "deposit3_to_deposit2": -300.0,

    },
    "arm": {
        "top": 1.0,

        "deposit_to_top": 10100.0,
        "top_to_deposit": -10100.0,
        
        "top_to_deposit_put": -9600.0,
        "top_to_deposit_pick": -9900.0,

        "top_to_bottom0": -14400.0,
        "bottom0_to_top": 14400.0,
        "top_to_bottom5": -12100.0,
        "bottom5_to_top": 12100.0,
        "top_to_bottom10": -10200.0,
        "bottom10_to_top": 10200.0,
        "top_to_bottom15": -8000.0,
        "bottom15_to_top": 8000.0, #botton15 = altura da mesa de 15

        "bottom15_to_deposit": -2900.0, #medidas intermediarias

        "deposit_to_bottom10": -1.0,
    }
}

class ManipulationInterface:
    def __init__(self):
        rospy.init_node("manipulation_interface")

        self.pub_arm = rospy.Publisher("/arm_control", Float32MultiArray, queue_size=10)
        self.pub_ee = rospy.Publisher("/end_effector_control", Float32MultiArray, queue_size=10)

        rospy.Subscriber("/scheduler/commands",
                         SchedulerCommand, self.cb_command, queue_size=10)

        rospy.loginfo("[manipulation_interface] Inicializado e aguardando comandos...")

    def cb_command(self, msg):
        if msg.target != TARGET_NAME:
            return

        rospy.loginfo(f"[manipulation_interface] Recebido uid={msg.uid} payload='{msg.payload}'")

        parts = [p.strip().lower() for p in msg.payload.split(",")]
        if len(parts) != 2:
            rospy.logerr("Payload inválido. Esperado formato: '<componente>, <ação>'")
            return

        component, action = parts

        if component not in ACTION_MAP:
            rospy.logerr(f"Componente desconhecido: '{component}'")
            return

        if action not in ACTION_MAP[component]:
            rospy.logerr(f"Ação desconhecida: '{action}' para componente '{component}'")
            return

        value = ACTION_MAP[component][action]

        if component in ["rotatory_base", "arm"]:
            idx = 0 if component == "rotatory_base" else 1
            arm_vals[idx] = value
            self.pub_arm.publish(Float32MultiArray(data=arm_vals))
            rospy.loginfo(f"[manipulation_interface] Publicado arm_control: {arm_vals}")
        else:
            idx = 0 if component == "wrist" else 1
            ee_vals[idx] = value
            self.pub_ee.publish(Float32MultiArray(data=ee_vals))
            rospy.loginfo(f"[manipulation_interface] Publicado end_effector_control: {ee_vals}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        ManipulationInterface().run()
    except rospy.ROSInterruptException:
        pass
