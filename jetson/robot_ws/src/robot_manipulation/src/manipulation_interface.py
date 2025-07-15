#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from robot_scheduler.msg import SchedulerCommand

TARGET_NAME = "manipulation"

arm_vals = [0.0, 0.0]
ee_vals = [0.0, 0.0]

ACTION_MAP = {
    "gripper": {
        "open": 170.0,
        "close": 0.0
    },
    "wrist": {
        "up": 170.0,
        "down": 0.0
    },
    "rotatory_base": {
        "front": -1.0,
        "back": -1.0,
        
        "front_to_deposit1_left": -1000.0,
        "front_to_deposit2_left": 1000.0,
        "front_to_deposit3_left": -900.0,

        "front_to_deposit1_right": 1.0,
        "front_to_deposit2_right": 2.0,
        "front_to_deposit3_right": 3.0,

        "deposit1_to_front_right": 3.0,
        "deposit2_to_front_right": 3.0,
        "deposit3_to_front_right": 3.0,

        "deposit1_to_front_left": 3.0,
        "deposit2_to_front_left": 3.0,
        "deposit3_to_front_left": 3.0
    },
    "arm": {
        "top": 1.0,
        
        "deposit": -1.0,

        "bottom0": -1.0,
        "bottom5": -1.0,
        "bottom10": -1.0,
        "bottom15": -1.0,
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
