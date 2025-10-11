#!/usr/bin/env python3
import rospy
import json
import math
from paho.mqtt import client as mqtt
import threading

from robot_base_controller.msg import encoder_data, velocity_data
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

BROKER = "192.168.1.100"
PORT = 1883

TOPIC_ENCODER_FRONT = "estado/encoders/front"
TOPIC_ENCODER_REAR  = "estado/encoders/rear"
TOPIC_COMMAND_FRONT = "comandos/motores/front"
TOPIC_COMMAND_REAR  = "comandos/motores/rear"

TOPIC_CMD_MANIPULATOR = "cmd/manipulator"

TOPIC_STATE_US = "state/ultrassonics"

ENCODER_RESOLUTION = 8256

encoder_msg = encoder_data()
lock = threading.Lock()

def _is_num(x):
    return x is not None and not math.isnan(x) and not math.isinf(x)

class MQTTBridge:
    def __init__(self):
        # MQTT
        self.client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        # ROS pubs/subs existentes
        self.pub_encoder = rospy.Publisher('/encoder_data', encoder_data, queue_size=10)
        self.sub_cmd = rospy.Subscriber('/velocity_command', velocity_data, self.cmd_callback)

        # NOVO: publisher ROS para ultrassônicos
        self.pub_us = rospy.Publisher('/ultrassonics', Float32MultiArray, queue_size=10)

        # Subs para o manipulador
        self.sub_arm = rospy.Subscriber('/arm_control', Float32MultiArray, self.cb_arm, queue_size=10)
        self.sub_ee  = rospy.Subscriber('/end_effector_control', Float32MultiArray, self.cb_ee, queue_size=10)

        # Conexão broker
        self.client.connect(BROKER, PORT, 60)

        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def on_connect(self, client, userdata, flags, rc):
        rospy.loginfo("Conectado ao broker MQTT (rc=%s)", rc)
        client.subscribe([
            (TOPIC_ENCODER_FRONT, 0),
            (TOPIC_ENCODER_REAR, 0),
            (TOPIC_STATE_US, 0),            
        ])

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())

            if msg.topic == TOPIC_ENCODER_FRONT or msg.topic == TOPIC_ENCODER_REAR:
                left  = payload.get("left", 0)
                right = payload.get("right", 0)

                with lock:
                    if msg.topic == TOPIC_ENCODER_FRONT:
                        encoder_msg.front_left_encoder_data  = (left  / 1000.0)
                        encoder_msg.front_right_encoder_data = (right / 1000.0)
                    elif msg.topic == TOPIC_ENCODER_REAR:
                        encoder_msg.rear_left_encoder_data  = -(right / 1000.0)
                        encoder_msg.rear_right_encoder_data = -(left  / 1000.0)

                self.pub_encoder.publish(encoder_msg)
                return

            if msg.topic == TOPIC_STATE_US:
                arr = payload.get("dist_cm", [])
                vals = []
                for i in range(3):
                    v = arr[i] if i < len(arr) else None
                    if v is None:
                        vals.append(float('nan'))
                    else:
                        vals.append(float(v))
                self.pub_us.publish(Float32MultiArray(data=vals))
                return

        except Exception as e:
            rospy.logerr(f"Erro ao processar mensagem MQTT: {e}")

    def cmd_callback(self, msg):
        try:
            # REAR
            data_rear = {
                "left":  int(msg.rear_right_wheel * 1000),
                "right": int(msg.rear_left_wheel  * 1000)
            }
            # FRONT
            data_front = {
                "left":  int(msg.front_left_wheel  * 1000),
                "right": int(msg.front_right_wheel * 1000)
            }

            self.client.publish(TOPIC_COMMAND_FRONT, json.dumps(data_front))
            self.client.publish(TOPIC_COMMAND_REAR,  json.dumps(data_rear))

        except Exception as e:
            rospy.logerr(f"Erro ao enviar comandos MQTT: {e}")

    def cb_arm(self, msg: Float32MultiArray):
        """ /arm_control: [base_delta, arm_delta] """
        try:
            data = msg.data or []
            base_delta = data[0] if len(data) > 0 else None
            arm_delta  = data[1] if len(data) > 1 else None

            payload = {}
            if _is_num(base_delta) and abs(base_delta) > 0.0:
                payload["base"] = float(base_delta)
            if _is_num(arm_delta) and abs(arm_delta) > 0.0:
                payload["arm"] = float(arm_delta)

            if payload:
                for i in range(5):
                    self.client.publish(TOPIC_CMD_MANIPULATOR, json.dumps(payload))

        except Exception as e:
            rospy.logerr(f"Erro ao enviar arm_control -> MQTT: {e}")

    def cb_ee(self, msg: Float32MultiArray):
        """ /end_effector_control: [wrist_abs(0..180), grip_abs(0..180)] """
        try:
            data = msg.data or []
            wrist = data[0] if len(data) > 0 else None
            grip  = data[1] if len(data) > 1 else None

            payload = {}
            if _is_num(wrist):
                w = max(0.0, min(290.0, float(wrist)))
                payload["wrist"] = w
            if _is_num(grip):
                g = max(0.0, min(290.0, float(grip)))
                payload["grip"] = g

            if payload:
                for i in range(5):
                    self.client.publish(TOPIC_CMD_MANIPULATOR, json.dumps(payload))

        except Exception as e:
            rospy.logerr(f"Erro ao enviar end_effector_control -> MQTT: {e}")

    def loop(self):
        self.client.loop_forever()

if __name__ == "__main__":
    rospy.init_node('mqtt_bridge')
    bridge = MQTTBridge()
    rospy.spin()
