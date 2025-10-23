#!/usr/bin/env python3
import rospy
import json
import math
from paho.mqtt import client as mqtt
import threading

from robot_base_controller.msg import encoder_data, velocity_data, UltrasonicDistances
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

BROKER = "192.168.1.100"
PORT = 1883

TOPIC_ENCODER = "estado/encoders"
TOPIC_COMMAND = "comandos/motores"

TOPIC_ULTRASONICS = "state/ultrassonics"

TOPIC_CMD_MANIPULATOR = "cmd/manipulator"

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

        # pubs com os ultrassônicos
        self.pub_ultrasonic = rospy.Publisher('/ultrasonic_distances', UltrasonicDistances, queue_size=10)
        
        # subs para o manipulador
        self.sub_arm = rospy.Subscriber('/arm_control', Float32MultiArray, self.cb_arm, queue_size=10)
        self.sub_ee  = rospy.Subscriber('/end_effector_control', Float32MultiArray, self.cb_ee, queue_size=10)

        # Conexão broker
        self.client.connect(BROKER, PORT, 60)

        self.ultrasonic_data = {
            "front_left": 0.0,
            "front_right": 0.0,
            "rear_left": 0.0,
            "rear_right": 0.0
        }
        self.ultrasonic_lock = threading.Lock()

        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def on_connect(self, client, userdata, flags, rc):
        rospy.loginfo("Conectado ao broker MQTT (rc=%s)", rc)
        client.subscribe([(TOPIC_ENCODER, 0),(TOPIC_ULTRASONICS, 0)])

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())

            if msg.topic == TOPIC_ENCODER:
                with lock:
                    left_front = payload.get("left_front" , 0)
                    right_front = payload.get("right_front" , 0)
                    left_rear = payload.get("left_rear" , 0)
                    right_rear = payload.get("right_rear" , 0)

                    encoder_msg.front_left_encoder_data = (left_front / 1000.0)
                    encoder_msg.front_right_encoder_data = (right_front / 1000.0)
                    encoder_msg.rear_left_encoder_data = -(right_rear / 1000.0)
                    encoder_msg.rear_right_encoder_data = -(left_rear / 1000.0)

                self.pub_encoder.publish(encoder_msg)

            elif msg.topic == TOPIC_ULTRASONICS:
                with self.ultrasonic_lock:
                    if "rear_left" in payload:
                        self.ultrasonic_data["rear_left"] = float(payload["rear_left"])
                    if "rear_right" in payload:
                        self.ultrasonic_data["rear_right"] = float(payload["rear_right"])
                    if "front_left" in payload:
                        self.ultrasonic_data["front_left"] = float(payload["front_left"])
                    if "front_right" in payload:
                        self.ultrasonic_data["front_right"] = float(payload["front_right"])
                self.publish_ultrasonics()

        except Exception as e:
            rospy.logerr(f"Erro ao processar mensagem MQTT: {e}")

    def publish_ultrasonics(self):
        # Cria a mensagem ROS com o novo tipo
        ultrasonic_msg = UltrasonicDistances()

        # Garante a leitura segura dos dados com o lock
        with self.ultrasonic_lock:
            # Atribui os valores aos campos nomeados (muito mais legível!)
            ultrasonic_msg.front_left  = self.ultrasonic_data["front_left"]
            ultrasonic_msg.front_right = self.ultrasonic_data["front_right"]
            ultrasonic_msg.rear_left   = self.ultrasonic_data["rear_left"]
            ultrasonic_msg.rear_right  = self.ultrasonic_data["rear_right"]

        # Publica a mensagem
        self.pub_ultrasonic.publish(ultrasonic_msg)

    def cmd_callback(self, msg):
        try:
            data = {
                "left_rear":  int(msg.rear_right_wheel * 1000),
                "right_rear": int(msg.rear_left_wheel  * 1000),
                "left_front":  int(msg.front_left_wheel  * 1000),
                "right_front": int(msg.front_right_wheel * 1000)
            }

            self.client.publish(TOPIC_COMMAND, json.dumps(data), qos=0)

        except Exception as e:
            rospy.logerr(f"Erro ao enviar comandos MQTT: {e}")

    def cb_arm(self, msg: Float32MultiArray):
        """ /arm_control: [base_delta, arm_delta] """
        print("oi")
        try:
            data = msg.data or []
            base_delta = data[0] if len(data) > 0 else None
            arm_delta  = data[1] if len(data) > 1 else None

            payload = {}
            # Envie somente campos válidos (firmware soma os deltas)
            if _is_num(base_delta) and abs(base_delta) > 0.0:
                payload["base"] = float(base_delta)
            if _is_num(arm_delta) and abs(arm_delta) > 0.0:
                payload["arm"] = float(arm_delta)

            if payload:
                self.client.publish(TOPIC_CMD_MANIPULATOR, json.dumps(payload))

        except Exception as e:
            rospy.logerr(f"Erro ao enviar arm_control -> MQTT: {e}")

    def cb_ee(self, msg: Float32MultiArray):
        """ /end_effector_control: [wrist_abs(0..180), grip_abs(0..180)] """
        print("oi")
        try:
            data = msg.data or []
            wrist = data[0] if len(data) > 0 else None
            grip  = data[1] if len(data) > 1 else None

            payload = {}
            if _is_num(wrist):
                w = max(0.0, min(180.0, float(wrist)))
                payload["wrist"] = w
            if _is_num(grip):
                g = max(0.0, min(180.0, float(grip)))
                payload["grip"] = g

            if payload:
                self.client.publish(TOPIC_CMD_MANIPULATOR, json.dumps(payload))

        except Exception as e:
            rospy.logerr(f"Erro ao enviar end_effector_control -> MQTT: {e}")

    def loop(self):
        self.client.loop_forever()

if __name__ == "__main__":
    rospy.init_node('mqtt_bridge')
    bridge = MQTTBridge()
    rospy.spin()
