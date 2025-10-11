#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import struct
import rospy
from smbus2 import SMBus, i2c_msg

from std_msgs.msg import Float32MultiArray
from robot_base_controller.msg import encoder_data, velocity_data
from robot_perception.msg import UltrasonicDistances

# ------------------------------
# Defaults (podem ser sobrescritos via rosparam ~)
# ------------------------------
DEFAULT_I2C_BUS = 1
ADDR_FRONT_DEF = 0x09
ADDR_REAR_DEF = 0x08
ADDR_ARDUINO_DEF = 0x0a
REG_ADDR_ENCODER_DEF = 10  # mesmo do seu script

ENCODER_RATE_HZ_DEF = 30.0
WHEEL_WRITE_RATE_HZ_DEF = 30.0
ULTRA_RATE_HZ_DEF = 50.0
ARM_INTERVAL_DEF = 0.5
REPEATS_DEF = 4
ROTATION_DIRECTION_OFFSET_DEF = 300.0

ENCODER_TOPIC_DEF = '/encoder_data'
ULTRA_TOPIC_DEF = '/ultrasonic_distances'

NUM_ULTRA_SENSORS = 5
ULTRA_PKT_LEN = NUM_ULTRA_SENSORS * 4  # 5 floats (little-endian)


class CombinedI2CMaster:
    """Nó agregador de todos os dispositivos I2C."""

    def __init__(self):
        # ---------------- Params ----------------
        self.i2c_bus_num = rospy.get_param('~i2c_bus', DEFAULT_I2C_BUS)
        self.addr_front = rospy.get_param('~addr_front', ADDR_FRONT_DEF)
        self.addr_rear = rospy.get_param('~addr_rear', ADDR_REAR_DEF)
        self.addr_arduino = rospy.get_param('~addr_arduino', ADDR_ARDUINO_DEF)
        self.reg_addr_encoder = rospy.get_param('~reg_address', REG_ADDR_ENCODER_DEF)

        self.encoder_rate_hz = float(rospy.get_param('~encoder_rate_hz', ENCODER_RATE_HZ_DEF))
        self.wheel_write_rate_hz = float(rospy.get_param('~wheel_write_rate_hz', WHEEL_WRITE_RATE_HZ_DEF))
        self.ultra_rate_hz = float(rospy.get_param('~ultra_rate_hz', ULTRA_RATE_HZ_DEF))
        self.arm_interval = float(rospy.get_param('~arm_interval', ARM_INTERVAL_DEF))
        self.repeats_default = int(rospy.get_param('~repeats', REPEATS_DEF))
        self.rotation_direction_offset = float(rospy.get_param('~rotation_direction_offset', ROTATION_DIRECTION_OFFSET_DEF))

        encoder_topic = rospy.get_param('~encoder_topic', ENCODER_TOPIC_DEF)
        ultra_topic = rospy.get_param('~ultra_topic', ULTRA_TOPIC_DEF)

        # ---------------- Estado braço/EE ----------------
        self.arm_raw = [0.0, 0.0]  # [base, arm] recebido
        self.arm = [0.0, 0.0]      # com correção de sentido
        self.ee = [0.0, 0.0]       # end-effector
        self.last_base_dir = 0     # -1, 0, +1
        self.repeats = 0           # quantas transmissões restantes
        self.last_arm_tx = 0.0     # (monotonic)

        # ---------------- Estado rodas ----------------
        # int escalado ×1000 (como no seu código original)
        self.front_cmd = [0, 0]  # [left, right]
        self.rear_cmd = [0, 0]   # [right, left] (mapeamento replicado abaixo)

        # ---------------- Estado encoders ----------------
        # brutos (inteiros) vindos das ESP32; global replicando seu padrão
        self.enc_global = [0, 0, 0, 0]  # [front_left, front_right, rear_left, rear_right] *bruto*

        # ---------------- Publishers ----------------
        self.pub_encoder = rospy.Publisher(encoder_topic, encoder_data, queue_size=10)
        self.pub_ultra = rospy.Publisher(ultra_topic, UltrasonicDistances, queue_size=10)

        # ---------------- Subscribers ----------------
        rospy.Subscriber('/velocity_command', velocity_data, self.cb_velocity, queue_size=1)
        rospy.Subscriber('/arm_control', Float32MultiArray, self.cb_arm, queue_size=1)
        rospy.Subscriber('/end_effector_control', Float32MultiArray, self.cb_ee, queue_size=1)

        # ---------------- I2C init ----------------
        self.bus = SMBus(self.i2c_bus_num)
        rospy.on_shutdown(self._on_shutdown)

        # ---------------- Timers internos ----------------
        self._last_encoders = 0.0
        self._last_wheels = 0.0
        self._last_ultra = 0.0

    # ==================================================
    # Callbacks ROS
    # ==================================================
    def cb_velocity(self, msg: velocity_data):
        # Escala para int ×1000
        # Frente segue ordem natural.
        self.front_cmd[0] = int(msg.front_left_wheel * 1000)
        self.front_cmd[1] = int(msg.front_right_wheel * 1000)

        # Traseira: replicar comportamento do seu script (inversão L/R ao atribuir)
        # Original:
        #   self.wheel_velocities[1] = int(msg.rear_left_wheel * 1000)
        #   self.wheel_velocities[0] = int(msg.rear_right_wheel * 1000)
        # Então pack('!ii', wheel[0], wheel[1]) → [rear_right, rear_left]
        self.rear_cmd[1] = int(msg.rear_left_wheel * 1000)
        self.rear_cmd[0] = int(msg.rear_right_wheel * 1000)

    def cb_arm(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        new_base = msg.data[0]
        new_arm = msg.data[1]

        if [new_base, new_arm] == self.arm_raw:
            return

        self.arm_raw = [new_base, new_arm]

        new_dir = 0
        if new_base > 0:
            new_dir = 1
        elif new_base < 0:
            new_dir = -1

        corrected_base = new_base
        if self.last_base_dir != 0 and new_dir != 0 and new_dir != self.last_base_dir:
            corrected_base = new_base + self.rotation_direction_offset * new_dir

        if new_dir != 0:
            self.last_base_dir = new_dir

        self.arm = [corrected_base, new_arm]
        self.repeats = self.repeats_default

    def cb_ee(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        ee_new = list(msg.data[:2])
        if ee_new == self.ee:
            return
        self.ee = ee_new
        self.repeats = self.repeats_default

    # ==================================================
    # I2C helpers - ESP32 encoders + rodas
    # ==================================================
    def _read_encoders(self, addr, is_front: bool):
        try:
            data = self.bus.read_i2c_block_data(addr, self.reg_addr_encoder, 9)
            if len(data) != 9:
                return False
            if data[0] != 0x01:  # header esperado
                return False
            value_right = struct.unpack('!i', bytes(data[1:5]))[0]
            value_left = struct.unpack('!i', bytes(data[5:9]))[0]

            if is_front:
                self.enc_global[0] = value_left
                self.enc_global[1] = value_right
            else:
                self.enc_global[2] = value_left
                self.enc_global[3] = value_right
            return True
        except Exception as e:
            rospy.logdebug_throttle(5.0, f'_read_encoders({hex(addr)}) falhou: {e}')
            return False

    def _write_wheels(self, addr, left_int, right_int):
        # big-endian 2 ints
        try:
            pkt = struct.pack('!ii', left_int, right_int)
            self.bus.write_i2c_block_data(addr, 0, list(pkt))
            return True
        except Exception as e:
            rospy.logdebug_throttle(5.0, f'_write_wheels({hex(addr)}) falhou: {e}')
            return False

    def _publish_encoders(self):
        msg = encoder_data()
        # Segue seu script original (note inversões rear L/R na publicação!)
        msg.front_left_encoder_data = self.enc_global[0] / 1000.0
        msg.front_right_encoder_data = self.enc_global[1] / 1000.0
        msg.rear_left_encoder_data = self.enc_global[3] / 1000.0  # invertido
        msg.rear_right_encoder_data = self.enc_global[2] / 1000.0 # invertido
        self.pub_encoder.publish(msg)

    # ==================================================
    # I2C helpers - Braço / EE (Arduino @0x0a)
    # ==================================================
    def _send_arm_ee(self):
        # little-endian 4 floats
        pkt = struct.pack('<4f', *(self.arm + self.ee))
        try:
            self.bus.i2c_rdwr(i2c_msg.write(self.addr_arduino, pkt))
            rospy.loginfo_throttle(2.0, f'[i2c_bridge] Enviando braço/EE: {self.arm + self.ee}')
            return True
        except Exception as e:
            rospy.logwarn_throttle(5.0, f'_send_arm_ee falhou: {e}')
            return False

    # ==================================================
    # I2C helpers - Ultrassônicos (mesmo Arduino @0x0a)
    # ==================================================
    def _read_ultrasonic(self):
        try:
            read = i2c_msg.read(self.addr_arduino, ULTRA_PKT_LEN)
            self.bus.i2c_rdwr(read)
            raw = bytes(read)
            if len(raw) != ULTRA_PKT_LEN:
                raise IOError('tamanho incorreto')
            vals = list(struct.unpack('<' + 'f'*NUM_ULTRA_SENSORS, raw))

            msg = UltrasonicDistances()
            # Mapeamento replicando seu script isolado
            msg.front_left  = vals[3]
            msg.front_right = vals[2]
            msg.left        = vals[1]
            msg.right       = vals[0]
            msg.rear        = vals[4]
            self.pub_ultra.publish(msg)
            return True
        except Exception as e:
            rospy.logwarn_throttle(5.0, f'I2C ultrassônico falhou: {e}')
            return False

    # ==================================================
    # Loop principal
    # ==================================================
    def spin(self):
        rate = rospy.Rate(max(self.encoder_rate_hz, self.wheel_write_rate_hz, self.ultra_rate_hz, 200))
        while not rospy.is_shutdown():
            now = time.monotonic()

            # Encoders
            if now - self._last_encoders >= (1.0 / self.encoder_rate_hz):
                self._read_encoders(self.addr_front, True)
                self._read_encoders(self.addr_rear, False)
                self._publish_encoders()
                self._last_encoders = now

            # Comando rodas
            if now - self._last_wheels >= (1.0 / self.wheel_write_rate_hz):
                # frente ordem natural
                self._write_wheels(self.addr_front, self.front_cmd[0], self.front_cmd[1])
                # traseira preserva inversão (rear_cmd[0] = right, [1] = left)
                self._write_wheels(self.addr_rear, self.rear_cmd[0], self.rear_cmd[1])
                self._last_wheels = now

            # Braço/EE (repetições cronometradas)
            if self.repeats and (now - self.last_arm_tx) >= self.arm_interval:
                if self._send_arm_ee():
                    self.last_arm_tx = now
                    self.repeats -= 1

            # Ultrassônicos
            if now - self._last_ultra >= (1.0 / self.ultra_rate_hz):
                self._read_ultrasonic()
                self._last_ultra = now

            rate.sleep()

    # ==================================================
    def _on_shutdown(self):
        try:
            self.bus.close()
        except Exception:
            pass


def main():
    rospy.init_node('combined_i2c_master', anonymous=True)
    node = CombinedI2CMaster()
    node.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
