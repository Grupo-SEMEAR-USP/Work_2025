#!/usr/bin/env python3
import rospy
import serial
import struct
import threading
from robot_base_controller.msg import encoder_data, velocity_data
from time import sleep
import time
import os

# Constantes
BAUD_RATE = 115200
HEADER_RX  = 0x02          # cabeçalho dos encoders
HEADER_TX  = 0x02          # cabeçalho dos comandos de motor

class UARTCommunication:
    def __init__(self, serial_port, role):
        self.serial_port = serial_port
        self.role = role            # "front" ou "rear"
        self.wheel_velocities = [0, 0]
        self.ser = None

        # Publisher e Subscriber
        self.pub_encoder = rospy.Publisher('/encoder_data', encoder_data, queue_size=10)
        self.sub_velocity = rospy.Subscriber('/velocity_command', velocity_data, self.velocity_callback)

        self.encoder_msg = encoder_data()
        self.lock = threading.Lock()

        # Thread principal de leitura/escrita
        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()

    # ------------------------------------------------------------------ #
    #  Abertura da serial: espera o dispositivo aparecer e tenta de novo #
    # ------------------------------------------------------------------ #
    def open_serial(self):
        while not rospy.is_shutdown():
            if not os.path.exists(self.serial_port):
                rospy.logwarn(f"[{self.role}] Porta {self.serial_port} não existe. Tentando novamente…")
                time.sleep(1)
                continue
            try:
                self.ser = serial.Serial(
                    self.serial_port,
                    BAUD_RATE,
                    timeout=0.05,      # timeout curto; chamamos read() o tempo todo
                    dsrdtr=False,
                    rtscts=False
                )
                rospy.loginfo(f"[{self.role}] Serial aberta em {self.serial_port}")
                sleep(2)              # aguarda ESP32 enumerar USB
                return
            except serial.SerialException as e:
                rospy.logwarn(f"[{self.role}] Falha ao abrir serial: {e}")
                time.sleep(1)

    # --------------------------------------------------------- #
    #  Callback de velocidade (ros topic /velocity_command)     #
    # --------------------------------------------------------- #
    def velocity_callback(self, msg):
        if not msg:
            return
        if self.role == "front":
            self.wheel_velocities[0] = int(msg.front_left_wheel  * 1000)
            self.wheel_velocities[1] = int(msg.front_right_wheel * 1000)
        else:  # rear
            self.wheel_velocities[0] = int(msg.rear_left_wheel  * 1000)
            self.wheel_velocities[1] = int(msg.rear_right_wheel * 1000)

    # --------------------------------------------------------- #
    #  Envia valores de motor para o ESP32                      #
    # --------------------------------------------------------- #
    def send_motor_values(self):
        payload = struct.pack(">Bii", HEADER_TX,
                              self.wheel_velocities[0],
                              self.wheel_velocities[1])
        self.ser.write(payload)

    # --------------------------------------------------------- #
    #  Lê encoders: procura header, valida tamanho e publica     #
    # --------------------------------------------------------- #
    def read_encoder_values(self):
        # 1) procurar header 0x02
        while True:
            hdr = self.ser.read(1)
            if not hdr:                      # timeout
                return
            if hdr[0] == HEADER_RX:         # achou
                break

        # 2) ler bytes restantes
        rest = self.ser.read(8)

        print(rest)

        if len(rest) != 8:
            rospy.logwarn(f"[{self.role}] Pacote incompleto (faltaram {8-len(rest)} bytes)")
            return

        data = hdr + rest
        header, value_r, value_l = struct.unpack(">Bii", data)

        # Debug em hexadecimal
        rospy.logdebug(f"[{self.role}] RX: " + " ".join(f"{b:02X}" for b in data))

        # 3) publica mensagem
        with self.lock:
            msg = encoder_data()
            if self.role == "front":
                msg.front_left_encoder_data  = value_l / 1000.0
                msg.front_right_encoder_data = value_r / 1000.0
            else:
                msg.rear_left_encoder_data   = value_l / 1000.0
                msg.rear_right_encoder_data  = value_r / 1000.0
            self.pub_encoder.publish(msg)

    # --------------------------------------------------------- #
    #  Loop principal da thread                                 #
    # --------------------------------------------------------- #
    def update(self):
        rate = rospy.Rate(50)               # 50 Hz
        while not rospy.is_shutdown():
            if self.ser is None or not self.ser.is_open:
                rospy.logwarn(f"[{self.role}] Serial desconectada. Tentando reconectar…")
                self.open_serial()
                continue
            try:
                self.send_motor_values()
                self.read_encoder_values()
            except (serial.SerialException, OSError) as e:
                rospy.logerr(f"[{self.role}] Erro na comunicação: {e}")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None             # força reabrir
            rate.sleep()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

# ---------------------------------------------------------------------- #
#  Nó principal                                                          #
# ---------------------------------------------------------------------- #
if __name__ == "__main__":
    try:
        rospy.init_node('uart_serial', anonymous=True)

        # Ajuste o role correto de cada porta
        front_uart = UARTCommunication("/dev/ttyUSB3", "front")
        # rear_uart  = UARTCommunication("/dev/ttyUSB1", "rear")

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    finally:
        front_uart.close()
        # rear_uart.close()
        print("Conexões seriais fechadas.")
