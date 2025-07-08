#!/usr/bin/env python3
import rospy, struct, time, serial
from std_msgs.msg import Float32MultiArray

SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
REPEATS = 4
INTERVAL = 0.5
TX_PACKET_SIZE = 4 * 4  # 4 floats (elevator, base, gripper, wrist)
RX_PACKET_SIZE = 5 * 4  # 5 floats (ultrassônicos)
BYTE_INIT = b'\x7E'  # ~
BYTE_END  = b'\x7F' # DEL

class UART_ROS_Bridge:
    def __init__(self):
        rospy.init_node('uart_master_arduino', anonymous=True)
        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.05)
        time.sleep(2)  # Aguarda conexão estabilizar

        self.arm = [0.0, 0.0]
        self.ee  = [0.0, 0.0]
        self.repeats = 0
        self.last_tx = 0.0

        rospy.Subscriber('/arm_control', Float32MultiArray, self.cb_arm, queue_size=1)
        rospy.Subscriber('/end_effector_control', Float32MultiArray, self.cb_ee, queue_size=1)

        self.pub = rospy.Publisher('/ultrasonic', Float32MultiArray, queue_size=1)

    def cb_arm(self, msg):
        if len(msg.data) >= 2 and list(msg.data[:2]) != self.arm:
            self.arm = list(msg.data[:2])
            self.repeats = REPEATS

    def cb_ee(self, msg):
        if len(msg.data) >= 2 and list(msg.data[:2]) != self.ee:
            self.ee = list(msg.data[:2])
            self.repeats = REPEATS

    def send_once(self):
        payload = struct.pack('<4f', *(self.arm + self.ee))
        packet = BYTE_INIT + payload + BYTE_END
        self.ser.write(packet)

    def receive_ultrasonics(self):
        static_buffer = getattr(self, '_rx_buffer', bytearray())
        collecting = getattr(self, '_collecting', False)

        while self.ser.in_waiting:
            b = self.ser.read(1)
            if not b:
                return

            if not collecting:
                if b == BYTE_INIT:
                    static_buffer = bytearray()
                    collecting = True
            else:
                if b == BYTE_END:
                    if len(static_buffer) == RX_PACKET_SIZE:
                        try:
                            us = struct.unpack('<5f', static_buffer)
                            msg = Float32MultiArray(data=list(us))
                            self.pub.publish(msg)
                            print(msg)
                        except struct.error:
                            rospy.logwarn("Falha ao decodificar ultrassônicos")
                    else:
                        rospy.logwarn("Pacote com tamanho incorreto: %d bytes", len(static_buffer))
                    collecting = False
                else:
                    static_buffer.append(b[0])
                    if len(static_buffer) > RX_PACKET_SIZE:
                        rospy.logwarn("Overflow no pacote UART — reiniciando")
                        collecting = False

        # Armazena estados entre chamadas
        self._rx_buffer = static_buffer
        self._collecting = collecting


    def run(self):
        r = rospy.Rate(200)
        while not rospy.is_shutdown():
            '''if self.repeats and time.monotonic() - self.last_tx >= INTERVAL:
                self.send_once()
                self.last_tx = time.monotonic()
                self.repeats -= 1
'''
            self.send_once()
            self.receive_ultrasonics()
            r.sleep()

        self.ser.close()

if __name__ == '__main__':
    try:
        UART_ROS_Bridge().run()
    except rospy.ROSInterruptException:
        pass
