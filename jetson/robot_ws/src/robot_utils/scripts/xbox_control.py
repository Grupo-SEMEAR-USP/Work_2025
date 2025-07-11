#!/usr/bin/env python3
import rospy, math, time
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

# -------------------- configurações --------------------
AXIS_LX  = 0    # L-stick horizontal
AXIS_LY  = 1    # L-stick vertical
AXIS_LT  = 2    # Left Trigger
AXIS_RT  = 5    # Right Trigger
TRIGGER_DZ = 0.05   # zona-morta gatilhos

DEADZONE    = rospy.get_param('~deadzone', 0.03)
LIN_SCALE   = rospy.get_param('~lin_scale', 6.5)
ANG_SCALE   = rospy.get_param('~ang_scale', 6.5)
EXPO        = rospy.get_param('~expo', 1.5)
PUB_RATE_HZ = rospy.get_param('~pub_rate', 30)
DECEL_TIME  = rospy.get_param('~decel_time', 1.0)  # tempo sem input para começar desacelerar

class XboxTeleop:
    def __init__(self):
        self.arm_pub = rospy.Publisher('/arm_control', Float32MultiArray, queue_size=1)
        self.ee_pub  = rospy.Publisher('/end_effector_control', Float32MultiArray, queue_size=1)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.target_twist = Twist()
        self.current_twist = Twist()
        self.last_time = time.time()
        self.last_joy_time = time.time()
        self.last_arm, self.last_ee = [0.0, 0.0], [0.0, 0.0]
        self.wrist_val = self.grip_val = 0.0
        self.y_last = self.a_last = 0

        rospy.Subscriber('/joy', Joy, self.joy_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / PUB_RATE_HZ), self.publish_cmd)

    # ----------------------------- JOYSTICK ----------------------------
    def joy_cb(self, msg):
        # ----- braço / garra -------------------------------------------
        elev = self._expo(msg.axes[AXIS_LY])
        base = self._expo(msg.axes[AXIS_LX])
        if [elev, base] != self.last_arm:
            self.arm_pub.publish(Float32MultiArray(data=[elev, base]))
            self.last_arm = [elev, base]

        if msg.buttons[1]:
            self.grip_val = 80.0   # B
        elif msg.buttons[2]:
            self.grip_val = 0.0    # X
        if msg.buttons[3] and not self.y_last:
            self.wrist_val += 10.0   # Y
        if msg.buttons[0] and not self.a_last:
            self.wrist_val -= 10.0   # A
        self.y_last, self.a_last = msg.buttons[3], msg.buttons[0]

        if [self.grip_val, self.wrist_val] != self.last_ee:
            self.ee_pub.publish(Float32MultiArray(data=[self.grip_val, self.wrist_val]))
            self.last_ee = [self.grip_val, self.wrist_val]

        # ----- base holonômica com modo exclusivo de rotação ----------
        lt_norm = self._trigger_norm(msg.axes[AXIS_LT])
        rt_norm = self._trigger_norm(msg.axes[AXIS_RT])
        trigger_active = (lt_norm > TRIGGER_DZ) or (rt_norm > TRIGGER_DZ)

        if trigger_active:
            self.target_twist.linear.x = 0.0
            self.target_twist.linear.y = 0.0
            self.target_twist.angular.z = ANG_SCALE * (rt_norm - lt_norm)
        else:
            self.target_twist.linear.x = LIN_SCALE * self._expo(msg.axes[AXIS_LY])
            self.target_twist.linear.y = LIN_SCALE * self._expo(msg.axes[AXIS_LX])
            self.target_twist.angular.z = 0.0

        # Atualiza hora do último input
        self.last_joy_time = time.time()

    # ------------------------- PUBLICAÇÃO ------------------------------
    def publish_cmd(self, _):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # Se passou muito tempo sem input, desacelera target para zero
        time_since_input = now - self.last_joy_time
        if time_since_input > DECEL_TIME:
            target = Twist()  # tudo zero
        else:
            target = self.target_twist

        # Suaviza movimento
        self._smooth_towards(target, 'linear.x', 1.0, dt)
        self._smooth_towards(target, 'linear.y', 1.0, dt)
        self._smooth_towards(target, 'angular.z', 3.0, dt)

        self.cmd_pub.publish(self.current_twist)

    # --------------------------- UTIL ----------------------------------
    @staticmethod
    def _trigger_norm(v):
        return (1.0 - v) * 0.5

    @staticmethod
    def _deadzone(v):
        return 0.0 if abs(v) < DEADZONE else v

    def _expo(self, v):
        v = self._deadzone(v)
        return math.copysign(abs(v)**EXPO, v)

    def _smooth_towards(self, target, attr, rate, dt):
        """
        Move self.current_twist.<attr> em direção a target.<attr>
        """
        parts = attr.split('.')
        cur_ref = self.current_twist
        tgt_ref = target
        for p in parts[:-1]:
            cur_ref = getattr(cur_ref, p)
            tgt_ref = getattr(tgt_ref, p)
        field = parts[-1]
        cur_val = getattr(cur_ref, field)
        tgt_val = getattr(tgt_ref, field)
        step = rate * dt
        if tgt_val > cur_val:
            cur_val = min(tgt_val, cur_val + step)
        elif tgt_val < cur_val:
            cur_val = max(tgt_val, cur_val - step)
        setattr(cur_ref, field, cur_val)

if __name__ == '__main__':
    rospy.init_node('xbox_teleop_triggers_with_decel')
    XboxTeleop()
    rospy.spin()
