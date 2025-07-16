#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time, threading, collections
import rospy
from robot_scheduler.msg import SchedulerCommand, SchedulerCommand

BURST_REPEATS = 8
BURST_RATE_HZ = 10

class Scheduler:
    def __init__(self):
        self.pub_cmd = rospy.Publisher("/scheduler/commands",
                                       SchedulerCommand, queue_size=30)
        rospy.Subscriber("/scheduler/feedback",
                         SchedulerCommand, self._fb_cb, queue_size=30)

        self.rate = rospy.Rate(BURST_RATE_HZ)
        self._pending = {} 

        self.plan = [
            # target, payload, expect_ack, timeout_or_delay
            ("manipulation",     "gripper,open",   False,  1),

            ("navigation",       "<nome_workspace>",      True,  50),
            ("table_aproach",    "start,10",        False, 5),

            # --- loop de pegar itens ----

            ("manipulation",     "arm,deposit1_to_top",   False,  1),
            ("manipulation",     "rotatory_base,deposit1_to_front_right",   False,  1),
            
            ("search_item",      "<nome_item>", False, 10),
            ("align_item",       "<nome_item>", False, 10),
            ("align_gripper",    "<nome_item>", False, 10),

            ("manipulation",     "arm,bottom10",   False,  1),
            ("manipulation",     "gripper,close",   False,  1),
            ("manipulation",     "arm,bottom10_to_deposit",   False,  1),
            ("manipulation",     "rotatory_base,front_to_deposit1_right",   False,  1),

            # ----- após iterações -----
            ("move_time",     "tras,2",   False,  1), # da uam ré
            ("navigation",       "<nome_workspace>",      True,  50),

            # --- rotina de deposito ----

            # ----- após iterações -----
            ("move_time",     "tras,2",   False,  1), # da uam ré
            ("navigation",       "<nome_workspace>",      True,  50),

        ]

    @staticmethod
    def new_uid():
        return int(time.time() * 1e6)

    def _send_burst(self, target, payload, need_ack):
        uid = self.new_uid()
        cmd = SchedulerCommand(uid=uid, target=target,
                      payload=payload, need_ack=need_ack)

        for _ in range(BURST_REPEATS):
            self.pub_cmd.publish(cmd)
            self.rate.sleep()

        rospy.loginfo(f"[SCHEDULER] → {target} uid={uid} ack={need_ack}")
        return uid
    
    def _fb_cb(self, fb):
        ev = self._pending.get(fb.uid)
        if ev:
            ev.set()

    def wait_ack(self, uid, timeout):
        ev = threading.Event()
        self._pending[uid] = ev
        ok = ev.wait(timeout)
        self._pending.pop(uid, None)
        return ok

    def run(self):
        for target, payload, expect_ack, t in self.plan:
            uid = self._send_burst(target, payload, expect_ack)
            if expect_ack:
                if not self.wait_ack(uid, t):
                    rospy.logwarn(f"ACK timeout: {target} (uid={uid})")
            else:
                rospy.sleep(t)

        rospy.loginfo("Scheduler concluiu o plano completo.")

def main():
    rospy.init_node("scheduler")
    Scheduler().run()

if __name__ == "__main__":
    main()
