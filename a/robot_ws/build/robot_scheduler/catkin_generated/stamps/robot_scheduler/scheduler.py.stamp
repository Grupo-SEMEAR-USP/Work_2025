#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time, threading
import rospy
from robot_scheduler.msg import SchedulerCommand

class Scheduler:
    def __init__(self):
        # --- leitura de config ---
        cfg_ns = rospy.get_param("~config_ns", "/scheduler_cfg")
        self.cfg = rospy.get_param(cfg_ns)
        self._validate_config(self.cfg)

        g = self.cfg.get("globals", {})
        defaults = g.get("defaults", {})

        self.BURST_REPEATS = int(g.get("burst_repeats", 8))
        self.BURST_RATE_HZ  = int(g.get("burst_rate_hz", 10))

        # tempos padrões
        self.T_APPROACH     = float(defaults.get("approach_sleep", 5))
        self.T_SEARCH       = float(defaults.get("search_timeout", 10))
        self.T_ALIGN_BLOCK  = float(defaults.get("align_block_timeout", 10))
        self.T_ALIGN_GRIP   = float(defaults.get("align_gripper_timeout", 10))
        self.T_MANIP        = float(defaults.get("manip_sleep", 1))
        self.T_BACKOFF      = float(defaults.get("backoff_sleep", 1))
        self.T_NAV_TO       = float(defaults.get("nav_timeout", 50))

        # --- pubs/subs ---
        self.pub_cmd = rospy.Publisher("/scheduler/commands",
                                       SchedulerCommand, queue_size=30)
        rospy.Subscriber("/scheduler/feedback",
                         SchedulerCommand, self._fb_cb, queue_size=30)

        self.rate = rospy.Rate(self.BURST_RATE_HZ)
        self._pending = {}

        # --- plano dinâmico ---
        self.plan = self._build_plan_from_config()

    # --------------- config helpers ----------------
    @staticmethod
    def _validate_config(cfg):
        missing = []
        for k in ("tables_order", "workspaces", "blocks"):
            if k not in cfg:
                missing.append(k)
        if missing:
            raise RuntimeError("YAML inválido. Chaves faltantes: " + ", ".join(missing))

        ws = cfg["workspaces"]
        if "deposito" not in ws:
            raise RuntimeError("Defina workspaces.deposito no YAML.")

    def _build_plan_from_config(self):
        plan = []

        tables_order = self.cfg["tables_order"]
        workspaces   = self.cfg["workspaces"]
        blocks       = self.cfg["blocks"]
        max_carry    = int(self.cfg.get("max_carry_per_table", 3))
        side         = self.cfg.get("deposit_side", "right").lower()
        if side not in ("right", "left"):
            side = "right"

        def rb_front_to_deposit(n): return f"rotatory_base,front_to_deposit{n}_{side}"
        def rb_deposit_to_front(n): return f"rotatory_base,deposit{n}_to_front_{side}"

        for mesa in tables_order:
            ws_mesa = workspaces.get(mesa)
            if not ws_mesa:
                rospy.logwarn(f"[SCHEDULER] Mesa '{mesa}' sem workspace; pulando.")
                continue

            plan += [
                ("manipulation", "arm,deposit_to_top", False, self.T_MANIP),
                ("manipulation", "rotatory_base,deposit1_to_front_left", False, self.T_MANIP),

                ("manipulation", "wrist,center", False, 1),
                ("manipulation", "gripper,open", False, 1),

                # procurar bloco e alinhar

                ("manipulation", "arm,top_to_bottom15", False, self.T_MANIP),
                ("manipulation", "gripper,close", False, 1),
                ("manipulation", "arm,bottom15_to_deposit", False, self.T_MANIP),
                ("manipulation", "rotatory_base,front_to_deposit1_left", False, self.T_MANIP),

                ("manipulation", "gripper,open", False, 1),

            ]

            plan += [
                # ("manipulation", "wrist,center", False, self.T_MANIP),
                ("manipulation", "gripper,open", False, self.T_MANIP),

                ("navigation", ws_mesa, True, self.T_NAV_TO),
                ("table_aproach", "start,10", False, self.T_APPROACH),
                
                ("manipulation", "arm,deposit_to_top", False, self.T_MANIP),
                ("manipulation", "rotatory_base,front_to_deposit1_right",     False, self.T_MANIP),
            ]

            blocos_da_mesa = [b for b in blocks if b.get("table") == mesa]
            if not blocos_da_mesa:
                continue

            for i in range(0, len(blocos_da_mesa), max_carry):
                lote = blocos_da_mesa[i:i+max_carry]

                for k, b in enumerate(lote, start=1):
                    nome = b.get("name", f"block_{i+k}")

                    # --- buscar, alinhar e pegar bloco ---
                    plan += [
                        ("manipulation", "wrist,center", False, self.T_MANIP),
                        ("manipulation", "gripper,open",      False, self.T_MANIP),

                        ("align_table_border", nome, False, self.T_SEARCH),
                        ("search_block",  nome, False, self.T_SEARCH),
                        ("align_block",   nome, False, self.T_ALIGN_BLOCK),
                        ("align_gripper", nome, False, self.T_ALIGN_GRIP),

                        ("manipulation", "arm,top_to_bottom10", False, self.T_MANIP),
                        ("manipulation", "gripper,close",       False, self.T_MANIP),
                        ("manipulation", "arm,deposit_pick_to_top", False, self.T_MANIP),

                        # --- guardar no depósito interno k ---
                        ("manipulation", rb_front_to_deposit(k), False, self.T_MANIP),
                        ("manipulation", "arm,top_to_deposit_put",  False, self.T_MANIP),
                        ("manipulation", "gripper,open",            False, self.T_MANIP),
                        ("manipulation", "arm,deposit_put_to_top",  False, self.T_MANIP),
                        ("manipulation", rb_deposit_to_front(k),    False, self.T_MANIP),

                        # --- recuar e realinhar mesa ---
                        ("table_aproach", "start,10", False, self.T_APPROACH),
                        ("align_table_border", nome, False, self.T_SEARCH),
                    ]

                # terminou o lote na mesa -> volta à postura neutra de frente
                plan += [
                    ("move_time", "tras,2", False, self.T_BACKOFF),
                    ("manipulation", "wrist,center", False, self.T_MANIP),
                    ("manipulation", "arm,top",      False, self.T_MANIP),
                ]

            # (se houver navegação para próxima mesa, o 'tuck' será aplicado no próximo loop)

        return plan


    # --------------- messaging helpers ----------------
    @staticmethod
    def new_uid():
        return int(time.time() * 1e6)

    def _send_burst(self, target, payload, need_ack):
        uid = self.new_uid()
        cmd = SchedulerCommand(uid=uid, target=target,
                               payload=payload, need_ack=need_ack)

        for _ in range(self.BURST_REPEATS):
            self.pub_cmd.publish(cmd)
            self.rate.sleep()

        rospy.loginfo(f"[SCHEDULER] → {target} payload='{payload}' uid={uid} ack={need_ack}")
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
        rospy.loginfo("[SCHEDULER] Iniciando execução do plano gerado pelo YAML.")
        for target, payload, expect_ack, t in self.plan:
            uid = self._send_burst(target, payload, expect_ack)
            if expect_ack:
                if not self.wait_ack(uid, t):
                    rospy.logwarn(f"[SCHEDULER] ACK timeout: target={target} uid={uid}")
            else:
                rospy.sleep(t)

        rospy.loginfo("[SCHEDULER] Plano concluído com sucesso.")

def main():
    rospy.init_node("scheduler")
    Scheduler().run()

if __name__ == "__main__":
    main()
