#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time, threading, collections
import rospy
from robot_scheduler.msg import SchedulerCommand, SchedulerResponse

BURST_REPEATS = 8
BURST_RATE_HZ = 10

class Scheduler:
    def __init__(self):
        self.pub_cmd = rospy.Publisher("/scheduler/commands",
                                       SchedulerCommand, queue_size=30)
        rospy.Subscriber("/scheduler/feedback",
                         SchedulerResponse, self._fb_cb, queue_size=30)

        self.rate = rospy.Rate(BURST_RATE_HZ)
        self._pending = {} 
        self.index = 0 

        self.plan = [
            # target,            payload,       expect_ack, timeout_or_delay

            # ("move", "frente", False, 3),
            # ("move", "parar", False, 1),
            # ("move", "esquerda_90", False, 2.81),
            # ("move", "parar", False, 1),
            # ("move", "frente", False, 7),
            # ("move", "parar", False, 1),
            # ("move", "esquerda_90", False, 2.81),
            # ("move", "parar", False, 1),
            # ("move", "frente", False, 3.2),
            # ("move", "parar", False, 1),
            ("move", "direita_90", False, 3.0),

            ("manipulation",     "arm,deposit_to_top",   False,  10.0),
            ("manipulation",     "rotatory_base,deposit2_to_left",   False,  7.0),

            # Procurar blocos
            # ("align_table", "10", True, 15),
            ("search_block",      "1,direita", True, 30),
            ("move", "parar", False, 1),
            ("align_table", "10", True, 15),
            ("move", "parar", False, 1),
            ("search_block",      "2,direita", True, 60),
            ("move", "parar", False, 1),
            ("align_table", "10", True, 15),
            ("move", "parar", False, 1),
            ("search_block",      "7,direita", True, 60),
            ("move", "parar", False, 1),
            ("align_table", "10", True, 15),
            ("move", "parar", False, 1),
            ("search_block",      "4,direita", True, 60),
            ("move", "parar", False, 1)

            
            # # AMT plano principal: Idenficar todos os blocos da esquerda para a direita
            # #   A partir do último (direita), voltar e manipular os últimos 3 blocos

            # # Mover até a WS
            # ("manipulation",     "gripper,open",   False,  2),
            # ("move", "frente",False,7),
            # ("move", "esquerda_90",False,7),
            # ("move", "parar",False,7),
            # ("move", "frente",False,7),
            # ("move", "esquerda_90",False,7),
            # ("move", "parar",False,7),
            # ("move", "frente",False,7),
            # ("move", "parar",False,7),
            
            # # Estamos na borda esquerda da mesa:
	        # # Objetivo 1-) Procurar bloco e alinhar (Procurar movendo para a direita):

            # # Colocar a câmera na frente
            # ("manipulation",     "arm,deposit_to_top",   False,  10),
            # ("manipulation",     "rotatory_base,deposit2_to_front_left",   False,  7),
            
            # # Procurar bloco 01
            # ("align_table", "10", True, 15),
            # ("search_block",      "1,direita", True, 30),
            # ("move", "parar", False, 1),

            # # Bloco vazio. Implementar move_time (caso coloque move_time, dar um align_table antes) ou procurar duas vezes pelo bloco 2

            # # Procurar bloco 02
            # ("align_table", "10", True, 15),
            # ("search_block",      "1,direita", True, 30),
            # ("move", "parar", False, 1),

            # # Procurar bloco 03
            # ("align_table", "10", True, 15),
            # ("search_block",      "1,direita", True, 30),
            # ("move", "parar", False, 1),

            # # Procurar bloco 04
            # ("align_table", "10", True, 15),
            # ("search_block",      "1,direita", True, 30),
            # ("move", "parar", False, 1),

            # # Procurar bloco 05
            # ("align_table", "10", True, 15),
            # ("search_block",      "1,direita", True, 30),
            # ("move", "parar", False, 1),

            # # Procurar bloco 06
            # ("align_table", "10", True, 15),
            # ("search_block",      "1,direita", True, 30),
            # ("move", "parar", False, 1),

            # # Estamos na borda direita da mesa (especificamente em cima do bloco 06)
            # # Objetivo 2-) Manipular os blocos (Procurar movendo para a esquerda) 

            # # Manipular bloco 06
            # ("align_table", "10", True, 15),
            # ("align_block",       "1", False, 10),
            #  # Bloco 06 alinhado à câmera/garra - iniciar a manipulação:
            # ("manipulation",     "arm,top_to_bottom10",   False,  10),			
            # ("manipulation",     "gripper,close",   False,  3),
            # ("manipulation",     "arm,bottom10_to_top",   False,  10),			
            # ("manipulation",     "rotatory_base,front_to_deposit1_left",   False,  7),
            # ("manipulation",     "arm,top_to_deposit",   False,  10),
            # ("manipulation",     "gripper,open",   False,  3),
            # ("manipulation",     "arm,deposit_to_top",   False,  3),
            # ("manipulation",     "rotatory_base,deposit1_to_front_left",   False,  3),

            # # Manipular bloco 05
            # ("search_block",      "1,esquerda", True, 30),
            # ("move", "parar", False, 1),
            # ("align_table", "10", True, 15),
            # ("align_block",       "<nome_item>", False, 10),
            #  # Bloco 05 alinhado à câmera/garra - iniciar a manipulação:
            # ("manipulation",     "arm,top_to_bottom10",   False,  10),			
            # ("manipulation",     "gripper,close",   False,  3),
            # ("manipulation",     "arm,bottom10_to_top",   False,  10),			
            # ("manipulation",     "rotatory_base,front_to_deposit2_left",   False,  7),
            # ("manipulation",     "arm,top_to_deposit",   False,  10),
            # ("manipulation",     "gripper,open",   False,  3),
            # ("manipulation",     "arm,deposit_to_top",   False,  3),
            # ("manipulation",     "rotatory_base,deposit2_to_front_left",   False,  3),

            # # Manipular bloco 04
            # ("search_block",      "1,esquerda", True, 30),
            # ("move", "parar", False, 1),
            # ("align_table", "10", True, 15),
            # ("align_block",       "<nome_item>", False, 10),
            #  # Bloco 04 alinhado à câmera/garra - iniciar a manipulação:
            # ("manipulation",     "arm,top_to_bottom10",   False,  10),			
            # ("manipulation",     "gripper,close",   False,  3),
            # ("manipulation",     "arm,bottom10_to_top",   False,  10),			
            # ("manipulation",     "rotatory_base,front_to_deposit3_left",   False,  7),
            # ("manipulation",     "arm,top_to_deposit",   False,  10),
            # ("manipulation",     "gripper,open",   False,  3),
            # ("manipulation",     "arm,deposit_to_top",   False,  3),
            # ("manipulation",     "rotatory_base,deposit3_to_front_left",   False,  3),



        



            




            # Talvez daqui pra baixo é inutil    


            # # Teste da BMT usado:
            # ("search_block",      "2,direita", True, 60),
            # ("move", "parar", False, 1),
            # ("align_table", "10", True, 15),
            # ("search_block",      "4,direita", True, 60),
            # ("move", "parar", False, 1),
            # ("align_table", "10", True, 15),
            # ("search_block",      "7,direita", True, 60),
            # ("move", "parar", False, 1)

            
            # Rascunho insano daqui pra frente

            # ("align_block",       "1", False, 10),
            				
	        #  # Bloco alinhado à câmera/garra: iniciar a manipulação:
            # ("manipulation",     "arm,top_to_bottom10",   False,  10),			
            # ("manipulation",     "gripper,close",   False,  3),
            # ("manipulation",     "arm,bottom10_to_top",   False,  10),			
            # ("manipulation",     "rotatory_base,front_to_deposit1_left",   False,  7),
            # ("manipulation",     "arm,top_to_deposit",   False,  10),
            # ("manipulation",     "gripper,open",   False,  3),
            # ("align_table", "10", False, 5)



            # ("navigation",       "WS1",      True,  30),
            # ("navigation",       "A",      True,  30),
            # ("navigation",       "B",      True,  30),
            # ("navigation",       "C",      True,  30),
            # ("navigation",       "D",      True,  30),
            # ("navigation",       "E",      True,  30),
            # ("navigation",       "WS8",      True,  30),
            # ("navigation",       "F",      True,  30),
            # ("navigation",       "FINISH",      True,  30),
            # ("table_aproach",    "start,10",        False, 5),
            # ("align_block",     "1",   True,  30),

            # # --- loop de pegar itens ----

            # ("manipulation",     "arm,deposit1_to_top",   False,  1),
            # ("manipulation",     "rotatory_base,deposit1_to_front_right",   False,  1),
            
            # ("search_item",      "<nome_item>", False, 10),
            # ("align_item",       "<nome_item>", False, 10),
            # ("align_gripper",    "<nome_item>", False, 10),

            # ("manipulation",     "arm,bottom10",   False,  1),
            # ("manipulation",     "gripper,close",   False,  1),
            # ("manipulation",     "arm,bottom10_to_deposit",   False,  1),
            # ("manipulation",     "rotatory_base,front_to_deposit1_right",   False,  1),

            # # ----- após iterações -----
            # ("move_time",     "tras,2",   False,  1), # da uam ré
            # ("navigation",       "<nome_workspace>",      True,  50),

            # # --- rotina de deposito ----

            # # ----- após iterações -----
            # ("move_time",     "tras,2",   False,  1), # da uam ré
            # ("navigation",       "<nome_workspace>",      True,  50),

        ]

    @staticmethod
    def new_uid():
        return int(time.time() * 1e6)

    def _send_burst(self, target, payload, need_ack,t):
        uid = self.new_uid()
        cmd = SchedulerCommand(uid=uid, target=target,
                      payload=payload, need_ack=need_ack, timeout=t)

        for _ in range(BURST_REPEATS):
            self.pub_cmd.publish(cmd)
            self.rate.sleep()

        rospy.loginfo(f"[SCHEDULER] → {target} uid={uid} ack={need_ack}")
        return uid
    
    def _fb_cb(self, fb: SchedulerResponse):
        ev = self._pending.get(fb.uid)
        if ev:
            ev.set()

        """
        Recebe o feedback (uid, target, status) e verifica a condição para liberar a espera.
        """
        
        # 1. Tenta obter o Evento correspondente ao UID
        ev = self._pending.get(fb.uid)
        
        if ev:
            # 2. Faz a verificação: o status precisa ser 'OK'
            if fb.status.upper() == "OK":
                rospy.loginfo(f"[SCHEDULER] {fb.target} uid={fb.uid} OK. Liberando a espera.")
                # 3. Sinaliza o Evento, liberando a função wait_ack()
                ev.set()
            elif fb.status.upper() == "SKIP":  
                rospy.loginfo(f"[SCHEDULER] {fb.target} uid={fb.uid} SKIP. Pulando a próxima tarefa.")
                self.index += 1
            else:
                # Opcional: registrar falha se o status não for OK
                rospy.logwarn(f"[SCHEDULER] {fb.target} uid={fb.uid} FALHA. Status: {fb.status}")


    def wait_ack(self, uid, timeout):
        ev = threading.Event()
        self._pending[uid] = ev
        ok = ev.wait(timeout)
        self._pending.pop(uid, None)
        return ok

    def run(self):
        self.index = 0
        while True:
            if self.index == len(self.plan):
                break

            task = self.plan[self.index]
            target, payload, expect_ack, t = task
            uid = self._send_burst(target, payload, expect_ack,t)
            if expect_ack:
                if not self.wait_ack(uid, t):
                    rospy.logwarn(f"ACK timeout: {target} (uid={uid})")
            else:
                rospy.sleep(t)

            self.index += 1

        rospy.loginfo("Scheduler concluiu o plano completo.")

def main():
    rospy.init_node("scheduler")
    Scheduler().run()

if __name__ == "__main__":
    main()

'''
Pontos importantes:
    O atributo plan possui todas as etapas (comandos) de uma rodada. 
    A função run() é responsável por gerenciar o fluxo dos comandos: envia-os e espera resposta, quando necessário.
    Cada comando pode ou não requerer uma resposta de feedback (tarefa realizada) e isso é definido pelo expect_ack. 
    Caso seja True, o programa fica esperando até receber o retorno ou dar timeout antes de prosseguir com os próximos comandos. 
    Essa verificação é feita monitoração de um Evento (linha 77): o feedback é recebido pelo tópico scheduler/feedback 
    e então a função de callback é acionada, setando o evento (linha 72) e finalizando a espera (na função wait_ack).

'''