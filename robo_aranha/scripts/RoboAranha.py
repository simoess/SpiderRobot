#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import pprint
import time
from servo_master.msg import ServoMessage
from robo_aranha.msg import AranhaCommand
from robo_aranha.msg import Command
from threading import  Lock

DEBUG = False #quando True os commandos não são enviados para o ServoMaster

def createDicionaryJoints():
    data = dict()
    data['coxa4'] = dict()
    data['coxa4']['FRENTE'] = 30
    data['coxa4']['MEIO'] =  70
    data['coxa4']['TRAS'] =  100

    data['femur4'] = dict()
    data['femur4']['SUBIR'] =  170
    data['femur4']['MEIO'] =  90
    data['femur4']['DESCER'] =  90

    data['tibia4'] = dict()
    data['tibia4']['ESQUERDA'] =  0
    data['tibia4']['MEIO'] =  90
    data['tibia4']['DIREITA'] =  180

    data['coxa5'] = dict()
    data['coxa5']['FRENTE'] =  60
    data['coxa5']['MEIO'] =  90
    data['coxa5']['TRAS'] =  120

    data['femur5'] = dict()
    data['femur5']['SUBIR'] =  170
    data['femur5']['MEIO'] =  90
    data['femur5']['DESCER'] =  90

    data['tibia5'] = dict()
    data['tibia5']['ESQUERDA'] =  0
    data['tibia5']['MEIO'] =  90
    data['tibia5']['DIREITA'] =  180

    data['coxa6'] = dict()
    data['coxa6']['FRENTE'] =  60
    data['coxa6']['MEIO'] =  110
    data['coxa6']['TRAS'] =  150

    data['femur6'] = dict()
    data['femur6']['SUBIR'] =  170
    data['femur6']['MEIO'] =  90
    data['femur6']['DESCER'] =  85

    data['tibia6'] = dict()
    data['tibia6']['ESQUERDA'] =  10
    data['tibia6']['MEIO'] =  90
    data['tibia6']['DIREITA'] =  180


    data['coxa1'] = dict()
    data['coxa1']['FRENTE'] =  150
    data['coxa1']['MEIO'] =  110
    data['coxa1']['TRAS'] =  80

    data['femur1'] = dict()
    data['femur1']['SUBIR'] =  170
    data['femur1']['MEIO'] =  90
    data['femur1']['DESCER'] =  90

    data['tibia1'] = dict()
    data['tibia1']['ESQUERDA'] =  0
    data['tibia1']['MEIO'] =  90
    data['tibia1']['DIREITA'] =  180

    data['coxa2'] = dict()
    data['coxa2']['FRENTE'] =  120
    data['coxa2']['MEIO'] =  80
    data['coxa2']['TRAS'] =  60

    data['femur2'] = dict()
    data['femur2']['SUBIR'] =  170
    data['femur2']['MEIO'] =  90
    data['femur2']['DESCER'] =  90

    data['tibia2'] = dict()
    data['tibia2']['ESQUERDA'] =  0
    data['tibia2']['MEIO'] =  90
    data['tibia2']['DIREITA'] =  180

    data['coxa3'] = dict()
    data['coxa3']['FRENTE'] =  120
    data['coxa3']['MEIO'] =  75
    data['coxa3']['TRAS'] =  30

    data['femur3'] = dict()
    data['femur3']['SUBIR'] =  170
    data['femur3']['MEIO'] =  90
    data['femur3']['DESCER'] =  90

    data['tibia3'] = dict()
    data['tibia3']['ESQUERDA'] =  0
    data['tibia3']['MEIO'] =  90
    data['tibia3']['DIREITA'] =  180

    return data


joint_dict = createDicionaryJoints()


class Action:
    #'representa uma acao'
    def __init__(self, segment, angle ):
        self.segment = segment
        self.position = angle

class Segment:
    #'representa um segmento de perna'
    def __init__(self, jointName, alias, discreteAngle):
        self.jointName = jointName
        self.alias = alias
        self.position = discreteAngle

class Leg:
    #'Representa uma perna'
    def __init__(self, segments, alias):
        self.segments = segments
        self.alias = alias

class Robot:
    def __init__(self, segmentsDicionary):
        self.segmentsDicionary = segmentsDicionary
        self.publisher = rospy.Publisher('/ServoMaster', ServoMessage, queue_size=10)
        self.lock_queue = Lock()
        self.command_queue = []
        self.action_queue = []
        self.lastMovment = None

    def setLastMovment(movment):
        self.lastMovment = movment

    '''
       Ativa um motor por vez, a nao ser que seja feita uma acao composta
    '''
    def runRoboTicks(self):
        rospy.loginfo("RobotTick")

        if(self.action_queue == [] or self.action_queue == None):
            if(self.command_queue != [] and self.command_queue != None):
                self.lock_queue.acquire()
                try:
                    self.action_queue = self.command_queue[0](self.lastMovment)

                    if self.command_queue[0] == goFront:
                        self.lastMovment = Command.FRENTE
                    elif self.command_queue[0] == goBack:
                        self.lastMovment = Command.TRAS
                    else:
                        self.lastMovment = None

                    del self.command_queue[0]
                finally:
                    self.lock_queue.release()
            else:
                return


        action = self.action_queue[0]
        del self.action_queue[0]

        if(action.segment ==  'JUNTO' and action.position == 'INICIO'):
            while(True):
                action = self.action_queue[0]
                del self.action_queue[0]
                #TODO criar uma função para reaproveitar o código abaixo
                if(action.position == 'FIM'):
                    break
                segment = action.segment
                angle = self.segmentsDicionary[action.segment][action.position]


                servoMessage = ServoMessage()
                servoMessage.segment = segment
                servoMessage.angle = angle
                rospy.loginfo("sending message to /ServoMaster")

                if( not DEBUG):
                    self.publisher.publish(servoMessage)
        elif (action.segment == "STOP"):
            return
        elif (action.segment == "SPECIAL"):
            femur_angle = 90 - self.segmentsDicionary['femur1']['MEIO']
            rospy.loginfo("femur_angle: " + str(femur_angle))
            if(femur_angle < 0):
                rospy.loginfo("Os angulos do femur devem estar todos em uma posicao entre 0 e 90 graus para alterar a altura")
            elif(action.position == "UP"):
                if(femur_angle == 90):
                    rospy.loginfo("UP ja esta no maximo")
                else:
                    femur_angle += 10
                    for i in range(1, 7):
                        femur_meio   = joint_dict["femur1"]["MEIO"] - femur_angle
                        femur_descer = femur_meio
                       # tibia_esquerda = joint_dict["tibia1"]["ESQUERDA"] + 10
                        tibia_meio = joint_dict["tibia1"]["MEIO"] + femur_angle
                       # tibia_direita = joint_dict["tibia1"]["DIREITA"] + 10

                        self.segmentsDicionary["femur"+str(i)]["MEIO"] = femur_meio
                        self.segmentsDicionary["femur"+str(i)]["DESCER"] = femur_descer
                      #  self.segmentsDicionary["tibia"+str(i)]["ESQUERDA"] = tibia_esquerda
                        self.segmentsDicionary["tibia"+str(i)]["MEIO"] = tibia_meio
                      #  self.segmentsDicionary["tibia"+str(i)]["DIREITA"] = tibia_direita

            elif(action.position == "DOWN"):
                if(femur_angle == 0):
                    rospy.loginfo("DOWN ja esta no maximo")
                else:
                    femur_angle -= 10
                    for i in range(1, 7):
                        femur_meio   = joint_dict["femur1"]["MEIO"] + femur_angle
                        femur_descer = femur_meio

                       # tibia_esquerda = joint_dict["tibia1"]["ESQUERDA"] + 10
                        tibia_meio = joint_dict["tibia1"]["MEIO"] - femur_angle
                        # tibia_direita = joint_dict["tibia1"]["DIREITA"] + 10

                        self.segmentsDicionary["femur"+str(i)]["MEIO"] = tibia_meio
                        self.segmentsDicionary["femur"+str(i)]["DESCER"] = tibia_meio
                      #  self.segmentsDicionary["tibia"+str(i)]["ESQUERDA"] = tibia_esquerda
                        self.segmentsDicionary["tibia"+str(i)]["MEIO"] = femur_meio
                      #  self.segmentsDicionary["tibia"+str(i)]["DIREITA"] = tibia_direita
            rospy.loginfo("Novo Angulo FEMUR: " + str(self.segmentsDicionary["femur1"]["MEIO"]))
            rospy.loginfo("Novo Angulo TIBIA: " + str(self.segmentsDicionary["tibia1"]["MEIO"]))

        else:
            segment = action.segment
            angle = self.segmentsDicionary[action.segment][action.position]

            servoMessage = ServoMessage()
            servoMessage.segment = segment
            servoMessage.angle = angle
            rospy.loginfo("sending message to /ServoMaster")

            if( not DEBUG):
                self.publisher.publish(servoMessage)




    def turnOff(self):
        rospy.loginfo("Desligando o Robo :(")
        if(not DEBUG):
            servoMessage = ServoMessage()
            servoMessage.segment = "DESLIGAR"
            self.publisher.publish(servoMessage)

    # o command deve ser uma funcao
    def addCommandInQueue(self, command, num):
        rospy.loginfo("Comando adicionado na fila de execucao do Robo")
        self.lock_queue.acquire()
        try:
            for i in range(num):
                self.command_queue.append(command)
        finally:
            self.lock_queue.release()

#class Movement:
#TODO chamar o método setLastMovment do Robo ************************************************





def goFront(initial):
    actions = list()
    if(initial != Command.FRENTE):
        actions.append(Action('femur3', 'SUBIR'))
        actions.append(Action('coxa3', 'TRAS'))
        actions.append(Action('femur3', 'DESCER'))
        actions.append(Action('femur6', 'SUBIR'))
        actions.append(Action('coxa6', 'TRAS'))
        actions.append(Action('femur6', 'DESCER'))
    actions.append(Action('femur6', 'SUBIR'))
    actions.append(Action('coxa6', 'MEIO'))
    actions.append(Action('femur2', 'SUBIR'))
    actions.append(Action('femur6', 'DESCER'))
    actions.append(Action('coxa2', 'FRENTE'))
    actions.append(Action('femur4', 'SUBIR'))
    actions.append(Action('femur2', 'DESCER'))
    actions.append(Action('coxa4', 'FRENTE'))
    actions.append(Action('femur3', 'SUBIR'))
    actions.append(Action('femur4', 'DESCER'))
    actions.append(Action('coxa3', 'MEIO'))
    actions.append(Action('femur5', 'SUBIR'))
    actions.append(Action('femur3', 'DESCER'))
    actions.append(Action('coxa5', 'FRENTE'))
    actions.append(Action('femur1', 'SUBIR'))
    actions.append(Action('femur5', 'DESCER'))
    actions.append(Action('coxa1', 'FRENTE'))
    actions.append(Action('femur1', 'DESCER'))

    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('coxa6', 'TRAS'))
    actions.append(Action('coxa5', 'MEIO'))
    actions.append(Action('coxa4', 'MEIO'))
    actions.append(Action('coxa2', 'MEIO'))
    actions.append(Action('coxa3', 'TRAS'))
    actions.append(Action('coxa1', 'MEIO'))
    actions.append(Action('JUNTO', 'FIM'))
    return actions





def goBack(initial):
    actions = list()
    if(initial != Command.TRAS):
        actions.append(Action('femur4', 'SUBIR'))
        actions.append(Action('coxa4', 'FRENTE'))
        actions.append(Action('femur4', 'DESCER'))
        actions.append(Action('femur1', 'SUBIR'))
        actions.append(Action('coxa1', 'FRENTE'))
        actions.append(Action('femur1', 'DESCER'))
    actions.append(Action('femur6', 'SUBIR'))
    actions.append(Action('coxa6', 'TRAS'))
    actions.append(Action('femur2', 'SUBIR'))
    actions.append(Action('femur6', 'DESCER'))
    actions.append(Action('coxa2', 'TRAS'))
    actions.append(Action('femur4', 'SUBIR'))
    actions.append(Action('femur2', 'DESCER'))
    actions.append(Action('coxa4', 'MEIO'))
    actions.append(Action('femur3', 'SUBIR'))
    actions.append(Action('femur4', 'DESCER'))
    actions.append(Action('coxa3', 'TRAS'))
    actions.append(Action('femur5', 'SUBIR'))
    actions.append(Action('femur3', 'DESCER'))
    actions.append(Action('coxa5', 'TRAS'))
    actions.append(Action('femur1', 'SUBIR'))
    actions.append(Action('femur5', 'DESCER'))
    actions.append(Action('coxa1', 'MEIO'))
    actions.append(Action('femur1', 'DESCER'))
    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('coxa6', 'MEIO'))
    actions.append(Action('coxa5', 'MEIO'))
    actions.append(Action('coxa4', 'FRENTE'))
    actions.append(Action('coxa2', 'MEIO'))
    actions.append(Action('coxa3', 'MEIO'))
    actions.append(Action('coxa1', 'FRENTE'))
    actions.append(Action('JUNTO', 'FIM'))
    return actions

#======================================================================
#======================================================================
#======================================================================
#======================================================================
#======================================================================
#======================================================================
#======================================================================
def goFrontnewtest(initial):
    actions = list()

    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('femur6', 'SUBIR'))
    actions.append(Action('femur2', 'SUBIR'))
    actions.append(Action('femur4', 'SUBIR'))
    actions.append(Action('coxa6', 'FRENTE'))
    actions.append(Action('coxa2', 'FRENTE'))
    actions.append(Action('coxa4', 'FRENTE'))
    actions.append(Action('JUNTO', 'FIM'))

    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('femur6', 'DESCER'))
    actions.append(Action('femur4', 'DESCER'))
    actions.append(Action('femur2', 'DESCER'))
    actions.append(Action('JUNTO', 'FIM'))

    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('coxa6', 'MEIO'))
    actions.append(Action('coxa2', 'MEIO'))
    actions.append(Action('coxa4', 'MEIO'))
    actions.append(Action('JUNTO', 'FIM'))

    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('femur1', 'SUBIR'))
    actions.append(Action('femur5', 'SUBIR'))
    actions.append(Action('femur3', 'SUBIR'))
    actions.append(Action('coxa1', 'FRENTE'))
    actions.append(Action('coxa5', 'FRENTE'))
    actions.append(Action('coxa3', 'FRENTE'))
    actions.append(Action('JUNTO', 'FIM'))

    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('femur1', 'DESCER'))
    actions.append(Action('femur5', 'DESCER'))
    actions.append(Action('femur3', 'DESCER'))
    actions.append(Action('JUNTO', 'FIM'))

    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('coxa1', 'MEIO'))
    actions.append(Action('coxa5', 'MEIO'))
    actions.append(Action('coxa3', 'MEIO'))
    actions.append(Action('JUNTO', 'FIM'))

    return actions


#======================================================================
#======================================================================
#======================================================================
#======================================================================
#======================================================================
#======================================================================
#======================================================================

def bestMove():
    actions = list()
    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('tibia1', 'DIREITA'))
    actions.append(Action('tibia2', 'DIREITA'))
    actions.append(Action('tibia3', 'DIREITA'))
    actions.append(Action('tibia4', 'ESQUERDA'))
    actions.append(Action('tibia5', 'ESQUERDA'))
    actions.append(Action('tibia6', 'ESQUERDA'))
    actions.append(Action('JUNTO', 'FIM'))


    actions.append(Action('femur1', 'DESCER'))
    actions.append(Action('tibia1', 'MEIO'))
    actions.append(Action('femur2', 'DESCER'))
    actions.append(Action('tibia2', 'MEIO'))
    actions.append(Action('femur3', 'DESCER'))
    actions.append(Action('tibia3', 'MEIO'))
    actions.append(Action('femur4', 'DESCER'))
    actions.append(Action('tibia4', 'MEIO'))
    actions.append(Action('femur5', 'DESCER'))
    actions.append(Action('tibia5', 'MEIO'))
    actions.append(Action('femur6', 'DESCER'))
    actions.append(Action('tibia6', 'MEIO'))
    return actions

def goRight(initial):
    actions = list()

    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('femur1', 'SUBIR'))
    actions.append(Action('femur3', 'SUBIR'))
    actions.append(Action('femur5', 'SUBIR'))
    actions.append(Action('JUNTO', 'FIM'))
    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('tibia1', 'ESQUERDA'))
    actions.append(Action('tibia5', 'ESQUERDA'))
    actions.append(Action('tibia3', 'ESQUERDA'))
    actions.append(Action('JUNTO', 'FIM'))
    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('femur1', 'DESCER'))
    actions.append(Action('femur3', 'DESCER'))
    actions.append(Action('femur5', 'DESCER'))
    actions.append(Action('JUNTO', 'FIM'))
    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('tibia1', 'MEIO'))
    actions.append(Action('tibia3', 'MEIO'))
    actions.append(Action('tibia5', 'MEIO'))
    actions.append(Action('JUNTO', 'FIM'))

    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('femur4', 'SUBIR'))
    actions.append(Action('femur2', 'SUBIR'))
    actions.append(Action('femur6', 'SUBIR'))
    actions.append(Action('JUNTO', 'FIM'))
    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('tibia4', 'ESQUERDA'))
    actions.append(Action('tibia2', 'ESQUERDA'))
    actions.append(Action('tibia6', 'ESQUERDA'))
    actions.append(Action('JUNTO', 'FIM'))
    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('femur4', 'DESCER'))
    actions.append(Action('femur2', 'DESCER'))
    actions.append(Action('femur6', 'DESCER'))
    actions.append(Action('JUNTO', 'FIM'))
    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('tibia4', 'MEIO'))
    actions.append(Action('tibia2', 'MEIO'))
    actions.append(Action('tibia6', 'MEIO'))
    actions.append(Action('JUNTO', 'FIM'))

    return actions







def goLeft(initial):
    actions = list()

    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('femur1', 'SUBIR'))
    actions.append(Action('femur3', 'SUBIR'))
    actions.append(Action('femur5', 'SUBIR'))
    actions.append(Action('JUNTO', 'FIM'))
    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('tibia1', 'DIREITA'))
    actions.append(Action('tibia5', 'DIREITA'))
    actions.append(Action('tibia3', 'DIREITA'))
    actions.append(Action('JUNTO', 'FIM'))
    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('femur1', 'DESCER'))
    actions.append(Action('femur3', 'DESCER'))
    actions.append(Action('femur5', 'DESCER'))
    actions.append(Action('JUNTO', 'FIM'))
    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('tibia1', 'MEIO'))
    actions.append(Action('tibia3', 'MEIO'))
    actions.append(Action('tibia5', 'MEIO'))
    actions.append(Action('JUNTO', 'FIM'))

    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('femur4', 'SUBIR'))
    actions.append(Action('femur2', 'SUBIR'))
    actions.append(Action('femur6', 'SUBIR'))
    actions.append(Action('JUNTO', 'FIM'))
    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('tibia4', 'DIREITA'))
    actions.append(Action('tibia2', 'DIREITA'))
    actions.append(Action('tibia6', 'DIREITA'))
    actions.append(Action('JUNTO', 'FIM'))
    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('femur4', 'DESCER'))
    actions.append(Action('femur2', 'DESCER'))
    actions.append(Action('femur6', 'DESCER'))
    actions.append(Action('JUNTO', 'FIM'))
    actions.append(Action('JUNTO', 'INICIO'))
    actions.append(Action('tibia4', 'MEIO'))
    actions.append(Action('tibia2', 'MEIO'))
    actions.append(Action('tibia6', 'MEIO'))
    actions.append(Action('JUNTO', 'FIM'))

    return actions




def turnRight(initial):

  actions = list()
  actions.append(Action('femur1', 'SUBIR'))
  actions.append(Action('coxa1', 'TRAS'))
  actions.append(Action('femur5', 'SUBIR'))
  actions.append(Action('femur1', 'DESCER'))
  actions.append(Action('coxa5', 'FRENTE'))
  actions.append(Action('femur3', 'SUBIR'))
  actions.append(Action('femur5', 'DESCER'))
  actions.append(Action('coxa3', 'TRAS'))
  actions.append(Action('femur4', 'SUBIR'))
  actions.append(Action('femur3', 'DESCER'))
  actions.append(Action('coxa4', 'FRENTE'))
  actions.append(Action('femur2', 'SUBIR'))
  actions.append(Action('femur4', 'DESCER'))
  actions.append(Action('coxa2', 'TRAS'))
  actions.append(Action('femur6', 'SUBIR'))
  actions.append(Action('femur2', 'DESCER'))
  actions.append(Action('coxa6', 'FRENTE'))
  actions.append(Action('femur6', 'DESCER'))

  actions.append(Action('JUNTO', 'INICIO'))
  actions.append(Action('coxa1', 'MEIO'))
  actions.append(Action('coxa2', 'MEIO'))
  actions.append(Action('coxa3', 'MEIO'))
  actions.append(Action('coxa4', 'MEIO'))
  actions.append(Action('coxa5', 'MEIO'))
  actions.append(Action('coxa6', 'MEIO'))
  actions.append(Action('JUNTO', 'FIM'))

  return actions

def turnLeft(initial):
  actions = list()
  actions.append(Action('femur1', 'SUBIR'))
  actions.append(Action('coxa1', 'FRENTE'))
  actions.append(Action('femur5', 'SUBIR'))
  actions.append(Action('femur1', 'DESCER'))
  actions.append(Action('coxa5', 'TRAS'))
  actions.append(Action('femur3', 'SUBIR'))
  actions.append(Action('femur5', 'DESCER'))
  actions.append(Action('coxa3', 'FRENTE'))
  actions.append(Action('femur4', 'SUBIR'))
  actions.append(Action('femur3', 'DESCER'))
  actions.append(Action('coxa4', 'TRAS'))
  actions.append(Action('femur2', 'SUBIR'))
  actions.append(Action('femur4', 'DESCER'))
  actions.append(Action('coxa2', 'FRENTE'))
  actions.append(Action('femur6', 'SUBIR'))
  actions.append(Action('femur2', 'DESCER'))
  actions.append(Action('coxa6', 'TRAS'))
  actions.append(Action('femur6', 'DESCER'))

  actions.append(Action('JUNTO', 'INICIO'))
  actions.append(Action('coxa1', 'MEIO'))
  actions.append(Action('coxa4', 'MEIO'))
  actions.append(Action('coxa2', 'MEIO'))
  actions.append(Action('coxa5', 'MEIO'))
  actions.append(Action('coxa3', 'MEIO'))
  actions.append(Action('coxa6', 'MEIO'))
  actions.append(Action('JUNTO', 'FIM'))
  return actions



def goDown(initial):
    actions = list()
    actions.append(Action('SPECIAL','DOWN'))
   # actions.append(Action('JUNTO', 'INICIO'))
    for i in range(1, 7):
        actions.append(Action('tibia'+str(i), 'MEIO'))
        actions.append(Action('femur'+str(i), 'MEIO'))
  #  actions.append(Action('JUNTO', 'FIM'))

    return actions


def goUp(initial):
    actions = list()
    actions.append(Action('SPECIAL','UP'))

#    actions.append(Action('JUNTO', 'INICIO'))
    for i in range(1, 7):
        actions.append(Action('tibia'+str(i), 'MEIO'))
        actions.append(Action('femur'+str(i), 'MEIO'))

 #   actions.append(Action('JUNTO', 'FIM'))
    return actions


'''
Responsavel por receber commandos externos e chamar os metodos da classe Robo acima para ativar os atuadores
'''
class RobotNode:
    '''
    TODO -> colocar a excutacao das acoes no Robo Diretamente, o Robo(classe) deverá receber somente uma função como parametro, e dentro dela deve ser tratado o caso de inicializacao, nao aqui.

    '''
    def __init__(self,     segmentsDictionary):
        self.robot = Robot(segmentsDictionary)
        self.segmentsDictionary = segmentsDictionary

        self.running = True
        self.command = Command()#so pra acessar as constantes
    '''
    Implementa o listener do no de comandos externos
    '''
    def callbackReceiveCommand(self, data):
        #Colocar uma acao dumy
        rospy.loginfo("Command Received:|%s|",data.command)
        if data.command == self.command.FRENTE:
            self.robot.addCommandInQueue( goFrontnewtest, data.parameter)
        elif data.command == self.command.TRAS:
            self.robot.addCommandInQueue( goBack, data.parameter)
        elif data.command == self.command.DIREITA:
            self.robot.addCommandInQueue( goRight, data.parameter)
        elif data.command == self.command.ESQUERDA:
            self.robot.addCommandInQueue( goLeft, data.parameter)
        elif data.command == self.command.ROTATE_LEFT:
            self.robot.addCommandInQueue( turnLeft, data.parameter)
        elif data.command == self.command.ROTATE_RIGHT:
            self.robot.addCommandInQueue( turnRight, data.parameter)
        elif data.command == self.command.INICIAR:
            self.robot.addCommandInQueue( initialize, data.parameter)
        elif data.command == self.command.DESCER:
            self.robot.addCommandInQueue( goDown, data.parameter)
        elif data.command == "SUBIR":#$//self.command.SUBIR:
            self.robot.addCommandInQueue( goUp, data.parameter)
        elif data.command == self.command.DANCAR:
            self.robot.addCommandInQueue( danceBaby, data.parameter)
        elif data.command == self.command.DESLIGAR:
            #TODO Enviar mensagem pro servo Master desligar a energia dos controladores
            rospy.loginfo("Comando Desligar Recebido")
            rospy.loginfo("COmm:| %s|", self.command.FRENTE)
            rospy.loginfo("X> " + self.command)
            self.robot.turnOff()
            self.running = False
            return
        else:
            rospy.loginfo("Comando: " + self.command.FRENTE + " nao reconhecido")

    def listener(self):
        rospy.Subscriber("RoboAranhaController", AranhaCommand, self.callbackReceiveCommand)


    def loop(self):
        rospy.loginfo("RobotNode.loop()")
        rospy.init_node('RoboAranhaNode', anonymous=True)

        rate = rospy.Rate(5) # 10hz

        while(self.running):
            self.robot.runRoboTicks()
            rate.sleep()



if __name__ == '__main__':
    robot = RobotNode( createDicionaryJoints())
    robot.listener()
    robot.loop()
