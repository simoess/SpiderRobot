#!/usr/bin/env python
from __future__ import division
import rospy
import PCA9685
import time
from servo_master.msg import ServoMessage
pwm_frequency = 60

# TODO todos os parametros devem ser carregados por um servidor de configuracoes ROS

# servo calibration values. order: pulseLen1, pulseLen2, degrees at pulseLen1, degrees at pulseLen2. linear interpolation/extrapolation is used from these calibration values
servoParameters = { 
	"coxa1": [260, 400, 90, 145], #direita
#>>>>>>> 9a1825cb4c0ab92e2e554df5f41be216314b2485
	"coxa2": [350, 250 , 90 , 55], #direita
	"coxa3": [290, 110, 90, 0], #direita
	"coxa4": [370, 150, 90, 0], #esquerda
	"coxa5": [400, 350, 90 , 75], #esquerda
	"coxa6": [420, 690, 90 , 180 ], #esquerda
#<<<<<<< HEAD
	"femur1": [575, 325, 0, 90], #direita
	"femur2": [540, 305, 0 , 90], #direita
	"femur3": [535, 300, 0 , 90 ], #direita
	"femur4": [215, 433, 0, 90], #esquerda
	"femur5": [220, 480 , 0, 90 ], #esquerda
	"femur6": [190, 410, 0 , 90], #esquerda
	"tibia1": [200, 425, 90, 180],
	"tibia2": [150, 350, 90 ,  180],
	"tibia3": [190, 420, 90, 180],
	"tibia4": [575, 325, 90, 180], #esquerda
	"tibia5": [615, 370 , 90 , 180 ],
	"tibia6": [450, 230, 90 , 180 ]
#=======
#	"femur1": [330, 100, 90, 180], #direita
##	"femur2": [130, 295, 180 , 90], #direita
#	"femur3": [120, 300 , 180 , 90 ], #direita
#	"femur4": [330, 610, 90, 180], #esquerda
#	"femur5": [595, 310 , 180, 90 ], #esquerda
#	"femur6": [590, 310 , 180 , 90], #esquerda
#	"tibia1": [335, 65, 90, 0],
#	"tibia2": [335, 120, 90 , 0 ],
#	"tibia3": [315, 115, 90, 0],
#	"tibia4": [310, 600, 90, 180], #esquerda
#	"tibia5": [360, 700 , 90 , 180 ],#
#	"tibia6": [310, 640, 90 , 180 ]
#>>>>>>> 9a1825cb4c0ab92e2e554df5f41be216314b2485
}

# angulo maximo, minimo e inicial de cada servo
servoAngleLimits = { 
#<<<<<<< HEAD
	"coxa1": [60, 150, 110],
	"coxa2": [60, 120, 90],
	"coxa3": [30, 120, 75],
	"coxa4": [30, 120, 70],
	"coxa5": [60, 120, 90],
	"coxa6": [60, 150, 110],
	"femur1": [0, 180, 90],
	"femur2": [0, 180, 90],
	"femur3": [0, 180, 80],
#=======
#	"coxa1": [60, 150, 90],
#	"coxa2": [60, 120, 90],
#	"coxa3": [30, 120, 90],
#	"coxa4": [30, 120, 90],
#	"coxa5": [60, 120, 90],
#	"coxa6": [60, 150, 90],
#	"femur1": [65, 180, 90],
#	"femur2": [65, 180, 90],
#	"femur3": [65, 180, 90],
#>>>>>>> 9a1825cb4c0ab92e2e554df5f41be216314b2485
	"femur4": [0, 180, 90],
	"femur5": [0, 180, 90],
	"femur6": [0, 180, 90],
	"tibia1": [0, 180, 90],
	"tibia2": [0, 180, 90],
	"tibia3": [0, 180, 90],
	"tibia4": [0, 180, 90],
	"tibia5": [0, 180, 90],
	"tibia6": [0, 180, 90]
}

# contem o endereco da PCA9865 e os canais onde os segmentos das pernas estao localizados
servosLocations = {
#<<<<<<< HEAD
	"coxa1": [0x40, 0], #
	"coxa2": [0x40, 11],
	"coxa3": [0x41, 15],
	"coxa4": [0x40, 15],
	"coxa5": [0x41, 10],
	"coxa6": [0x41, 2],
	"femur1": [0x40, 2],
	"femur2": [0x40, 6],
	"femur3": [0x41, 12],
	"femur4": [0x40, 12],
	"femur5": [0x41, 8],
	"femur6": [0x41, 4],
	"tibia1": [0x40, 4],
	"tibia2": [0x40, 8],
	"tibia3": [0x41, 14],
	"tibia4": [0x40, 14],
	"tibia5": [0x41, 6],
#=======
#	"coxa1": [0x40, 5], #
#	"coxa2": [0x40, 11],
#	"coxa3": [0x41, 10],
#	"coxa4": [0x40, 15],
#	"coxa5": [0x41, 6],
#	"coxa6": [0x41, 1],
#	"femur1": [0x40, 6],
###	"femur2": [0x40, 9],
#	"femur3": [0x41, 8],
#	"femur4": [0x40, 13],
#	"femur5": [0x41, 5],
#	"femur6": [0x41, 2],
#	"tibia1": [0x40, 7],
#	"tibia2": [0x40, 10],
#	"tibia3": [0x41, 9],
#	"tibia4": [0x40, 14],
#	"tibia5": [0x41, 4],
#>>>>>>> 9a1825cb4c0ab92e2e554df5f41be216314b2485
	"tibia6": [0x41, 0],
}

# calculates the servo pulse length for a desired angle. assumes servo angle is linear wrt the pulseLen
def getPulseLenFromAngle(legSection, angle):
	totalSteps = servoParameters[legSection][1] - servoParameters[legSection][0]
	totalAngle = servoParameters[legSection][3] - servoParameters[legSection][2]
	slope = totalSteps / totalAngle
	intercept = servoParameters[legSection][0] - slope * servoParameters[legSection][2]
	pulseLen = angle * slope + intercept
	return int(round(pulseLen, 0))


'''
Publica um topico com os angulos de todos as joints
'''
class Servo_Master: 
    def __init__(self, addresses):
          rospy.loginfo("/ServoMaster Constructor")
    
          self.board = dict()
          self.currentServoAngles = dict()
          self.pulseLenghtLimits = dict()
          
          for address in addresses:
              self.board[address] = PCA9685.PCA9685(address, pwm_frequency=60)
	      self.board[address].set_pwm_freq(60)
          for segment, angles in servoAngleLimits.items():  
              self.currentServoAngles[segment] = angles[2] #inicializa o segmento
              self.pulseLenghtLimits[segment] = dict()

              pulse1 = getPulseLenFromAngle(segment, angles[0])
              pulse2 = getPulseLenFromAngle(segment, angles[1])
              if(pulse1 > pulse2):
                      self.pulseLenghtLimits[segment]['maxPulse'] = pulse1
                      self.pulseLenghtLimits[segment]['minPulse'] = pulse2
              else:
                      self.pulseLenghtLimits[segment]['maxPulse'] = pulse2
                      self.pulseLenghtLimits[segment]['minPulse'] = pulse1
              self.moveServoToAngle(segment, angles[2])
              time.sleep(0.20)
    '''
    Funcao que implementa o listener do recebimento de mensagens do /ServoMaster
    '''          
    def callback(self, data):
        rospy.loginfo("/ServoMaster Message received. Segment: %s, Angle %d ", data.segment, data.angle)
        if(data.segment == "DESLIGAR"):
                for address in addresses:
                        self.board[address].set_all_pwm(0, 0)
                return
        
        segment = (str)(data.segment)
        angle = (int)(data.angle)

        self.moveServoToAngle(segment, angle)
        
	
        
    def Node_Listener(self):
        rospy.init_node('ServoMasterListener', anonymous=True)
        rospy.Subscriber("/ServoMaster", ServoMessage , self.callback)
        rospy.loginfo("/ServoMaster Listener Initialized")
        rospy.spin()


    def moveServoToAngle(self, legSection, angle):
        address = servosLocations[legSection][0]
        servoChan = servosLocations[legSection][1]

	pulseLen = getPulseLenFromAngle(legSection, angle)
        
        
	if self.pulseLenghtLimits[legSection]['minPulse'] <= pulseLen <= self.pulseLenghtLimits[legSection]['maxPulse']:
	    rospy.loginfo( "servo %s legSection told to go to pos %d", legSection, pulseLen)
            self.board[address].set_pwm(servoChan, 0, pulseLen)
	    self.currentServoAngles[legSection] = angle
	else:
            rospy.loginfo( "servo %s legSection told to go to  invalid pos %d", legSection, pulseLen)
            rospy.loginfo( "legSection %s minPulse: %d, maxPulse: %d", legSection,self.pulseLenghtLimits[legSection]['minPulse'], self.pulseLenghtLimits[legSection]['maxPulse'])
            
	    

            
if __name__ == '__main__':
    #todo pegar a lista de enderecos do servidor de configuracoes
    servoMaster = Servo_Master({0x40, 0x41})#{0x40, 0x41})
    servoMaster.Node_Listener()
