#!/usr/bin/env python
import rospy
from servo_master.msg import ServoMessageCallibration
import PCA9685


class Servo_Master_Callibration: 
    def __init__(self, addresses):
          self.board = dict()
          for address in addresses:
              self.board[address] = PCA9685.PCA9685(address, pwm_frequency=60 )
              pass
    def callback(self, data):
        rospy.loginfo("/ServoMasterCallibration Message Received")
        rospy.loginfo("/ServoMaster Message received. Adress: %d, ServoNum %d pulseLenght %d", data.address, data.servoNum, data.pulseLenght)

        address = data.address
        servoNum = data.servoNum
        pulseLenght = data.pulseLenght
        self.board[address].set_pwm(servoNum, 0, pulseLenght)
        
    def Node_Listener(self):
        rospy.init_node('ServoMasterListener', anonymous=True)
        rospy.Subscriber("/ServoMasterCallibration", ServoMessageCallibration , self.callback)
        rospy.spin()

if __name__ == '__main__':
    #todo pegar a lista de enderecos do servidor de configuracoes
    servoMasterCallibration = Servo_Master_Callibration({0x40, 0x41})#{0x40, 0x41})
    servoMasterCallibration.Node_Listener()
