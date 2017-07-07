# SpiderRobot
Hexapod ROS robot

![Alt text](complete-robot.jpg?raw=true "RoboAranha - SpiderRobot")


[Demonstration](https://www.youtube.com/watch?v=ryPauhWIOoM&t=1m11s)




The project consists of an embedded system used to control a hexapod robot, using ROS. The control base consists of a web page. In it the user can send commands to the robot with a WiFi connection, eliminating the need for cables and giving a greater degree of freedom for the robot.


![Alt text](general-diagram.png?raw=true "Block Diagram - SpiderRobot")


The robot consists of a microcontroller, servomotors and drivers mounted on the appropriate hexapod structure, as can be seen in figure below. The microcontroller is responsible for all information processing. It is responsible to generate a communication network, receives commands to the decoder to send them to the servomotors. The servomotors are responsible for converting the signal into motion. One of them is required at each robot joint, resulting in 18 SG90 servomotors. Finally, the drivers have a function of allowing communication between the microcontroller and the servomotors. Each driver communicates with the microcontroller using the I2C protocol, converts the signal to one of its 16 PWM outputs where the servomotors are connected. 


![Alt text](comunnication.png?raw=true "Comunnication Diagram - SpiderRobot")


![Alt text](state-machine.png?raw=true "State Machine Diagram - SpiderRobot")

The project was done for the discipline "Oficinas de Integraço III" of the course of Computer Engineering, UTFPR - Curitiba.

Project members:
 - Lucas Simões (manager)
 - Gustavo P. Ramos
 - Matheus F. Ferraz
 - Rodrigo Y. Endo 
