# GyroBot - Self Balancing Robot 

A self balancing robot programmed on the STM32! It uses a 6-axis IMU to capture
gyroscope and accelerometer readings. It is programmed using a state machine and combined
with a PID controller to maintain robot stability. Note, the robot uses a L298N motor driver 
to control the motors.

Design Process
![Image1](Images\planning.png)

Modelling
![Image2](Images\design1.png)
![Image3](Images\design2.png)

Control Logic
![Image4](Images\state_machine.png)

