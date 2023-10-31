# GyroBot - Self Balancing Robot 

A self balancing robot programmed on the STM32! It uses a 6-axis IMU to capture
gyroscope and accelerometer readings. It is programmed using a state machine and combined
with a PID controller to maintain robot stability. Note, the robot uses a L298N motor driver 
to control the motors.

## Design Process
![planning](Images/planning.png)

## Modelling
<div style="display: flex; flex-direction: row;">
  <img src="Images/design1.png" width="400" alt="design 1">
  <img src="Images/design2.png" width="400" alt="design 2">
</div>

## Control Logic
<div style="display: flex; justify-content: center;">
  <img src="Images/state_machine.png" width="400" alt="state_machine">
</div>