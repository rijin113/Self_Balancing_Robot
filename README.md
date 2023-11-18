# GyroBot - Self Balancing Robot

A self-balancing robot programmed on the STM32! It uses an L298N motor driver to control the
motors and a 6-axis IMU to capture gyroscope and accelerometer readings. It is programmed using 
a state machine and combined with a PID controller to maintain robot stability.

## Design Process
![planning](Images/planning.png)

## Modelling
<p align="center" width="100%">
  <img src="Images/design1.png"  width="33%" hspace="10" alt="design 1">
  <img src="Images/design2.png" width="33%" hspace="10" alt="design 2">
</p>

## Results
<p align="center" width="100%">
  <img src="Images/robotV1.jpg" width="27%" hspace="10" alt="robot v1">
  <img src="Images/robotV2.jpg" width="27%" hspace="10" alt="robot v2">
</p>

## Next Steps
- Add Bluetooth control to the robot
- Add a Kalman or a complementary filter that fuses gyroscope and accelerometer readings for better pitch accuracy