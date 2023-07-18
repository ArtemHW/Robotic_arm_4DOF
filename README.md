# Robotic_arm_4DOF

## Video
<a href="https://www.youtube.com/watch?v=iODTWuAJiZA" target="_blank">![Watch the video](https://github.com/ArtemHW/images/blob/main/robotic_arm_video_screen.png)</a>
## Description
The purpose of this project was to create a robotic arm with 4 degrees of freedom (4DOF). The project utilized the MCU STM32F302. To ensure a reliable program, FreeRTOS was used. The robotic arm can be controlled using two joysticks. Additionally, it allows the user to save different coordinates of the robotic arm's position, effectively creating a program path.

Once programmed, the robotic arm can execute preprogrammed paths, which are stored in the EEPROM memory. This feature ensures that even if the robotic arm is turned off, it will retain the programmed path. The motion of the robotic arm is achieved using four servo motors.
## Block scheme
![App Screenshot](https://github.com/ArtemHW/images/blob/main/robotic_arm.svg)
