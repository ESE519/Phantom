Phantom
=======
  Group Member: Chenyang Zhu, Hanfei Sun, Chao Liu
  
Objective
-----------------------------------
  Nowadays wearable devices are more accessible to human beings, with wearable devices, human beings can do things more easily and effortlessly. This project, PhanTom, is to design a gesture-based control system which lets users to use the movements and motions of hands to effortlessly control digital devices. We built a master system which integrates EMG signal as well as IMU signal to get gesture and motion control to seamlessly interpret what the hands and fingers doing. Then we transmit the control signals over wireless channels, to the slave systems which can execute correspond actions according to the instructions. We implement multiple slave systems, including vehicle, cannon, quadrotor and some software on PC. And in the final demo, we set two slave systems, attack and defend system respectively. The attack system consisted of a vehicle and a canon, the vehicle could move around with the canon. And the canon could also aim around and shoot. The defend system consisted of a canon which is much bigger and can aim around and shoot. Two users wear our devices can fight with each other. With some simple and intuitive gestures and hand motions, people can play the game easily.

File List
-----------------------------------
  Master System: Main part of integrating EMG signal and IMU signal. Then with sensor fusion to send out control signals. 
  Slave System:  Including code controling vehicle, attack canon, defend canon and PPT control.  
  Quadrotor: Including the library for Quadrotor control and PID test.  
  IMU: Including the IMU library to use.  
  ESE519 REPORT.

Project Blog
-----------------------------------
  https://sites.google.com/site/phantomupenn2013/home

Demo Video
-----------------------------------
  https://www.youtube.com/watch?v=bnSDAKjZlDg
