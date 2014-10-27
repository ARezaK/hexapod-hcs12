hexapod-hcs12
=============
Source code for controlling PhantomX AX Hexapod Mark II robot using:  

Freescale HCS12  

SRF05 - Ultrasonic Ranger   

TPA 81 – Infrared, Thermopile Array  

DE-SW050 – Voltage Regulator  

Nerf Gun  


This code is for an HCS12 that is connected to: an ArbotiX on a Hexapod, Gun Trigger Servo, Ultrasonic Sensor, and Thermopile. Actions: Turn slowly and read thermopile. When a heat source above ambient is detected, it will shoot the gun and walk towards the heat source. Using the ultrasonic sensor, it will approach the heat source until it is close. The gun will shoot again and the robot will turn around 180 degrees and search for another heat source.   

