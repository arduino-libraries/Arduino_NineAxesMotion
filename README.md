NAxisMotion
-----------
The NAxisMotion.cpp and NAxisMotion.h files are C++ wrapper codes for the 
BNO055.c and BNO055.h Sensor API. The wrapper code has been designed to
abstract the Sensor API and also to give an idea on how to use the 
advanced features in the Sensor API. Apart from that it acts a bridge
between the Sensor API and the Arduino framework. Copy this library into
"yourArduinoInstallation"/libraries folder.
	
	
-------------------------------------------------------------------------------	
There are 4 examples with the NAxisMotion library. 
 
 - BareMinimum: This example code is as the name says the minimum code
	required to use the NAxisMotion sensor shield.
	
 - Euler: This example code reads out the Euler angles in the NDoF mode to
	the Serial Monitor. It also reads out the Calibration Status. Each sensor
	and the System itself has its own Calibration Status. See below on how to
	calibrate each of the sensors.

 - Accelerometer: This example code reads out the Accelerometer data and
	associated data which are the Linear Acceleration data, which is the
	Accelerometer data without the gravity vector, the other is the Gravity
	Acceleration data, which is only the gravity vector.
	
 - Motion: This example code is a game to test how steadily you can move an
	object, in this case it is the shield with the Arduino board. The goal is
	to demonstrate on how to use the Any motion and No motion Interrupts.
	
Calibration helps the Sensor identify its environment and automatically
determine offsets. Follow the instructions below to calibrate your sensor.

 - Gyroscope: Keep it steady and do not move it. Preferably keep it on a fixed
	surface such as a table.
	
 - Accelerometer: Rotate the shield slowly and pause at every 45deg for a
	second. Rotate one 1 axis at a time. Preferably rotate along 2 axes.
	
 - Magnetometer: Move the magnetometer in a large 8 like pattern a few times
	gently.
	
	