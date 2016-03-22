Description:
The car project is a non-loopback based solution for a line follower robot car. The speed and orientation of the car is assigned based on 
IR sensor's output. There are three timer/counters used for this project implementation. Timer/counter 4 is used at 15khz for the PWM 
output generation for motor. Timer/counter 3 is used at 200hz for PWM output generation for servo. Timer/counter 3 is used at 1000hz for 
handling the read sensor values. All the timer/counters are used in fast mode (mode 14). 

Implemented features:
1) Switching on/off the car using push button.
2) Motor control using PWM signal
3) Servo control using PWM signal
4) Speed control based on the curvature of the path.

Omitted features:
1) Calculating the speed of car using tachometer.
2) Displaying the current speed or configuration.
3) safety features to handle car when it moves out of track.

Program control flow:
1) Initialize the registers and counters calling init()
2) enable the interrupts.
3) Poll was push button press.
4) If the push button gets pressed when car is at halt,
	a) enable non inverted PWM outputs for motor and servo
	b) enable input pin for motor.
	c) Start timer/counter 3 for sensor reading handling
	d) Set initial servo position and initial motor speed.
5) The ISR for timer/counter 3 handles the read sensor input.
6) The servo movement is based on the read values from the sensor registers.
7) The motor speed is maximum if the white line beneath the IR sensor four and five.
8) If the white line is below any other sensor then the speed is set to minimum.
9) If the push button gets pressed when car is running,
	a) All the timer/counters are disabled.
	b) vehicle stops moving.
	
To run the project:

1) Make
	To compile the project.
2) Make program
	compile and load the .hex file into car.
3) Make clean
	To clean the project.
