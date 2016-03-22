#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include "carproject.h"

extern unsigned char car_state;

/*************************************************************************************************
Function: ISR()
Parameters: TIMER3_COMPA_vect
Return: 
Description: This is a interrupt service routine for timer/counter 3. The periodicity of this ISR
			 is 1 ms. The ISR handles the speed, orientation of the car based on a non-loopback based 
			 implementation. The speed and orientation is decided based on the sensor reading. 
/*************************************************************************************************/
ISR(TIMER3_COMPA_vect)
{
	uint8_t IR_read = ~PINA;

	switch(IR_read)
	{
		case 0x80:
			OCR4A = INITIAL_SPEED;
			OCR1A = SERVO_RIGHTMOST;
			break;
		case 0x40:
			OCR4A = INITIAL_SPEED;
			OCR1A = SERVO_RIGHTMOST -  (1 * SERVO_STEP);
			break;
		case 0x20:
			OCR4A = INITIAL_SPEED;
			OCR1A = SERVO_RIGHTMOST -  (2 * SERVO_STEP);
			break;
		case 0x10:
			OCR4A = 60;
			OCR1A = SERVO_CENTER;
			break;
		case 0x08:
			OCR4A = 60;
			OCR1A = SERVO_CENTER;
			break;
		case 0x04:
			OCR4A = INITIAL_SPEED;
			OCR1A = SERVO_LEFTMOST + (2 * SERVO_STEP);
			break;
		case 0x02:
			OCR4A = INITIAL_SPEED;
			OCR1A = SERVO_LEFTMOST + (1 * SERVO_STEP);
			break;
		case 0x01:
			OCR4A = INITIAL_SPEED;
			OCR1A = SERVO_LEFTMOST;
			break;
		default:
			//OCR1A = SERVO_CENTER;
			break;
	}
}

/*************************************************************************************************
Function: pushb_handle
Parameters: void
Return: void
Description: This function handles the push button press. When button is pressed to start the car,
			 the state of the car is changed. Timer/counter 1, Timer/counter 3 and Timer/counter 4
			 are set to one. Initial speed, direction of rotation and car orientation are also set.
			 When the button is pressed to stop the car, all of the running timers/
			 counters are made zero.
/*************************************************************************************************/
void pushb_handle(void)
{
	if(car_state == STOP)
	{
		// glow led 0.
		LED_PORT |= (1 << PC0);
		
		// change car state.
		car_state = RUNNING;
		
		// configure motor
		PORTK |= (1 << PK0);
		TCCR4A |= 1 << COM4A1;
		OCR4A = INITIAL_SPEED;

		// configure servo
		TCCR1A |= 1 << COM1A1;
		PORTB |= (1 << PB5);
		OCR1A = SERVO_CENTER;

		// start timer to handle IR sensing
		TIMSK3 |= 1 << OCIE3A;

	}
	else if(car_state == RUNNING)
	{
		OCR1A = SERVO_CENTER;
		car_state = STOP;
		// reset all the pins set in if condition

		PORTB &= ~(1<<PB5);
		PORTH &= ~(1<<PH3);
		PORTK &= ~(1<<PK1);
		TCCR4A &= ~(1<< COM4A1);
		TCCR1A &= ~(1<< COM1A1);
		LED_PORT &= ~(1 << PC0);
		TIMSK3 &= ~(1 << OCIE3A);
	}
}


/*************************************************************************************************
Function: init
Parameters: void
Return: void 
Description: Initializes the DDRs of Sensor, Servo, Motor and LED. Also sets the mode, prescalar 
			 for ICRX value for all the timer/counters
/*************************************************************************************************/

void init(void)
{

	DDRE = INPUT; // DDE5 as input
	SENSOR_DDR = INPUT;

	DDRB = _BV(PB5); // DDB5 as output
	LED_DDR = _BV(PC1)|_BV(PC0); // PC0 and PC1 as output

	DDRH = _BV(PH3);    // enable motor PWM
	DDRK = _BV(PK1)|_BV(PK0);  // enable motor input pins.

	// fast mode, pre-scalar to 64 
	// count value equivalent to 5 ms, fOCnxPWM is set to 200HZ
	TCCR1A |= (1<<WGM11);
	TCCR1B |= (1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10);
	ICR1 = TOP_SERVO;

	// timer/counter 4 in fast mode for wheel rotation.
	// prescalar as 8
	TCCR4A |= (1<<WGM41);
	TCCR4B |= (1<<WGM43)|(1<<WGM42)|(1<<CS41); 
	ICR4 = TOP_MOTOR;

	// timer3 for servo control
	// fast mode, pre-scalar 64 
	TCCR3A |= (1<<WGM31);
	TCCR3B |= (1<<WGM33)|(1<<WGM32)|(1<<CS31)| (1 << CS30); 
	ICR3 = TOP_SENSOR_COUNTER;
}

