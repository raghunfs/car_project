#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include "carproject.h"

extern unsigned char car_state;

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


void pushb_handle(void)
{
	if(car_state == STOP)
	{
		// light a LED
		//SET_BIT(LED_PORT, PC0);

		car_state = RUNNING;
		//set PB5 as output
		PORTK |= (1 << PK0);

		TCCR4A |= 1 << COM4A1;

		OCR4A = INITIAL_SPEED;

		TCCR1A |= 1 << COM1A1;
		PORTB |= (1 << PB5);

		OCR1A = SERVO_CENTER;

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
		RESET_BIT(LED_PORT, PC0);
		RESET_BIT(LED_PORT, PC1);
		TIMSK3 &= ~(1 << OCIE3A);
	}
}

// Initialize DDR pins
void init(void)
{

	DDRE = INPUT; // DDE5 as input
	SENSOR_DDR = INPUT;

	DDRB = _BV(PB5); // DDB5 as output
	LED_DDR = _BV(PC1)|_BV(PC0)|_BV(PC5); // PC0 and PC1 as output

	DDRH = _BV(PH3);
	DDRK = _BV(PK1)|_BV(PK0);

	// set PRESCALE to 64 (1<<CS11)|(1<<CS10)
	// set PWM to fast mode
	// count value equivalent to 5 ms, fOCnxPWM is set to 200HZ
	TCCR1A |= (1<<WGM11);
	TCCR1B |= (1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10);
	ICR1 = TOP_SERVO;

	// timer/counter 4 in fast mode for wheel rotation.
	// prescalar as 8
	TCCR4A |= (1<<WGM41);
	TCCR4B |= (1<<WGM43)|(1<<WGM42)|(1<<CS41); // 8
	ICR4 = TOP_MOTOR;

	// timer3 for servo control
	TCCR3A |= (1<<WGM31);
	TCCR3B |= (1<<WGM33)|(1<<WGM32)|(1<<CS31)| (1 << CS30); //  pre-scalar 64
	ICR3 = 250;
}

