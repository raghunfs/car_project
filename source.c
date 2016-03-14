#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#define SET_BIT(port , bit) {port |= (1<<bit);}
#define RESET_BIT(port, bit) {port &= ~(1<<bit);}

#define TOP_SERVO 		1250
#define TOP_MOTOR		132
#define INITIAL_SPEED 		30
#define STEP 			1
#define MAX_SPEED 		44 // TOP_MOTOR/3

// using PE5 for start and stop
#define PUSHB_PORT PORTE5
#define PUSHB_DDE  DDE5
#define PUSHB_PIN  PINE5

// PORTA to read IR sensor values
#define SENSOR_PORT PORTA
#define SESNSOR_DDR DDRA

// DDR states
#define INPUT 			0
#define OUTPUT 			1

//car states
#define RUNNING 		1
#define STOP 			0

// sensor sides
#define GET_RIGHT 		0x0F
#define GET_LEFT 		0XF0

#define SERVO_DDR DDB5
#define SERVO_PORT PB5

#define MOTOR_PWM_DDR DDH3
#define MOTOR_PWM_PORT PH3

#define MOTOR_INPUT_DDR DDRK
#define MOTOR_INPUT_PORT PORTK


// using portC for LEDs
#define LED_PORT PORTC
#define LED_DDR  DDRC


unsigned char car_state = STOP;

//toggle car state inside ISR.
//ISR(INT5_vect)
void pushb_handle(void)
{
	if(car_state == STOP)
	{
		// lighta LED
		SET_BIT(LED_PORT, PC0);

		car_state = RUNNING;
		
		// set non inverted PWM
		SET_BIT(TCCR1A,COM1A1);
		// start servo timer
		SET_BIT(TIMSK1,OCIE1A);

		SET_BIT(PORTB,PB5);
		

		// set non inverted PWM
		SET_BIT(TCCR4A,COM4A1);

		// start motor timer
		SET_BIT(TIMSK4,OCIE4A);

		// set OCXA pins

		SET_BIT(PORTH,PH3);

		// set for clockwise rotation
		SET_BIT(MOTOR_INPUT_PORT, PK1);
		// set initial speed
		OCR4A = INITIAL_SPEED;

	}
	else
	{
		car_state = STOP;
		// reset all the pins set in if condition

		//RESET_BIT(LED_PORT, PC0);
		RESET_BIT(TIMSK1,OCIE1A);
		RESET_BIT(TCCR1A,COM1A1);
		RESET_BIT(TIMSK4,OCIE4A);
		RESET_BIT(TCCR4A,COM4A1);
	}
}

// Initialize DDR pins
void init(void)
{
	DDRE = 0X0; // DDE5 as input
	SESNSOR_DDR = INPUT;
	DDRB = _BV(PB5); // DDB5 as output
	DDRH = _BV(PH3); // DDH3 as output
	DDRK = _BV(PK1)|_BV(PK2)|_BV(PK3);
	LED_DDR = _BV(PC1)|_BV(PC0); // PC0 and PC1 as output

	// set PRESCALE to 64 (1<<CS11)|(1<<CS10)
	// set PWM to fast mode
	TCCR1A |= (1<<WGM11);
	TCCR1B |= (1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10);
	
	// count value equivalent to 20 ms, fOCnxPWM is set to 200HZ
	ICR1 = TOP_SERVO;

	// timer/counter 4 for motor
	// prescalar as 8
	TCCR4A |= (1<<WGM41);
	TCCR4B |= (1<<WGM43)|(1<<WGM42)|(1<<CS41);
	ICR4 = TOP_MOTOR;
}


int main(void)
{
	unsigned char color;
	unsigned char right_side, left_side;
	unsigned char state;
	unsigned char prev_state = 0;
	init();

	sei();
	
	while(1)
	{
		
		state = !(PINE & _BV(PE5));
		//if(1 == state && 0 == prev_state)
		if(1)
		{
			PINC |= _BV(PC1);
			_delay_ms(500);
			//pushb_handle();
			
		}
	
		prev_state = state;

		if(RUNNING == car_state)
		{
			color = SENSOR_PORT;
		
			if(0XFF != car_state)
			{
				right_side = color & GET_RIGHT;
			
				left_side = (color & GET_LEFT)>>4;
				OCR1A = 125;
			}
		
		}
	
	}
return 0;	
}


