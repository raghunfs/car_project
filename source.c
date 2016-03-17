
#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#define SET_BIT(port , bit) {port |= (1<<bit);}
#define RESET_BIT(port, bit) {port &= ~(1<<bit);}


#define TOP_SERVO 		1250 // 200HZ
#define SERVO_CENTER 	375
#define SERVO_STEP		15

#define TOP_MOTOR		200 // for 10 kHz
#define INITIAL_SPEED 		25
#define STEP 			1
#define MAX_SPEED 		66 // TOP_MOTOR/3

// using PE5 for start and stop
#define PUSHB_PORT 		PORTE5
#define PUSHB_DDE  		DDE5
#define PUSHB_PIN  		PINE5

// PORTA to read IR sensor values
#define SENSOR_PORT 		PORTA
#define SENSOR_DDR 		DDRA


// DDR states
#define INPUT 			0
#define OUTPUT 			1

//car states
#define RUNNING 		1
#define STOP 			0

// sensor sides
#define GET_RIGHT 		0x0F
#define GET_LEFT 		0XF0

#define SERVO_DDR 		DDB5
#define SERVO_PORT 		PB5

#define MOTOR_PWM_DDR 	DDH3
#define MOTOR_PWM_PORT 	PH3

#define MOTOR_INPUT_DDR DDRK
#define MOTOR_INPUT_PORT PORTK


// using portC for LEDs
#define LED_PORT PORTC
#define LED_DDR  DDRC

#define SERVO_TURN(value)\
{\
	TIMSK3 |= 1 << OCIE3A;\
	TCCR1A |= 1 << COM1A1;\
	OCR1A = value;\
}

#define SERVO_BACK()\
{\
	TIMSK3 &= ~(1 << OCIE3A);\
	TCCR1A &= ~ (1 << COM1A1);\
}

unsigned char car_state = STOP;
unsigned short int servo_position;
unsigned char prev_IR_read = 0x18;


ISR(TIMER3_COMPA_vect)
{
	SERVO_BACK();
}

void follow_line(void)
{
	//uint8_t IR_read = ~ SENSOR_PORT;
	uint8_t IR_read = ~PINA;
	
	if(0xFF == IR_read)
	{
		PINC |= _BV(PC0);  //debug
	}
	else if(IR_read > 0x18)
	{
		PINC |= _BV(PC1);  //debug
		servo_position = (prev_IR_read < IR_read ? (servo_position - SERVO_STEP) 
					: (servo_position + SERVO_STEP));
		SERVO_TURN(servo_position);
	}
	else if(IR_read < 0x18)
	{
		//PINC |= _BV(PC0);  //debug
		servo_position += SERVO_STEP;
		servo_position = (prev_IR_read > IR_read ? (servo_position + SERVO_STEP) 
						: (servo_position - SERVO_STEP));
						
		SERVO_TURN(servo_position);
	}
	
	prev_IR_read = IR_read;
}


//toggle car state inside ISR.
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
		servo_position = SERVO_CENTER;
		
	}

	else if(car_state == RUNNING)
	{
		car_state = STOP;
		// reset all the pins set in if condition

		PORTH &= ~(1<<PH3);
		PORTK &= ~(1<<PK1);
		TIMSK4 &= ~(1<<OCIE4A);
		TCCR4A &= ~(1<< COM4A1);
		RESET_BIT(LED_PORT, PC0);
	}
}

// Initialize DDR pins
void init(void)
{

	DDRE = 0X0; // DDE5 as input
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
	ICR3 = 1250; // 200hz
}


int main(void)
{
	unsigned char state = 0;
	unsigned char prev_state = 0;
	
	init();

	sei();

	// using timer/counter three for servo control
	
	while(1)
	{
		state = !(PINE & _BV(PE5));
		//state = !(PINA & _BV(PA0));
		if(1 == state && 0 == prev_state)
		{
			pushb_handle();
			_delay_ms(200);
		}
		prev_state = state;

		if(RUNNING == car_state)
		{
			follow_line();
		}
	}
	return 0;
}


