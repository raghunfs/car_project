#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#define SET_BIT(port , bit) {port |= (1<<bit);}
#define RESET_BIT(port, bit) {port &= ~(1<<bit);}

#define TOP 			1250
#define INITIAL_SPEED 	100
#define STEP 			5
#define MAX_SPEED 		415 // TOP/3

// using PE5 for start and stop
#define PUSHB_PORT PORTE5
#define PUSHB_DDE  DDE5
#define PUSHB_PIN  PINE5

// PORTA to read IR sesnsor values
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

#define MOTOR_DDR DDH3
#define MOTOR_PORT PH3

unsigned char car_state;

//toggle car state inside ISR.
ISR(INT5_vect)
{
	if(car_state == STOP)
	{
		car_state = RUNNING;
		MOTOR_PORT |= 1 << MOTOR_PORT;
	}
}

// Initialize DDR pins
void init(void)
{
	PUSHB_DDE = INPUT;
	SESNSOR_DDR = INPUT;
	SERVO_DDR = OUTPUT;
	MOTOR_DDR = OUTPUT;
	
	// set PRESCALE to 64 (1<<CS11)|(1<<CS10)
	// set PWM to fast mode
	TCCR1A |= (1<<WGM11)|(1<<COM1A1);
	TCCR1B |= (1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10);
	
	// count value equivalent to 20 ms, fOCnxPWM is set to 200HZ
	ICR1 = TOP;
}


int main(void)
{
	unsigned char color;
	unsigned char right_side, left_side;
	unsigned char speed = INITIAL_SPEED;
	init();
	
	//enable interrupts
	sei();
	
	while(RUNNING == car_state)
	{
		color = SENSOR_PORT;
		
		if(0XFF != car_state)
		{
			right_side = color & GET_RIGHT;
			
			left_side = (color & GET_LEFT)>>4;
			SENSOR_PORT = 125;
		}
		
	}
	return 0;
}