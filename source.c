#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#define SET_BIT(port , bit) {port |= (1<<bit);}
#define RESET_BIT(port, bit) {port &= ~(1<<bit);}


#define TOP_SERVO 		1250
//#define TOP_MOTOR		132 // for 15 kHz
#define TOP_MOTOR		200 // for 10 kHz
#define INITIAL_SPEED 		30
#define STEP 			1
#define MAX_SPEED 		66 // TOP_MOTOR/3

// using PE5 for start and stop
#define PUSHB_PORT PORTE5
#define PUSHB_DDE  DDE5
#define PUSHB_PIN  PINE5

// PORTA to read IR sensor values
#define SENSOR_PORT PORTA
#define SENSOR_DDR DDRA

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
		//set PB5 as output
        	PORTK |= (1 << PK1);

        	//TIMSK4 |= 1 << OCIE4A;

        	TCCR4A |= 1 << COM4A1;
        	//enable interrupts

		OCR4A = INITIAL_SPEED;
		//OCR4A = MAX_SPEED;
		//TCCR1A |= 1 << COM1A1;
		PORTB |= (1 << PB5);
		//OCR1A = 125;
		
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
	//LED_DDR = 0xff; // PC0 and PC1 as output
	

	DDRH = _BV(PH3);
        DDRK = _BV(PK1);


	// set PRESCALE to 64 (1<<CS11)|(1<<CS10)
	// set PWM to fast mode
	TCCR1A |= (1<<WGM11);
	TCCR1B |= (1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10);
	
	// count value equivalent to 20 ms, fOCnxPWM is set to 200HZ
	ICR1 = TOP_SERVO;

	// timer/counter 4 in fast mode for wheel rotation.
	// prescalar as 8
	TCCR4A |= (1<<WGM41);
        TCCR4B |= (1<<WGM43)|(1<<WGM42)|(1<<CS41); // 8
        //TCCR4B |= (1<<WGM43)|(1<<WGM42)|(1<<CS42); // 256
        //TCCR4B |= (1<<WGM43)|(1<<WGM42)|(1<<CS40)|(1<<CS41); // 64
	ICR4 = TOP_MOTOR;
	
}


int main(void)
{
	unsigned char color;
	unsigned char right_side, left_side;
	unsigned char state = 0;
	unsigned char prev_state = 0;
	unsigned short int count = 0;
	init();

	sei();

	// using timer/counter three for servo control
	
	while(1)
	{
		state = !(PINE & _BV(PE5));
		//state = !(PINA & _BV(PA0));
		if(1 == state && 0 == prev_state)
		{
			PINC |= _BV(PC1);
			pushb_handle();	
			_delay_ms(500);
							
/*			for(count = 0; count < 10 ; count ++ )
			{
				TCCR1A |= 1 << COM1A1;
				if(count%2 == 0)
				{ 
					OCR1A = 312;
					_delay_ms(500);
				}
				else
				{
					OCR1A = 437;
					_delay_ms(500);
					
				}
				TCCR1A &= ~(1 << COM1A1);
			}
*/
		}
		prev_state = state;

                if(RUNNING == car_state)
                {
                        color = SENSOR_PORT;
                
                        if(0XFF != car_state)
                        {
                                right_side = color & GET_RIGHT;
				if(0xFF > right_side)
				OCR1A = 400;
				
                        
                                left_side = (color & GET_LEFT)>>4;
				if(0xFF > left_side)
                                OCR1A = 325;
                        }
                
                }

	
			
	}
return 0;	
}


