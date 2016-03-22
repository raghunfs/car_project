#ifndef _VALUES
#define _VALUES

// set max speed for red and green car to 60

#define TOP_SERVO 		    1250 // 200HZ
#define SERVO_CENTER 		375
#define SERVO_LEFTMOST		290    // value for red and green car 
#define TOP_SENSOR_COUNTER  250

//yellow car
#if 0
#define SERVO_RIGHTMOST		455
#define SERVO_STEP			23
#else // red and green car

#define SERVO_RIGHTMOST		438
#define SERVO_STEP			20
#endif


#define TOP_MOTOR			200 // for 10 kHz
#define INITIAL_SPEED 		29	//yellow car
#define MAX_SPEED 			66 // TOP_MOTOR/3

// using PE5 for start and stop
#define PUSHB_PORT 			PORTE5
#define PUSHB_DDE  			DDE5
#define PUSHB_PIN  			PINE5

// PORTA to read IR sensor values
#define SENSOR_PORT 		PORTA
#define SENSOR_DDR 			DDRA

// DDR states
#define INPUT 			0
#define OUTPUT 			1

//car states
#define RUNNING 		1
#define STOP 			0

#define SERVO_DDR 		DDB5
#define SERVO_PORT 		PB5

// using portC for LEDs
#define LED_PORT PORTC
#define LED_DDR  DDRC


/* Function prototypes */

void pushb_handle(void);
void init(void);
#endif
