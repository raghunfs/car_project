#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include "carproject.h"


unsigned char car_state = STOP;


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

	}
	return 0;
}
