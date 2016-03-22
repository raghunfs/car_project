#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include "carproject.h"


unsigned char car_state = STOP;

/*************************************************************************************************
Function: main
Parameters: void
Return: 0
Description: calls the initialization function, enables interrupts and constantly polls for push
			 button press. If the push button gets pressed then corresponding handling function 
			 gets called.
/*************************************************************************************************/

int main(void)
{
	unsigned char state = 0;
	unsigned char prev_state = 0;

	init();
	sei();

	while(1)
	{
		state = !(PINE & _BV(PE5));
		// if the pin was not down before and is down now, then handle.
		if(1 == state && 0 == prev_state)
		{
			pushb_handle();
			// delay to suppress multiple button press effect. 
			_delay_ms(200);
		}
		prev_state = state;

	}
	return 0;
}
