/*
* BlinkyBlinkyExample.cpp
*
* Created: 1/18/2022 9:40:59 AM
* Author : Zach
*/
#include <avr/io.h>
int main(void)
{
	//Configure Pins
	DDRD = (1 << PORTD4); //Set pin D4 as output, also sets D2 to input
	DDRB = (1 << PORTB1); //Set pin B1 as output
	PORTB |= (1 << PORTB1); // turns on forever
	//Main loop (Do forever)
	while(1) {
		//Note: index2 is an uint16_t. In an 8-bit AVR this is a 16-bit
		//variable therefore we can count up to 65536
		for ( uint8_t index1=0; index1<=100; index1=index1+1 ) {
			for ( uint16_t index2=0; index2<=10000; index2=index2+1 ){
				//Do nothing
			}
		}
		//Toggle Port B
		PORTB ^= 0x02; // 00000010 is equivalent
		//Check if button has been pushed
		if(PIND & (1<<PIND2)) {
			PORTD &= ~(1<<PORTD4);//Clear Pin
		}
		else {
			PORTD |= (1<<PORTD4);//Set Pin
		}
	}
}
