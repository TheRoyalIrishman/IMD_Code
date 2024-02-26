/*
 * Lab2.cpp
 *
 * Created: 1/30/2024 9:57:21 AM
 * Author : CamsComputer
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>


int main(void)
{
	
	DDRD = 0 << PORTD2; // set D2 as input
	DDRB = 1 << PORTB1; // set B1 as output
	
	TCCR1A = (0 << COM1A1) | (1 << COM1A0); //Toggle OCCR1A on Compare, run in CTC (Clear Timer on Compare)
	TCCR1B = (0 << WGM11) | (1 << WGM12) | (1 << CS12) | (0 << CS11) | (0 << CS10); // this sets the prescalar to 256 and sets to CTC
	
	TIMSK1 = 1 << OCIE1A; //Enable Match Interrupt
	
	OCR1A = 46847; // (0.75s) * (16 MHz / 256) - 1
	
	sei(); // sets the interrupts
	
    /* Replace with your application code */
    while (1) 
    {
    }
}

ISR(TIMER1_COMPA_vect) {
	// run test to see if this is backwards because this is a pull high resistor
	if (!(PIND & (1 << PIND2))) {
		OCR1A = 12499; // (0.2s) * (16 MHz / 256) - 1
	} else {
		OCR1A = 46847; // (0.75s) * (16 MHz / 256) - 1
	}
	
	PORTB ^= 1 << PORTB1; //Toggle Pin B1
}

