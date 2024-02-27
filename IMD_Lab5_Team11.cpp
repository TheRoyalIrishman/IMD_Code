/*
 * Lab 5
 * IMD_Demo5_H_Bridge.cpp
 *
 * Created: 2/26/2024 9:37:00 PM
 * Author : Cameron Clarke, Logan York
 */ 

#include <avr/interrupt.h>
#include <avr/io.h>

int main(void)
{
	sei(); //Enable Interrupts
	//Configure Pins
	DDRB = (1<<PORTB2)|(1<<PORTB1)|(1<<PORTB0); //Set Pin PB0,PB1(OC1A),PB2 to output
	DDRD = (1<<PORTD6); //Set Pin PD6(OC0A) to output
	//**************************************************************************************
	//Configure Timer 1
	//**************************************************************************************
	//Setup Compare Output Mode for Channel A and Wave Generation Mode
	TCCR1A=(0<<COM1A1)|(1<<COM1A0)|(0<<WGM11)|(0<<WGM10);
	TCCR1B = (0<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(1<<CS10); //Define Clock Source Prescaler
	OCR1A = 25000; //Set Output Compare A to match every 24999 clocks (24999x8uS =100mS)
	TIMSK1 = (1<<OCIE1A); //Enable Interrupt for Output Compare A Match
	//**************************************************************************************
	//Configure Timer 0
	//**************************************************************************************
	TCCR0A=(1<<COM0A1)|(0<<COM0A0)|(1<<WGM01)|(1<<WGM00);//Setup Fast PWM Mode for Channel A
	TCCR0B = (0<<WGM02)|(1<<CS02)|(0<<CS01)|(1<<CS00); //Define Clock Source Prescaler
	OCR0A = 64; //Initialized PWM Duty Cycle
	//**************************************************************************************
	//Configure A/D
	//**************************************************************************************
	//Select Analog Port 0 and Internal Reference Voltage;
	ADMUX = (0<<REFS1)|(1<<REFS0)|(1<<ADLAR)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0);
	ADCSRA = (1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0); //Enable A/D, Enable Interrupt, Set A/D Prescalar
	DIDR0 = (1<<ADC0D); //Disable Input Buffers
	ADCSRA |= (1<<ADSC); //Start Conversion
	while(1) {
	//Wait and do nothing
	}
}
//Interrupt Routine for Timer 1 Match
ISR (TIMER1_COMPA_vect) {
	PORTB ^= 0x01; //Toggle Pin PB0
	ADCSRA |= (1<<ADSC); //Start Conversion
}
//Interrupt Routine for A/D Completion
ISR (ADC_vect) {
	PORTB ^= 0x04;//Toggle Pin PB2
	OCR0A = ADCH; //Load A/D High Register in OCR0A to set PWM Duty Cycle
}
