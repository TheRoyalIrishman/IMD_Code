/*
 * IMD_Demo2_LCD_BlockingCode.cpp
 *
 * Created: 2/13/2022 2:47:06 PM
 * Author : Zach
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

//Define the codes for TWI actions
#define TWCR_START 0xA4|(1<<TWIE) //send start condition
#define TWCR_STOP 0x94|(1<<TWIE) //send stop condition
#define TWCR_RSTART 0xA4|(1<<TWIE) //send repeated start
#define TWCR_RACK 0xC4|(1<<TWIE) //receive byte and return ack to slave
#define TWCR_RNACK 0x84|(1<<TWIE) //receive byte and return nack to slave
#define TWCR_SEND 0x84|(1<<TWIE) //pokes the TWINT flag in TWCR and TWEN

//Function Prototypes
void InitializeDisplay();
void UpdateDisplay();
void TWI_Start();
void TWI_SendAddress(uint8_t);
void TWI_SendData(uint8_t);
void TWI_Stop();
void DummyLoop(uint16_t);
void UpdateTWIDisplayState();
void UpdateValues();

//Global variables for display strings
volatile uint8_t FirstLineStr[21] =  "S1: XXXX           ";
volatile uint8_t SecondLineStr[21] = "S2: XXXX         ";
volatile uint8_t ThirdLineStr[21] =  "Button: XXX         ";
volatile uint8_t FourthLineStr[21] = "                    ";

//Global variables for TWI state and data
uint8_t display_state =0;
uint8_t TWI_data = 0;
uint8_t TWI_slave_address = 0;
uint8_t TWI_char_index = 0;

volatile uint16_t time = 0;
volatile uint16_t mins = 0;
volatile uint16_t seconds = 0;
volatile uint16_t seconds1 = 0;
volatile uint16_t tenths = 0;

// we can use 8 bit unsigned ints because we're not dealing with as large of numbers, so it can save space
volatile uint8_t minutes_tens_digit = 0; // the tens place of the minutes component
volatile uint8_t minutes_ones_digit = 0; // the ones place of the minutes component
volatile uint8_t seconds_tens_digit = 0; // the tens place of the seconds component
volatile uint8_t seconds_ones_digit = 0; // the ones place of the seconds component
volatile uint16_t tenths_digit = 0; // tenths place

int main(void)
{

    //Configure I/O
    //Set SDA (PC4) and SCL (PC5) as outputs for TWI bus
    DDRC = (1<<PORTC5)|(1<<PORTC4);
    //Configure PD0 as output for debug output
    DDRD = (1<<PORTD0)|(1<<PORTD1)|(1<<PORTD2)|(1<<PORTD3);
	DDRB = (1<<PORTB1);

    //Configure Timer 1 to generate an ISR every 100ms
    //No Pin Toggles, CTC Mode
    TCCR1A = (0<<COM1A1)|(1<<COM1A0)|(0<<COM1B1)|(0<<COM1B0)|(0<<WGM11)|(0<<WGM01);
    //CTC Mode, N=64
    TCCR1B = (0<<ICNC1)|(0<<ICES1)|(0<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(1<<CS10);
    //Enable Channel A Match Interrupt
    TIMSK1 = (0<<ICIE1)|(0<<OCIE1B)|(1<<OCIE1A)|(0<<TOIE1);
    //Setup output compare for channel A
    OCR1A = 24999; //OCR1A = 16Mhz/N*Deltat-1 = 16Mhz/64*.1-1
	//OCR1B = 6249; // 0.1s * (16 MHz / 256) - 1
	
    
    //Configure TWI module (no interrupts)
    //Configure Bit Rate (TWBR)
    TWBR = 50;
    //Configure TWI Interface (TWCR)
    //TWINT: 0 - Interrupt Flag
    //TWEA: 0 - No acknowledge Bit
    //TWSTA: 0 - Start Condition Bit
    //TWSTO: 0 - Stop Condition
    //TWEN: 1 - Enable TWI Interface
    //TWIE: 0 - Disable Interrupt
    TWCR = (0<<TWINT)|(0<<TWEA)|(0<<TWSTO)|(0<<TWWC)|(1<<TWEN)|(1<<TWIE);
    //Configure TWI Status Register (TWSR)
    //TWS7-TWS3: 00000 - Don't change status
    //TWPS1-TWPS0: 10 - Prescaler Value = 64
    TWSR = (0<<TWS7)|(0<<TWS6)|(0<<TWS5)|(0<<TWS4)|(0<<TWS3)|(1<<TWPS1)|(0<<TWPS0);

	//Initialize Display
	display_state=17;//Set state machine to initialization section
	UpdateTWIDisplayState();//Run state machine
	
    sei();


    /* Replace with your application code */
    while (1) 
    {
        //Toggle Pin
		PIND = (1<<PIND0);
		
		tenths = time; // this gets us tenths of seconds
		seconds = tenths / 10; // this gets us seconds
		seconds1 = seconds % 60;
		mins = seconds / 60; // this gets us minutes
		
		// we can use 8 bit unsigned ints because we're not dealing with as large of numbers, so it can save space
		minutes_tens_digit = mins / 10; // the tens place of the minutes component
		minutes_ones_digit = mins % 10; // the ones place of the minutes component
		seconds_tens_digit = seconds1 / 10; // the tens place of the seconds component
		seconds_ones_digit = seconds1 % 10; // the ones place of the seconds component
		tenths_digit = tenths % 10; // tenths place
		
		UpdateValues();
    }
}


ISR(TIMER1_COMPA_vect)
{
    //Start New LCD Screen Update
	display_state = 0;
	if(display_state==0) {
		UpdateTWIDisplayState();
	}
	time +=1;
}

void UpdateValues()
{
	FirstLineStr[8] = 0b00110000 | ( tenths_digit & 0b00001111);
	SecondLineStr[9] = 0b00110000 | ( seconds_tens_digit & 0b00001111);
	SecondLineStr[10] = 0b00110000 | ( seconds_ones_digit & 0b00001111);
	ThirdLineStr[9] = 0b00110000 | ( minutes_tens_digit & 0b00001111);
	ThirdLineStr[10] = 0b00110000 | ( minutes_ones_digit & 0b00001111);
}


/************************************************************************/
/* Display Functions                                                    */
/************************************************************************/



void DummyLoop(uint16_t count)
{
    //Each index1 loop takes approx 100us 
    for ( uint16_t index1=0; index1<=count; index1++ ) {
        //Each index2 loop takes .5us (200 loops = 100us)
        for ( uint16_t index2=0; index2<=200; index2++ ){
            //Do nothing
        }
    }
}


void UpdateTWIDisplayState()
{
	switch(display_state) {
		case 0: //Start of a new display update
			TWCR = TWCR_START;  //send start condition
			display_state++;
			break;
		case 1: //Send address
			TWDR = 0x78; //Set TWI slave address
			TWCR = TWCR_SEND; //Send address
			display_state++;
			break;
		case 2: //Send Control Byte (Instruction: 0x80)
			TWDR = 0x80; //Instruction 0x80
			TWCR = TWCR_SEND; //Set TWINT to send data
			display_state++;
			break;
		case 3: //Send Instruction Byte (Set DDRAM Address: 0x80)
			TWDR = 0x80; //Set DDRAM Address: 0x80
			TWCR = TWCR_SEND; //Set TWINT to send data
			display_state++;
			break;
		case 4: //Send Control Byte (DDRAM Data: 0x40)
			TWDR = 0x40; //DDRAM Data: 0x40
			TWCR = TWCR_SEND; //Set TWINT to send data
			display_state++;
			break;
		case 5: //Send First Line Character
			TWDR = FirstLineStr[TWI_char_index];
			if(TWI_char_index<19) //If not last character
			{
				TWI_char_index++; //Increment index
			}
			else //If last character
			{
				TWI_char_index = 0; //Reset index for next line
				display_state++; //Move to next line state
			}
			TWCR = TWCR_SEND; //Set TWINT to send data
			break;
		case 6: //Send Third Line Character
			TWDR = ThirdLineStr[TWI_char_index];
			if(TWI_char_index<19) //If not last character
			{
				TWI_char_index++; //Increment index
			}
			else //If last character
			{
				TWI_char_index = 0; //Reset index for next line
				display_state++; //Send stop signal
			}
			TWCR = TWCR_SEND; //Set TWINT to send data
			break;
		case 7: //Send repeated start to reset display
			TWCR = TWCR_RSTART;
			display_state++;
			break;
		case 8: //Send address
			TWDR = 0x78; //Set TWI slave address
			TWCR = TWCR_SEND; //Send address
			display_state++;
			break;
		case 9: //Send Control Byte (Instruction: 0x80)
			TWDR = 0x80; //Instruction 0x80
			TWCR = TWCR_SEND; //Set TWINT to send data
			display_state++;
			break;
		case 10: //Send Instruction Byte (Set DDRAM Address: 0x80)
			TWDR = 0xC0; //Set DDRAM Address: 0xC0
			TWCR = TWCR_SEND; //Set TWINT to send data
			display_state++;
			break;
		case 11: //Send Control byte for DDRAM data
			TWDR = 0x40;
			TWCR = TWCR_SEND; //Set TWINT to send data
			TWI_char_index =0;
			display_state++;
			break;
		case 12: //Send Second Line Characters
			TWDR = SecondLineStr[TWI_char_index];
			if(TWI_char_index<19) //If not last character
			{
				TWI_char_index++; //Increment index
			}
			else //If last character
			{
				TWI_char_index = 0; //Reset index for next line
				display_state++; //Send stop signal
			}
			TWCR = TWCR_SEND; //Set TWINT to send data
			break;					
		case 13: //Send Fourth Line Characters
			TWDR = FourthLineStr[TWI_char_index];
			if(TWI_char_index<19) //If not last character
			{
				TWI_char_index++; //Increment index
			}
			else //If last character
			{
				TWI_char_index = 0; //Reset index for next line
				display_state++; //Send stop signal
			}
			TWCR = TWCR_SEND; //Set TWINT to send data
			break;
		case 14: //Create Stop Condition
			TWCR = TWCR_STOP;//finish transaction
			display_state =0;
			break;
		/************************************************************************/
		/* Initialization States                                                */
		/************************************************************************/	
		case 17: //Initialize Step One
			DummyLoop(400);//Wait 40ms for powerup

			TWCR = TWCR_START;
			display_state++;
			break;
		case 18:
			TWDR = 0x78; //Set TWI slave address
			TWCR = TWCR_SEND; //Send address
			display_state++;
			break;
		case 19: //Send control byte for instruction
			TWDR = 0x80; //Instruction 0x80
			TWCR = TWCR_SEND; //Set TWINT to send data
			display_state++;
			break;	
		case 20: //Set Function Mode
			TWDR = 0x38; 
			TWCR = TWCR_SEND; //Set TWINT to send data
			display_state++;
			break;
		case 21: //Send Stop and Wait
			TWCR = TWCR_STOP;//finish transaction
			DummyLoop(1);//Delay 100us
			TWCR = TWCR_START;//Start next transaction
			display_state++;
			break;
		case 22: //Send Slave Address
			TWDR = 0x78; //Set TWI slave address
			TWCR = TWCR_SEND; //Send address
			display_state++;
			break;
		case 23: //Send control byte for instruction
			TWDR = 0x80; //Instruction 0x80
			TWCR = TWCR_SEND; //Set TWINT to send data
			display_state++;
			break;
		case 24: //Turn on Display, Cursor, and Blink
			TWDR = 0x0C; //Instruction 0x80
			TWCR = TWCR_SEND; //Set TWINT to send data
			display_state++;
			break;
		case 25: //Send Stop and Wait
			TWCR = TWCR_STOP;//finish transaction
			DummyLoop(1);//Delay 100us
			TWCR = TWCR_START;//Start next transaction
			display_state++;
			break;
		case 26: //Send Slave Address
			TWDR = 0x78; //Set TWI slave address
			TWCR = TWCR_SEND; //Send address
			display_state++;
			break;
		case 27: //Send control byte for instruction
			TWDR = 0x80; //Instruction 0x80
			TWCR = TWCR_SEND; //Set TWINT to send data
			display_state++;
			break;
		case 28: //Clear Display
			TWDR = 0x01; //Instruction 0x01
			TWCR = TWCR_SEND; //Set TWINT to send data
			display_state++;
			break;
		case 29: //Send Stop and Wait
			TWCR = TWCR_STOP;//finish transaction
			DummyLoop(100);//Delay 10ms
			TWCR = TWCR_START;//Start next transaction
			display_state++;
			break;
		case 30: //Send Slave Address
			TWDR = 0x78; //Set TWI slave address
			TWCR = TWCR_SEND; //Send address
			display_state++;
			break;
		case 31: //Send control byte for instruction
			TWDR = 0x80; //Instruction 0x80
			TWCR = TWCR_SEND; //Set TWINT to send data
			display_state++;
			break;
		case 32: //Clear Display
			TWDR = 0x01; //Instruction 0x01
			TWCR = TWCR_SEND; //Set TWINT to send data
			display_state++;
			break;
		case 33: //Send Stop and Wait
			TWCR = TWCR_STOP;//finish transaction
			DummyLoop(1);//Delay 100us
			display_state=0;
			break;					
		default:
			display_state = 0;
	}

}

/************************************************************************/
/* Define TWI Functions                                                 */
/************************************************************************/

ISR(TWI_vect)
{
	PIND = (1<<PIND2);
	//Read status register and mask out prescaler bits
	uint8_t status = TWSR & 0xF8;

	//Switch based on status of TWI interface
	switch(status)
	{
		case 0x08: //Start Condition Transmitted
			UpdateTWIDisplayState();
			break;
		case 0x10: //Repeat Start Condition Transmitted
			UpdateTWIDisplayState();		
			break;
		case 0x18: //SLA+W transmitted and ACK received
			UpdateTWIDisplayState();
			break;
		case 0x20: //SLA+W transmitted and ACK NOT received
			//This is an error, do something application specific
			break;
		case 0x28: //Data byte transmitted and ACK received
			UpdateTWIDisplayState();
			break;
		case 0x30: //Data byte transmitted and ACK NOT received
			//This is an error, do something application specific
			break;
	}
	PIND = (1<<PIND2);
}




