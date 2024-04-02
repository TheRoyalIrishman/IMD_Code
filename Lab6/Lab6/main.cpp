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
void ReadButton();
uint8_t PID(uint8_t, uint8_t);
void SetMotorSpeed(signed int, signed int);
void oscillationCheck(int, int);

//Global variables for display strings
volatile uint8_t FirstLineStr[21] =  "S1:XXXX             ";
volatile uint8_t SecondLineStr[21] = "M1:XXX D1:X         ";
volatile uint8_t ThirdLineStr[21] =  "Button: XXX         ";
volatile uint8_t FourthLineStr[21] = "                    ";

//Global variables for TWI state and data
uint8_t display_state =0;
uint8_t TWI_data = 0;
uint8_t TWI_slave_address = 0;
uint8_t TWI_char_index = 0;

volatile uint16_t time = 0;
volatile uint8_t isrHalfSecondCount = 0;
volatile uint8_t ButtonPressed[4] = "OOM"; // Button will later be "ON " or "OFF"
uint8_t ON[4] = "ON "; // used since I can't change ButtonPressed char by char...
uint8_t OFF[4] = "OFF";

volatile uint16_t adcValueOutputZero = 0; // MUX 0
volatile uint16_t adcValueOutputOne = 0; // MUX 1
volatile uint16_t adcValueOutputTwo = 0; // MUX 2
volatile uint16_t adcValueOutputThree = 0; // MUX 3
volatile uint16_t adcValueOutputFour = 0; // MUX 4
volatile uint16_t adcValueOutputFive = 0; // MUX 5

volatile uint8_t readADCL = 0;
volatile uint8_t readADCH = 0;

uint8_t direction[2] = "A";

volatile uint8_t previousError = 0;
volatile uint8_t proportionalOffset = 0;
volatile uint8_t integralTerm = 0;
volatile uint8_t derivativeTerm = 0;

volatile uint8_t errorTerm = 0;
volatile uint8_t outputValue = 0;

const volatile uint8_t Kp = 1.0;
const volatile uint8_t Ki = 0.0;
const volatile uint8_t Kd = 0.0;
const volatile uint8_t dt = 1.0;

volatile uint8_t robotSpeed = 0;
volatile uint8_t leftMotorSpeed = 0;
volatile uint8_t rightMotorSpeed = 0;

int main(void)
{
    //Configure I/O
    //Set SDA (PC4) and SCL (PC5) as outputs for TWI bus
    DDRC = (1<<PORTC0)|(1<<PORTC1)|(1<<PORTC2)|(1<<PORTC3)|(1<<PORTC5)|(1<<PORTC4);
    DDRD = (1<<PORTD0)|(0<<PORTD1)|(0<<PORTD2)|(0<<PORTD3)|(1<<PORTD4)|(1<<PORTD5)|(1<<PORTD6)|(1<<PORTD7);
	DDRB = (1<<PORTB1)|(1<<PORTB2);

	
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
	
	//**************************************************************************************
	//Configure Timer 1
	//**************************************************************************************
	//Setup Compare Output Mode for Channel A and Wave Generation Mode
	/*TCCR1A = (0<<COM1A1)|(1<<COM1A0)|(0<<COM1B1)|(0<<COM1B0)|(0<<WGM11)|(0<<WGM01);     //No pin Toggles, CTC Mode
	TCCR1B = (0<<ICNC1)|(0<<ICES1)|(0<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(1<<CS10); //Define Clock Source Prescaler, N=64
	OCR1A = 24999; //Set Output Compare A to match every 24999 clocks (24999x8uS =100mS) ---OCR1A = 16Mhz/N*Deltat-1 = 16Mhz/64*.1-1
	TIMSK1 = (0<<ICIE1)|(0<<OCIE1B)|(1<<OCIE1A)|(0<<TOIE1); //Enable Interrupt for Output Compare A Match
*/
	//**************************************************************************************
	//Configure Timer 0
	//**************************************************************************************
	TCCR0A = (1 << COM0A1) | (0 << COM0A0) | (1 << COM0B1) | (0 << COM0B0) | (1 << WGM01) | (1 << WGM00);//Setup Fast PWM Mode for Channel A
	TCCR0B = (0 << WGM02) | (0 << CS02) | (1 << CS01) | (0 << CS00); //Define Clock Source Prescaler
	OCR0A = 1; //Initialized PWM Duty Cycle
	OCR0B = 1; //
	
	//**************************************************************************************
	//Configure A/D
	//**************************************************************************************
	//Select Analog Port 0 and Internal Reference Voltage;
	ADMUX = (0 << REFS1) | (1 << REFS0) | (1 << ADLAR) | (0 << MUX3) | (0 << MUX2) | (0 << MUX1) | (0 << MUX0);
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0); //Enable A/D, Enable Interrupt, Set A/D Prescalar
	DIDR0 = (1 << ADC0D); //Disable Input Buffers
	ADCSRA |= (1 << ADSC); //Start Conversion

	//Initialize Display
	display_state=17;//Set state machine to initialization section
	UpdateTWIDisplayState();//Run state machine
	
    sei();

    while (1) 
    {
		//ReadButton();
		UpdateValues();
    }
}

void ReadButton(){
	if(PINB & 0b00001000){
		PORTD = (PORTD | (1<<PORTD0)) & ~(1<<PORTD1);
		direction[0] = 0b00110000 | 0; //Clockwise		
		ButtonPressed[0] = OFF[0];
		ButtonPressed[1] = OFF[1];
		ButtonPressed[2] = OFF[2];
	}
	else {
		PORTD = (PORTD & ~(1<<PORTD0)) | (1<<PORTD1);
		direction[0] = 0b00110000 | 1; //Counterclockwise
		ButtonPressed[0] = ON[0];
		ButtonPressed[1] = ON[1];
		ButtonPressed[2] = ON[2];
		//OC0A PD6
		//OC0B PD7
	}
}

uint8_t PID(uint8_t leftWheelValue, uint8_t rightWheelValue) {
	/*if (integrationTimeChecker >= 20) { // >= 20 is equal to 2 seconds
		integrationTimeChecker = 0; // will stop the oscillation error
		integralTerm = 0;
	}*/
	
	errorTerm = leftWheelValue - rightWheelValue;
	proportionalOffset = Kp * errorTerm;
	
	
	
	//integralTerm = Ki * (integralTerm + errorTerm * dt);
	//derivativeTerm = Kd * (errorTerm - previousError) / dt;
	previousError = errorTerm;
	outputValue = proportionalOffset;
	
	leftMotorSpeed = robotSpeed + proportionalOffset; // once we get to integrating integral and derivative, this will be robotSpeed +/- outputValue
	rightMotorSpeed = robotSpeed - proportionalOffset;
	
	SetMotorSpeed(leftMotorSpeed, rightMotorSpeed);
	
	return outputValue;
}

void SetMotorSpeed(signed int leftMotorSpeed, signed int rightMotorSpeed) {
	OCR0A = leftMotorSpeed;
	OCR0B = rightMotorSpeed;
}

void oscillationCheck(int previousTurnDirection, int currentTurnDirection) {
	unsigned int oscillationCounter = 0;
	
	// right turn translates to 0x01
	// left turn translates to 0x02
	
	// checks if there was a left-to-right or right-to-left turn
	if ((previousTurnDirection | currentTurnDirection) & 0x03) {
		// increment number of oscillations
		oscillationCounter++;
	} else {
		// reset number of oscillations
		oscillationCounter = 0;
	}
	
	if (oscillationCounter > 20) {
		// implement escape routine
	}
}

void UpdateValues()
{
	FirstLineStr[3] = 0b00110000 | (adcValueOutputZero / 1000);
	FirstLineStr[4] = 0b00110000 | ((adcValueOutputZero / 100) % 10);
	FirstLineStr[5] = 0b00110000 | ((adcValueOutputZero / 10) % 10);
	FirstLineStr[6] = 0b00110000 | (adcValueOutputZero % 10);
	
	//Display duty cycle in terms of percentage
	SecondLineStr[3] = 0b00110000 | (OCR0A / 255);
	SecondLineStr[4] = 0b00110000 | ((OCR0A / 25) % 10);
	SecondLineStr[5] = 0b00110000 | ((2*(OCR0A % 25) / 5) % 10);
	SecondLineStr[10] = direction[0];
	//Debug by showing OCR0A value
	SecondLineStr[12] = 0b00110000 | (OCR0A / 100); 
	SecondLineStr[13] = 0b00110000 | ((OCR0A / 10) % 10);
	SecondLineStr[14] = 0b00110000 | (OCR0A % 10);
	
	ThirdLineStr[8] = ButtonPressed[0];
	ThirdLineStr[9] = ButtonPressed[1];
	ThirdLineStr[10] = ButtonPressed[2];
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
}

/************************************************************************/
/* Interrupt Routine for A/D Completion                                 */
/************************************************************************/

ISR (ADC_vect) { //Sample every 0.1s	
	
	
	readADCL = ADCL; // reading ADC low register
	readADCH = ADCH; // reading ADC high register
	
	// checks if last bit in ADMUX is set - go into MUX 0
	//---------------------------------  MUX 0  ---------------------------------//
	if (1){//(ADMUX & 0b00001111) == 0) {
		adcValueOutputZero = readADCH;
		adcValueOutputZero = (adcValueOutputZero << 2) | (readADCL >> 6);
		OCR0A = readADCH; //Load A/D High Register in OCR0A to set PWM Duty Cycle
		//OCR0A += 5; //used for lab5A to slowly increment OCR0A
		//ADMUX = ((ADMUX & 0b11110000) | 0b00000001);          //Sets to MUX1 //Maybe, cleaner
		//ADMUX = ADMUX | ((ADMUX & 0b11110000) | 0b00000000)  //Maybe too repetitive
		//ADMUX = ADMUX | (1 << MUX0);                       // switch sensor reading, true previous way
		PORTB ^= 0x04;//Toggle Pin PB2
	}
	//---------------------------------  MUX 1  ---------------------------------//
	/*else if ((ADMUX & 0b00001111) == 1) {
		adcValueOutputOne = readADCH;
		adcValueOutputOne = (adcValueOutputOne << 2) | (readADCL >> 6);
		//OCR0B = readADCH;
		ADMUX = ((ADMUX & 0b11110000) | 0b00000010);          //Sets to MUX2
		//ADMUX = ADMUX & 0b11111110; // switch sensor reading, true previous way
	}
	//---------------------------------  MUX 2  ---------------------------------//
	else if ((ADMUX & 0b00001111) == 2) {
		adcValueOutputTwo = readADCH;
		adcValueOutputTwo = (adcValueOutputTwo << 2) | (readADCL >> 6);
		ADMUX = ((ADMUX & 0b11110000) | 0b00000011);          //Sets to MUX3
	}
	//---------------------------------  MUX 3  ---------------------------------//
	else if ((ADMUX & 0b00001111) == 3) {
		adcValueOutputThree = readADCH;
		adcValueOutputThree = (adcValueOutputThree << 2) | (readADCL >> 6);
		ADMUX = ((ADMUX & 0b11110000) | 0b00000100);          //Sets to MUX4
	}
	//---------------------------------  MUX 4  ---------------------------------//
	else if ((ADMUX & 0b00001111) == 4) {
		adcValueOutputFour = readADCH;
		adcValueOutputFour = (adcValueOutputFour << 2) | (readADCL >> 6);
		ADMUX = ((ADMUX & 0b11110000) | 0b00000101);          //Sets to MUX5
	}
	//---------------------------------  MUX 5  ---------------------------------//
	else if ((ADMUX & 0b00001111) == 5) {
		adcValueOutputFive = readADCH;
		adcValueOutputFive = (adcValueOutputFive << 2) | (readADCL >> 6);
		ADMUX = ((ADMUX & 0b11110000) | 0b00000000);          //Sets to MUX0
	}
	else{
		//PIND = PIND | (1<<PIND5); //Put LED on here to check error
	}*/
}

ISR(TIMER1_COMPA_vect)
{
	//Start New LCD Screen Update
	//display_state = 0;
	isrHalfSecondCount += 1;
	
	if (isrHalfSecondCount > 4) {
		isrHalfSecondCount = 0;
		UpdateTWIDisplayState();
		if(display_state==0) {
			UpdateTWIDisplayState();
		}
	}
	
	time +=1;
	
	PORTB ^= 0x01; //Toggle Pin PD0 - this is a breadcrumb to say "did we get into ISR"  //red led
	ADCSRA |= (1 << ADSC); //Start Conversion
}

ISR(TIMER1_OVF_vect) {
	unsigned int backupCounter = 0;
	
	unsigned int threshold = 10; // this is going to be replaced with a more well defined constant later
	
	unsigned int backSensorOutput = 0; // this is going to be replaced with the sensor output
	unsigned int leftSensorOutput = 0; // this is going to be replaced with the sensor output
	unsigned int rightSensorOutput = 0; // this is going to be replaced with the sensor output
	
	// are we currently backing up the robot?
	if (backupCounter > 0) {
		// is there anything behind us?
		if (backSensorOutput < threshold) {
			SetMotorSpeed(50, 50); // back up robot
		} else {
			SetMotorSpeed(200, 50); // go right
			backupCounter--;
		}
	} else {
		// is there anything to the left us us?
		if (leftSensorOutput < threshold) {
			SetMotorSpeed(200, 50); // go right
		} else if (rightSensorOutput < threshold) { // is there anything to the right of us?
			SetMotorSpeed(50, 200); // go left
		} else if (backSensorOutput < threshold) { // is there anything behind us?
			SetMotorSpeed(50, 50); // go backwards
			backupCounter = 50; // run robot backwards for 50 ticks
		} else {
			SetMotorSpeed(200, 200); // go forward
		}
	}
}