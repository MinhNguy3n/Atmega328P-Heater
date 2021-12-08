/*
 * main.c
 *
 * Created: 07/04/2021 12:40:17
 * Author : Minh Nguyen
 */ 

#include "Heater.h"
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <math.h>
#include <string.h>

// buffers for storing incoming data (belong to the UART)
uint8_t in_buffer[BUFFER_SIZE];

/* Initialize USART */
void USART0_Init(void)
{
	
	UBRR0H = (UBRR_VALUE>>8);							// Configure USART0 Baud Rate
	UBRR0L = UBRR_VALUE;
	
	UCSR0C = (0<<UCSZ02)|(1<<UCSZ01)|(1<<UCSZ00)|		// Set frame format: 8 data, 1 stop bits, no parity
			(0<<USBS0)|(0<<UPM01)|(0<<UPM00);
	
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);						// Enable receiver and trasmmit complete interrupt
	PRR &= ~(1<<PRUSART0);
}


// initialize I/O Pins and external interrupts for SW event
void initPinsInterrupt(void)
{
	DDRB = (1<<PORTB1); 				// Set PB1/OC1A as output pin for PWM mode timer function, to control heating circuit
	DDRD = 0x78;						// PB0 and PD2 as input toggle Local/External and ON/OFF respectively
										// PD3..5 outputs that control LEDs
	DDRD &= ~(1<<DDD2);					// PD2 as input (also for INT0)
	PORTD |= (1<<PORTD2);				// Enable weak pull-up on pin PB0 and PD2
	PORTB |= (1<<PORTB0) | (1<<PORTB1);
	 
    PCICR  = (1<<PCIE0);				// Enable Pin Change interrupt for pin PCINT0..7
	PCMSK0 |= (1<<PCINT0);				// Enable PCINT0 (pin PB0)
}

// initialize ADC function
void initADC(void)
{						
	DIDR0 = ADC_RC_CHANNEL | ADC_POTENTIOMETER_CHANNEL;		// Disable digital input for pin ADC0 and ADC1 (PC0 and PC1)
	
	ADMUX = (0<<REFS1) | (1<<REFS0);						// AVCC with external capacitor at AREF as ref voltage
	ADMUX &= ~(1<<ADLAR);									// Right-adjusted ADC result
	
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);			// sampling freq 125kHz (Prescaler = 16MHz/150kHz = 128)
															// SAR circuit requires clock frequency between 50..200kHz
															// to get maximum resolution (10-bit)
	ACSR &= ~(1 << ACD);									// Allow Ananlog comparator
	ADCSRA |= (1<<ADEN);									// Enable ADC
	PRR &= ~(1<<PRADC);										// turn off ADC power reduction 
}

volatile uint16_t _millis;

// Timer0 OVF timer delay in millisecond
void initTimer0(uint16_t millis)
{
	TCNT0 = 0;												// reset Timer 0 counter
	TCCR0B |= (1<< CS00);									// no prescaling clkIO/1
	TIMSK0 |= (1<<TOIE0);									// Timer overflow interrupt
	PRR &= ~(1<<PRTIM0);									// turn off Timer/counter 0 power reduction
	_millis = millis;
}

// PWM Phase-correct mode configuration with Timer 1
void initTimer1PWM()
{
	TCNT1 = 0;												// reset Timer 1 counter
	TCCR1A |= (1<<COM1A1) | (1<<WGM11);						// Clear OCA1 on compare match, and Set OCA1 at the Bottom;
	TCCR1B |= (1<<WGM13) | (1<<WGM12) | (1<<CS10);			// Fast PWM mode 14 (WGM13..10 = b1110) and clkIO/1 (non-Prescaler)
	
	ICR1 = TIMER1_TOP;										// TOP value is calculated with the PWM frequency of 10kHz
	PRR &= ~(1<<PRTIM1);									// turn off Timer/Counter 1 power reduction
}

volatile uint8_t sleep;
volatile uint16_t timer0_ovf_cnt;
volatile uint8_t print_temp;

/* Interrupt Service routine for Pin Change interrupt vect 0, which is triggered whenever
pin PB0 detect a rising edge, thus toggles system ON/OFF. When SW is OFF, the system
stays in sleep mode and idle until it is switch ON again
*/

ISR(PCINT0_vect){											// wake-up controller on Pin PD2 detect Pin-Change interrupt
	if(ON_BTN_PUSHED){
		sleep ^= 0x01;										// Toggle sleep mode
	}
} 

// Re-enable ADC features
ISR(TIMER0_OVF_vect){
	if(timer0_ovf_cnt++ >= TIMER0_OVERFLOW_PER_MILLIS*_millis){		// Overflow counter per millisec is computed from the duration of TimerOVF event
		print_temp = 0x01;											// Allow sending RC-circuit and Pot reading via USART message
		timer0_ovf_cnt = 0;
	}
}

void disableTimer1(void)
{
	PRR |= (1<<PRTIM1);						// Power Reduction for Timer/Counter1
}

void fallasleep(void) 
{
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);	// set the power-down sleep-mode
	cli();									// disable interrupts
	sleep_enable();							// set SE-bit
	sei();									// enable interrupts
	sleep_cpu();							// SLEEP-instruction 
	sleep_disable();						// reset SE-bit
}

uint8_t USART0_Receive(void)
{
	while ( !(UCSR0A & (1<<RXC0)) ){		// Wait for data to be received
		if(LOCAL_SW_CLOSED || ON_BTN_PUSHED) return NULL_CHAR;			// break RX polling if switches state has changed
	}
	
	return UDR0;							// return received data from buffer
}

void USART0_Transmit(uint8_t byte)
{
	while(!(UCSR0A & (1<<UDRE0)));			// Wait for empty transmit buffer
	UDR0 = byte;							// Put data into buffer
	while (!(UCSR0A & (1 << TXC0)));		//Wait for transmit complete
	UCSR0A |= (1<<TXC0);					//clear TXC flag
}


/* Measure Analog channel input using internal ADC */
/* With 5.5V VCC as reference, and 10b resolution, */
/* The full scale is 1023, 0V = 0, 1023 = Vcc*/
uint16_t readADC(uint8_t chan_number){
	if (chan_number < 0 || chan_number > 7) return 0;	// non-existing channel -> return null
	ADMUX &= (1 << REFS0) | (1 << ADLAR);				// set Reference REFS0:1 and ADLAR to retain ADC configuration 
	ADMUX  |= chan_number;								// set Analog Channel selection bit to open channel 
	ADCSRA |= (1<<ADSC); 								// start conversion
	
	while(ADCSRA & (1<<ADSC));							// wait until conversion is ready by polling until ADSC = 0

	return ADC;											// return 10b ADC the result
}

// convert to ASCII Digit, which starts from U+48 to U+57
uint8_t convertToASCII(uint8_t ch){
	return (ch+DIGIT_0_CHAR);
}

uint8_t ConvertFromASCII(uint8_t ch){
	return (ch-DIGIT_0_CHAR);
}

// send string (i.e. character array) via USART0
void sendUSART(const char * string, uint8_t len){
	uint8_t index = 0;
	while(index < len){
		USART0_Transmit(string[index++]);
	}
}

/*************************************************************************/
/* main loop                                                             */
/* There exists two main interrupt handling operations, namely,			 */
/* External (Pin Change) Interrupt and Timer0 overflow, in addition to   */
/* ADC, USART0 RX/TX and Timer/Counter 1 which utilize hardware PWM      */
/* Once the main program detect sleep's variable is 0 (toggled during Pin*/
/* -Change interrupt 0 ISR), all above functions, except for Pin-Change  */
/* interrupt will be disabled during power-down sleep mode				 */
 
/* While awake, if the local mode is activated, the main program will	 */
/* continuously read from the ADC channels associated with hardware pins */
/* that are connected with Potentiometer and the RC Heating circuit,	 */
/* Thus, control the duty cycle of PWM TimerCounter 1, i.e. analog output.*/
/* External mode, on the other hand, allows temperature control	via USART*/
/* communication. During local mode, the readings of RC circuit's output */
/* and Potentiometers are also transmitted via USART*/
/************************************************************************/

int main(void)
{	
	initPinsInterrupt();		// initialize Pin Change Interrupts as well as pin directions
	initTimer0(1000);			// Timer0 Overflow for 1000ms delay
	initTimer1PWM();			// Timer1 Fast PWM mode
	initADC();					// ADC function with ADC0 (RC) and ADC1(Pot)
	USART0_Init();				// USART0 at 9600 baud rate and (8,1,N) frame
	
	sei();						// enable interrupts
	sleep = 0x00;
	
	float temperature;
	uint16_t temp_rounded = 0;
	uint16_t heat_temp = 0;
	int16_t temp_diff;
	uint8_t char_in;
	uint8_t char_cnt = 0;
    while (1) 
    {
		if (sleep & 0x01)		// if sleep bit 0 is flipped to 1 (during PCINT2 ISR)
		{
			RED_OFF;			// Turn off all LEDs
			BLUE_OFF;
			GREEN_OFF;
			fallasleep();		// Power-down sleep mode
		}
		else{ 
			/************************************************************************/
			/*								Local mode								*/
			/************************************************************************/
			if(LOCAL_SW_CLOSED) 
			{
				temp_rounded = readADC(ADC_POTENTIOMETER_CHANNEL);						// read voltage value from 1k pot
				temperature = temp_rounded* 100.0 / (float)(MAX_ADC_RANGE);				// convert ADC value to percentage scale
				temp_rounded = (uint16_t) (temperature + 0.5);							// and round up the integer value
			}
			/************************************************************************/
			/*							  External mode								*/
			/************************************************************************/
			else {	
				if(char_cnt == 0){
					sendUSART(PendingMsg, strlen(PendingMsg));
				}
				char_in = USART0_Receive();												// Start reading from USART0
				if(char_in >= DIGIT_0_CHAR && char_in <= DIGIT_9_CHAR){						// ignore all characters except for digit characters
					if(char_cnt < 3){													// write 3 input characters into buffer 
						in_buffer[char_cnt++] = ConvertFromASCII(char_in);				// and convert collected digit characters into decimal value														
						USART0_Transmit(char_in);										// print received character back to USART
					}
				}
				else if (char_in == LINE_BREAK ){										// Stop reading once return carriage is reached
					uint8_t mult_ten = 1;
					temp_rounded = 0;													// reset temp_rounded for the next record
					while(char_cnt--){													// examine all decimals stored in in_buffer
						temp_rounded += in_buffer[char_cnt] * mult_ten;					// in which case decimals are arranged in descending order of value 
						mult_ten *= 10;													// contribution thus, i.e. consecutive decimal's value is ten fold the previous
					}																
					if(temp_rounded > 100){
						USART0_Transmit(LINE_BREAK);
						sendUSART(ValErrorMsg, strlen(ValErrorMsg));					// "Value out of range" message
						USART0_Transmit(LINE_BREAK);
					}else{
						USART0_Transmit(LINE_BREAK);
						sendUSART(ValidMsg, strlen(ValidMsg));							// "Value OK" message
						USART0_Transmit(LINE_BREAK);
					}
					char_in = 0;														// reset char_in and char_cnt
					char_cnt = 0;
				}
				else if (char_in != NULL_CHAR){											// If received character is neither null nor digit
					USART0_Transmit(LINE_BREAK);
					sendUSART(DigitErrorMsg, strlen(DigitErrorMsg));					// "Character is unrecognized" message
					USART0_Transmit(LINE_BREAK);
					char_in = 0;
					char_cnt = 0;
				}
			}
			/************************************************************************/
			/*				Heating circuit measurement and control					*/
			/************************************************************************/
			heat_temp = readADC(ADC_RC_CHANNEL);
			temperature = heat_temp* 100.0 / (float)(MAX_ADC_RANGE);
			heat_temp = (uint16_t) (temperature + 0.5);

			if(print_temp){															// Print RC circuit reading 
				if(LOCAL_SW_CLOSED){												// while in Internal mode
					sendUSART(PotReadingMsg, strlen(PotReadingMsg));					// send "Pot reading" message
					USART0_Transmit(convertToASCII(temp_rounded/10));					// send the upper decimal
					USART0_Transmit(convertToASCII(temp_rounded  - temp_rounded/10*10));// and the lower decimal
					USART0_Transmit(LINE_BREAK);
					sendUSART(RCReadingMsg, strlen(RCReadingMsg));					// send RC-circuit reading message
					USART0_Transmit(convertToASCII(heat_temp/10));					// send the upper decimal 
					USART0_Transmit(convertToASCII(heat_temp  - heat_temp/10*10));	// and the lower byte
					USART0_Transmit(LINE_BREAK);
				}
				print_temp = 0;
			}
			
			// compare ADC value of heating element with the desired temperature
			// if the measured difference between the RC and the requested value is off over 2 decimal values	
			temp_diff = temp_rounded - heat_temp;
			if(temp_diff > 2){														// RC reading < desired temp => warming up 		
				RED_ON;
				BLUE_OFF;
				GREEN_OFF;
				
				PRR &= ~(1<<PRTIM1);												// re-enable Timer1
				SET_DUTY_CYCLE(temp_rounded);										// update PWM duty cycle with the desired value
			}
			else if(temp_diff < -2){												// RC reading > desired temp => cooling down
				temp_diff *= -1;
				RED_OFF;
				BLUE_ON;
				GREEN_OFF;
				
				PRR &= ~(1<<PRTIM1);												
				SET_DUTY_CYCLE(temp_rounded);										
			}
			else{																	// difference is within 2 decimals => temperature OK
				RED_OFF;
				BLUE_OFF;
				GREEN_ON;
				disableTimer1();													// disable PWM function to reduce power consumption
			}
		}
    }
}
