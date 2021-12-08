/*
 * Heater.h
 *
 * Created: 21/04/2021 07:48:28
 *  Author: Minh Nguyen
 */ 


#ifndef HEATER_H_
#define HEATER_H_

#define LOCAL_SW_CLOSED	(PIND & (1<<PIND2))		// toggle switch closed state when PIND2 is HIGH
#define ON_BTN_PUSHED   !(PINB & (1<<PINB0))	// tactile button closed state when PINB0 is LOW

#define TEMP_ON		PORTB |= (1<<PORTB1)		// Turn on heating circuit
#define TEMP_OFF	PORTB &= ~(1<<PORTB1)		// Turn off heating circuit

#define ADC_RC_CHANNEL				0
#define ADC_POTENTIOMETER_CHANNEL	1

#define RED_ON		PORTD |= (1<<PORTD3)	// Cathode LED will lit up when Digital Pin goes HIGH, and turns off otherwise	
#define RED_OFF		PORTD &= ~(1<<PORTD3)		
#define BLUE_ON		PORTD |= (1<<PORTD5)
#define BLUE_OFF	PORTD &= ~(1<<PORTD5)
#define GREEN_ON	PORTD |= (1<<PORTD4)
#define GREEN_OFF	PORTD &= ~(1<<PORTD4)

#define FOSC 16000000	// System Clock Speed, internal clock source (8MHz) or external crystal clock (16MHz)
#define BAUD 9600		// USART Baud rate
#define UBRR_VALUE (FOSC / 16 / BAUD - 1)
#define BUFFER_SIZE 20	

#define MAX_ADC_RANGE 1023
#define TIMER0_OVF_CNT_PER_SEC (FOSC / 65536)	// Count number Timer0 Overflow periods per second, Formula is: FOSC / N / 2^8
											// where N is Prescaler value, one period 8-bit Timer0 Counter Overflow 
#define NULL_CHAR		0x00				// Null character						
#define LINE_BREAK		0x0D				// a.k.a CR (Carriage return) character in ASCII (U+13)
#define DIGIT_0_CHAR		0x30				// '0' character in ASCII (U+48)
#define DIGIT_2_CHAR		0x32				// '2' character in ASCII (U+50)
#define DIGIT_9_CHAR		0x39				// '9' character in ASCII (U+57)	

/* 
The wave frequency of the Fast PWM mode is defined as follow:
 - F_PWM = FOSC / (N*(TOP+1)), where FOSC=16MHz, and N is the prescaler factor, 
 in this example project, prescaler is set to 8
Rearranging the equation to find the formula for TOP, we have:
 - TOP = FOSC / (N*F_PWM) - 1
For the sake of resolution accuracy, we'll use sampling frequency f=10kHz, thus yield the TOP value:
 - TOP = 16MHz/(10kHz*8) - 1 = 199 ~ 200
The duty cycle can be modified by setting OCR1A within the value range 0..200.
Thus, the desired temperature value is scaled from 0-100% range to 0-200 range (e.g. t = 25% -> OCR1A = 50)
*/

#define F_PWM		10000
#define TIMER1_TOP	FOSC/F_PWM - 1
#define SET_DUTY_CYCLE(dc) {OCR1A = dc*TIMER1_TOP/100;}		// set the Pulse train's duty cycle for PWM signal
#define TIMER0_OVERFLOW_PER_MILLIS FOSC/256/1000			// overflow counter per millisecond (non-prescaler)

const char PotReadingMsg [] = "Pot Reading = ";												// During External mode, there would be 
const char RCReadingMsg [] = "RC circuit reading = ";										// message prompts for users to follow
const char DigitErrorMsg [] = "ERR: Unrecognized Digit or Key command";						// status of the program
const char ValErrorMsg [] = "ERR: Value out of range (0-100)";								
const char PendingMsg [] = "Enter desired Temperature (0-100):";							
const char ValidMsg [] = "Value OK";														

#endif /* HEATER_H_ */