/*        _         _             _       _                  _          _            _            _   _            _
*         /\ \      /\ \         /\ \     /\_\               / /\       /\ \         /\ \         /\_\/\_\ _      /\ \
*        /  \ \     \_\ \       /  \ \   / / /         _    / /  \      \_\ \       /  \ \       / / / / //\_\   /  \ \
*       / /\ \ \    /\__ \     / /\ \ \  \ \ \__      /\_\ / / /\ \__   /\__ \     / /\ \ \     /\ \/ \ \/ / /__/ /\ \ \
*      / / /\ \_\  / /_ \ \   / / /\ \ \  \ \___\    / / // / /\ \___\ / /_ \ \   / / /\ \ \   /  \____\__/ //___/ /\ \ \
*     / / /_/ / / / / /\ \ \ / / /  \ \_\  \__  /   / / / \ \ \ \/___// / /\ \ \ / / /  \ \_\ / /\/________/ \___\/ / / /
*    / / /__\/ / / / /  \/_// / /    \/_/  / / /   / / /   \ \ \     / / /  \/_// / /   / / // / /\/_// / /        / / /
*   / / /_____/ / / /      / / /          / / /   / / /_    \ \ \   / / /      / / /   / / // / /    / / /        / / /    _
*  / / /\ \ \  / / /      / / /________  / / /___/ / //_/\__/ / /  / / /      / / /___/ / // / /    / / /         \ \ \__/\_\
* / / /  \ \ \/_/ /      / / /_________\/ / /____\/ / \ \/___/ /  /_/ /      / / /____\/ / \/_/    / / /           \ \___\/ /
* \/_/    \_\/\_\/       \/____________/\/_________/   \_____\/   \_\/       \/_________/          \/_/             \/___/_/
*
*
* Main Project: Bedlights
* Based on Arduino Nano board with Atmega328p chipset
*
* Module: Main.c
*
*/

//Includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>

//Defines
#define ONTIME  10

//Enumerations
enum {CH1, CH2, CH3, CH4}; //ADC Channels Day/Night, Brightness, Duration, LDR
enum {idle, fadein, glow, fadeout}; //Status of light engine state machine

//Global variables
volatile uint8_t duration = 0;
volatile uint8_t brightness = 0;
volatile uint8_t switchingthreshold = 0;
volatile uint8_t tick = 0;
volatile uint8_t statusleft = 0;
volatile uint8_t statusright = 0;
volatile uint8_t statusfront = 0;
volatile uint8_t runleft = 0;
volatile uint8_t runright = 0;
volatile uint8_t runfront = 0;
volatile uint8_t setup = 0;
volatile uint8_t night = 0;

//Function declarations
void init_ports(void);
void init_timers(void);
void init_interrupts(void);
void init_adc(void);
unsigned int adc_conversion(int channel);
_Bool is_night(void);

//Linearisation table for PWM brightness
unsigned int helligkeit[128]={0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 4,
	4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 8, 9, 9, 9, 9, 10,
	10, 10, 10, 10, 11, 11, 11, 11, 12, 31, 32, 34, 35, 36, 38, 39, 41, 42, 44, 45, 47,
	49, 51, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 75, 77, 79, 82, 84, 87, 89, 92,
	94, 97, 100, 103, 105, 108, 111, 114, 117, 120, 124, 127, 130, 133, 137, 140, 144, 147,
	151, 155, 158, 162, 166, 170, 174, 178, 182, 186, 190, 194, 199, 203, 208, 212, 217,
	221, 226, 231, 236, 241, 246, 251, 256
};


// function name: init_ports()
// Overall setup of I/O ports
// 
void init_ports(void){
	//Set pull-ups for inputs active = 1, disabled = 0 (default)
	PORTB = 0b00000000;
	PORTC = 0b00000000;
	PORTD = 0b00000000;

	//Set direction output = 1, input = 0 (default)
	DDRB = 0b11111011; 
	DDRC = 0b11100000;
	DDRD = 0b11100011;
}



// function name: init_timers()
// configuration of timers for PWM output
// and on time duration
void init_timers(void){
	//PWM for LEDs left and right side 
	OCR0A = 0;
	OCR0B = 0;
	TCCR0A = (1<<COM0A1) | (1<<COM0B1) | (1<<WGM01) | (1<<WGM00); //Set Fast PWM non inverting, enable OC0A & OC0B
	TCCR0B = (1<<CS02); //256x Prescaler = 240Hz PWM

	//PWM for LEDs front side
	OCR2A = 0;
	TCCR2A = (1<<COM2A1) | (1<<WGM21) | (1<<WGM20); //Set Fast PWM non inverting, enable OC2A
	TCCR2B = (1<<CS22) | (1<<CS21); //256x Prescaler = 240Hz PWM
	
	//Timer for system tick
	ICR1 = 780;												  //Tick occurs every 100ms = 10Hz 
	TCCR1A = (1<<WGM11);									  //Mode 14, Fast PWM Top at ICR1
	TCCR1B = (1<<WGM12) | (1<<WGM13) | (1<<CS10) | (1<<CS12); //Prescaler 1024
	TIMSK1 = (1<<TOIE1);									  //Enable Timer Interrupt
}


ISR(TIMER1_OVF_vect){
	tick = 1;
}


// function name: init_interrupts()
// enables the internal interrupts for
// PIR Sensor input on Pin PD2, PD3, PB2 and PC3
void init_interrupts(void){
	//Set interrupts
	EICRA = 0b00001111; //Rising edge of INT1 and INT0 to trigger
	EIMSK = 0b00000011; //Enable Interrupts INT1 and INT0
	PCICR = 0b00000111; //Enable all Pin change interrupt groups 
	PCMSK0 = 0b00000100; //Mask out all interrupts beside PCINT2
	PCMSK1 = 0b00010000; //Mask out all interrupts beside PCINT12
	PCMSK2 = 0b00010000; //Mask out all interrupts beside PCINT20
}


ISR(INT0_vect){
	if(statusleft == idle && night){
		statusleft = fadein;
	}
	runleft = duration;
}

ISR(INT1_vect){
	if(statusright == idle && night){
		statusright = fadein;
	}
	runright = duration;
}

ISR(PCINT0_vect){
	if(statusleft == idle && night){
		statusleft = fadein;
	}
	if(statusfront == idle && night){
		statusfront = fadein;
	}
	runleft = duration;
	runfront = duration;
}

ISR(PCINT1_vect){
	if(statusright == idle && night){
		statusright = fadein;
	}
	if(statusfront == idle && night){
		statusfront = fadein;
	}
	runright = duration;
	runfront = duration;
}

ISR(PCINT2_vect){
	setup = 1;
}


// function name: init_adc()
// configuration of ADC to read in values 
// of potentiometers on 3 channels PC0...PC2
void init_adc(void){
	ADMUX = (1<<REFS0) | (1<<ADLAR); //AVcc with external capacitor at AREF pin and justify left
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); //Enable ADC and set prescaler to x128 = 125kHz
	DIDR0 = (1<<ADC2D) | (1<<ADC1D) | (1<<ADC0D);  //Digital Input Disable for ADC0...ADC2
}

// function name: adc_conversion()
// call with ADC channel that shall be read out
// switches MUX register according to channel, 
// waits for conversion to finish and
// returns ADC value at selected channel
// Will temporary stop interrupts
unsigned int adc_conversion(int channel){
	uint8_t temp;
	cli();
	temp = ADMUX & 0xF0;			//read current setting of ADMUX register and mask out lower bits
	temp |= channel & 0x0F;			//overwrite lower bits with channel setting
	ADMUX = temp;					//write combined values back into ADMUX register
	ADCSRA |= (1<<ADSC);			//Start ADC conversion
	while(!(ADCSRA&(1<<ADIF)));		//Wait for conversion to finish
	ADCSRA|=(1<<ADIF);				//Clear interrupt flag
	sei();
	return ADCH;
}

// function name: is_night ()
// Returns true if LDR is under threshold of sensitivity 
// Hysteresis on ADC values to prevent flickering at border values
_Bool is_night(void)
{
	uint8_t LDR = 0;
	uint8_t Sensitivity = 0;
	
	LDR = adc_conversion(CH4);
	Sensitivity = adc_conversion(CH3)-10; //tbd figure out of we need offset to adc value
	
	if (Sensitivity > 5 )		//Hystersis size	
	{
		if (LDR > Sensitivity)	//Only illuminate bed if it is dark enough
		{
			return true;
		} 
		else
		{
			return false;
		}
	} 
	else
	{
		return false;
	}
	
}

int main(void)
{	
	uint8_t brightnessleft = 0;
	uint8_t brightnessright = 0;
	uint8_t brightnessfront = 0;

	duration = 20; //For testing only
	brightness = 100;

	cli();
    init_ports();
	init_timers();
	init_interrupts();
	init_adc();
	sei();
    while (1) 
    {
		night = is_night();
		if(tick){
			tick = 0;
				if(statusleft == fadein)
				{
					if(brightnessleft <= brightness){
						brightnessleft += 16;
						OCR0A = helligkeit[brightnessleft];
					}
					if(brightnessleft > brightness)
					statusleft = glow;
				}
				else if(statusleft == glow){
					if(runleft > 0)
						runleft--;
					else statusleft = fadeout;

				}
				else if(statusleft == fadeout ){
					if(brightnessleft > 0){
						brightnessleft -= 16;
						OCR0A = helligkeit[brightnessleft];
					}
					if(brightnessleft <= 0)
					statusleft = idle;
				}

				if(statusright == fadein)
				{
					if(brightnessright <= brightness){
						brightnessright += 16;
						OCR0B = helligkeit[brightnessright];
					}
					if(brightnessright > brightness)
					statusright = glow;
				}
				else if(statusright == glow){
					if(runright > 0)
						runright--;
					else statusright = fadeout;

				}
				else if(statusright == fadeout ){
					if(brightnessright > 0){
						brightnessright -= 16;
						OCR0B = helligkeit[brightnessright];
					}
					if(brightnessright <= 0)
					statusright = idle;
				}

				if(statusfront == fadein)
				{
					if(brightnessfront <= brightness){
						brightnessfront += 16;
						OCR2A = helligkeit[brightnessfront];
					}
					if(brightnessfront > brightness)
					statusfront = glow;
				}
				else if(statusfront == glow){
					if(runfront > 0)
						runfront--;
					else statusfront = fadeout;

				}
				else if(statusfront == fadeout ){
					if(brightnessfront > 0){
						brightnessfront -= 16;
						OCR2A = helligkeit[brightnessfront];
					}
					if(brightnessfront <= 0)
					statusfront = idle;
				}
		}

		if(setup){
			setup = 0;
		}
    }
}

