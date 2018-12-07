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
#include <avr/eeprom.h>

//Defines
#define TRANSIONTIME  10 //Since system tick is about 100ms, this will give us a 1 second fade in or fade out duration
#define HYSTERESIS 5	//Hysteresis for night time detection, will prevent flickering due to values near day / night border

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

//EEPROM variables
uint8_t eeDuration EEMEM = 0;
uint8_t eeBrightness EEMEM = 0;
uint8_t eeThreshold EEMEM = 0;

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
	221, 226, 231, 236, 241, 246, 251, 255
};


// function name: init_ports()
// Overall setup of I/O ports
// 
void init_ports(void){
	//Set pull-ups for inputs active = 1, disabled = 0 (default)
	PORTB = 0b00000000;
	PORTC = 0b00000000;
	PORTD = 0b00010000;

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
	TCCR0A = (1<<COM0A1) | (1<<COM0B1) | (1<<WGM02) | (1<<WGM00); //Set Phase correct PWM non inverting, enable OC0A & OC0B
	TCCR0B = (1<<CS02); //256x Prescaler = 120Hz PWM

	//PWM for LEDs front side
	OCR2A = 0;
	TCCR2A = (1<<COM2A1) | (1<<WGM22) | (1<<WGM20); //Set Phase correct PWM non inverting, enable OC2A
	TCCR2B = (1<<CS22) | (1<<CS21); //256x Prescaler = 120Hz PWM
	
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
	if(statusright == idle && night){
		statusright = fadein;
	}
	runright = duration;
}

ISR(INT1_vect){
	if(statusleft == idle && night){
		statusleft = fadein;
	}
	runleft = duration;
}


ISR(PCINT0_vect){
	if(statusright == idle && night){
		statusright = fadein;
	}
	if(statusfront == idle && night){
		statusfront = fadein;
	}
	runright = duration;
	runfront = duration;
}

ISR(PCINT1_vect){
	if(statusleft == idle && night){
		statusleft = fadein;
	}
	if(statusfront == idle && night){
		statusfront = fadein;
	}
	runleft = duration;
	runfront = duration;
}


ISR(PCINT2_vect){
	if (!(PIND & (1<<PIND4)))	//Since we have a pin change interrupt we check for low level and only then do our stuff
	{
		setup++;				//Cycle through the setup phased with each new press of the button
		if (setup > 3)
		setup = 0;
	}
	
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
	
	LDR = adc_conversion(CH4);
	
	if (switchingthreshold > HYSTERESIS )		//Hysteresis size	
	{
		if (LDR > switchingthreshold)	//Only illuminate bed if it is dark enough
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
	//Local variables
	uint8_t brightnessleft = 0;
	uint8_t brightnessright = 0;
	uint8_t brightnessfront = 0;
	uint8_t stepwidth = 19;
	uint8_t temp = 0;

	//Read values stores in EEPROM upon reset, no need to do the setup each time we unplug the controller
	duration = eeprom_read_byte(&eeDuration);
	brightness = eeprom_read_byte(&eeBrightness);
	switchingthreshold = eeprom_read_byte(&eeThreshold);

	cli();
    init_ports();
	init_timers();
	init_interrupts();
	init_adc();
	sei();
    while (1) 
    {
		night = is_night();		//Check if it dark enough to illuminate
		if(tick){				//Execute the state machine every 100ms
			tick = 0;			//Reset the tick for the next 100ms
			
				//State machine for the left side
				if(statusleft == fadein)		//If we are in idle fade in to maximum brightness
				{
					if(brightnessleft <= brightness){	//Fade in as long as the current brightness for this side is beneath the global brightness 
						brightnessleft += stepwidth;	//Step width is calculated inside the setup routine
						if(brightnessleft > 127)		//Limit brightness to the size for of our Linearization array
							brightnessleft = 127;
						OCR0A = helligkeit[brightnessleft];	//Set the PWM 
					}
					if((brightnessleft > brightness) || (brightnessleft == 127)) //The fade in is finished when we have reached the overall brightness or if we reach the end of the Linearization array
					statusleft = glow;			
				}
				else if(statusleft == glow){	//Now that we reached the desired brightness keep the lights on for the set duration
					if(runleft > 0)				//Decrease the run variable with each timer tick. A retrigger of the motion sensor resets the run variable and prolongs the on time
						runleft--;
					else statusleft = fadeout;	//The duration was long enough we can proceed to fade out the lights
				}
				else if(statusleft == fadeout ){			//Now fadeout with each timer tick one step at a time
					if(brightnessleft > stepwidth){			//Take care that the next subtraction will not lead to negative values or we are out of bound of the Linearization array
						brightnessleft -= stepwidth;
						OCR0A = helligkeit[brightnessleft];
					}
					else{									//Since the previous subtraction would have resulted in negative values we take care of this
						brightnessleft = 0;					//and set the brightness of this side back to 0. This is also the right value to start the fade in next ime
						OCR0A = helligkeit[brightnessleft];
						statusleft = idle;					//We can go back to idle and wait for the next motion sensor trigger event
					}
				}

				//State machine for the right side
				if(statusright == fadein)
				{
					if(brightnessright <= brightness){
						brightnessright += stepwidth;
						if(brightnessright > 127)
							brightnessright = 127;
						OCR0B = helligkeit[brightnessright];
					}
					if((brightnessright > brightness) || (brightnessright == 127))
					statusright = glow;
				}
				else if(statusright == glow){
					if(runright > 0)
						runright--;
					else statusright = fadeout;
				}
				else if(statusright == fadeout ){
					if(brightnessright > stepwidth){
						brightnessright -= stepwidth;
						OCR0B = helligkeit[brightnessright];
					}
					else{
						brightnessright = 0;
						OCR0B = helligkeit[brightnessright];
						statusright = idle;
					}
				}


				//State machine for the front, which also lights up the matching side of the bed
				if(statusfront == fadein)
				{
					if(brightnessfront <= brightness){
						brightnessfront += stepwidth;
						if(brightnessfront > 127)
							brightnessfront = 127;
						OCR2A = helligkeit[brightnessfront];
					}
					if((brightnessfront > brightness) || (brightnessfront == 127))
					statusfront = glow;
				}
				else if(statusfront == glow){
					if(runfront > 0)
						runfront--;
					else statusfront = fadeout;
				}
				else if(statusfront == fadeout ){
					if(brightnessfront > stepwidth){
						brightnessfront -= stepwidth;
						OCR2A = helligkeit[brightnessfront];
					}
					else{
						brightnessfront = 0;
						OCR2A = helligkeit[brightnessfront];
						statusfront = idle;
					}
				}
				
		}


		//State machine for the general setup process
		if(setup == 1){				//We have entered the setup routine
			PORTB |= (1<<PB5);
			_delay_ms(200);			//Single blink of the LED to indicate a correct push of the setup button
			PORTB &= ~(1<<PB5);
			setup = 2;
			while(setup == 2)		//We can now set the overall brightness  and the day / night boundary
			{
				temp = adc_conversion(CH1);			//CH1 is the poti for brightness 
				brightness = temp / 2;				//Devide value by 2 since the poti gives us FF as maxium value but or Linearization array is only 128 values
				switchingthreshold = adc_conversion(CH2);	//Now read the CH2 poti to determine the day / night boundry
				temp = adc_conversion(CH4);			//Also read out the LDR value right now. 
				if (switchingthreshold < temp)		//If the LDR values is bigger than the switching threshold turn led strips on. Attention the LDR will give us lower values the darker it gets
				{
					OCR0A = helligkeit[brightness];
					OCR0B = helligkeit[brightness];
					OCR2A = helligkeit[brightness];
				} 
				else
				{
					OCR0A = 0;
					OCR0B = 0;
					OCR2A = 0;
				}	
			}					//Upon press of the setup button we will leave the loop and proceed with our setup
			stepwidth = brightness / TRANSIONTIME; //Step width is used for Fade-In or Fade-Out animations. 
			if(stepwidth == 0)		//Since we need to substract or add setp width during the animations a value of 0 does not make sense
				stepwidth = 1;
			PORTB |= (1<<PB5);	 //Double blink LED to indicate next setup step
			_delay_ms(200);
			PORTB &= ~(1<<PB5);
			_delay_ms(200);
			PORTB |= (1<<PB5);
			_delay_ms(200);
			PORTB &= ~(1<<PB5);
			OCR0A = 0;			//Just make sure that the stripes are off before we proceed with the setup
			OCR0B = 0;
			OCR2A = 0;
			while(setup == 3)	//Now we somehow set the duration by indicating the poti setting through blinking stripes
			{
				duration = adc_conversion(CH3);	//Read the CH3 poti as value for the duration phase of the illumination
				OCR0A = helligkeit[brightness];
				OCR0B = helligkeit[brightness];
				OCR2A = helligkeit[brightness];
				while(duration--){				
					_delay_ms(1);	//We only delay a 100th of the actual system tick. So the duration during setup is a hundred times faster
				}
				OCR0A = 0;
				OCR0B = 0;
				OCR2A = 0;
				while(duration--){
					_delay_ms(1);
				}
			}		//Again on the press of the setup button we will leave the loop and finish with our setup
			PORTB |= (1<<PB5);		//Tripple blink LED to indicate that setup is finished
			_delay_ms(200);
			PORTB &= ~(1<<PB5);
			_delay_ms(200);
			PORTB |= (1<<PB5);
			_delay_ms(200);
			PORTB &= ~(1<<PB5);
			_delay_ms(200);
			PORTB |= (1<<PB5);
			_delay_ms(200);
			PORTB &= ~(1<<PB5);
			OCR0A = 0;			//Make sure that all strips are turned off 
			OCR0B = 0;
			OCR2A = 0;
			cli();				//Store values into EEPROM if they have changed and make sure that interrupts will no disturb this process
			eeprom_update_byte(&eeDuration, duration);		
			eeprom_update_byte(&eeBrightness, brightness);
			eeprom_update_byte(&eeThreshold, switchingthreshold);
			sei();
			setup = 0;			//Setup if finished and can be restarted upon press of the button
		}
    }
}


/*
Poti1   0	ff
Poti2   0	ff
Poti3   0	ff
LDR	(7) 11	90 (FF)
*/