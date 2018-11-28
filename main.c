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

//Global variables
volatile uint8_t duration = 0;
volatile uint8_t brightness = 0;
volatile uint8_t switchingthreshold = 0;

//Function declarations
void init_ports(void);
void init_timers(void);

//Linearisation table for PWM brightness
unsigned int helligkeit[128]={0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 4,
4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 8, 9, 9, 9, 9, 10,
10, 10, 10, 10, 11, 11, 11, 11, 12, 31, 32, 34, 35, 36, 38, 39, 41, 42, 44, 45, 47,
49, 51, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 75, 77, 79, 82, 84, 87, 89, 92,
94, 97, 100, 103, 105, 108, 111, 114, 117, 120, 124, 127, 130, 133, 137, 140, 144, 147,
151, 155, 158, 162, 166, 170, 174, 178, 182, 186, 190, 194, 199, 203, 208, 212, 217,
221, 226, 231, 236, 241, 246, 251, 256,
};


// function name: init_ports()
// Overall setup of I/O ports
// Including port pin interrup configuration
void init_ports(void){
//Set pull-ups for inputs active = 1, disabled = 0 (default)
PORTB = 0b00000000;
PORTC = 0b00000000;
PORTD = 0b00000000;

//Set direction input = 1, output = 0 (default)
DDRB = 0b00000100;
DDRC = 0b00001111;
DDRD = 0b00001100;

//Set interupts
EICRA = 0b00001111; //Rising edge of INT1 and INT0 to trigger
EIMSK = 0b00000011; //Enable Interrupts INT1 and INT0
PCMSK0 = 0b00000100; //Mask out all interrupts beside PCINT2
PCMSK1 = 0b00001000; //Mask out all interrupts beside PCINT11
PCICR = 0b00000011; //Enable Pin change interupt group PCINT7...0 and 8...14
}



// function name: init_timers()
// configuration of timers for PWM output 
// and ontime duration
void init_timers(void){
OCR0A = 128;
OCR0B = 128;
TCCR0A |= (1<<COM0A1) | (1<<COM0B1) | (1<<WGM01) | (1<<WGM00); //Set Fast PWM non inverting, enable OC0A & OC0B
TCCR0B |= (1<<CS01); //8x Prescaler = 8kHz PWM

OCR2A = 128;
TCCR0A |= (1<<COM2A1) | (1<<WGM21) | (1<<WGM20); //Set Fast PWM non inverting, enable OC2A
TCCR0B |= (1<<CS21); //8x Prescaler = 8kHz PWM
}