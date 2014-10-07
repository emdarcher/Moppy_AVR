/*
 * main.h header file with various avr includes & defines for the main code
 */

// Standard AVR includes
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>

//defines

//STEP pins' ddr and port registers
#define STEP_DDR DDRA
#define STEP_PORT PORTA

//DIR pins's ddr and port registers
#define DIR_DDR DDRC
#define DIR_PORT PORTC

//stuff for old-school-ish debugging,
//and for looking at with a logic analyzer
#if defined(__AVR_ATmega8515__)
#define DEBUG_DDR DDRE
#define DEBUG_PORT PORTE
#define DEBUG_TICK_BIT 1
#endif

