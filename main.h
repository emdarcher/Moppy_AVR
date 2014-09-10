// Standard AVR includes
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>


#define STEP_DDR DDRA
#define STEP_PORT PORTA

#define DIR_DDR DDRC
#define DIR_PORT PORTC

//stuff for old-school-ish debugging,
//and for look at with logic analyzer
#define DEBUG_DDR DDRE
#define DEBUG_PORT PORTE
#define DEBUG_TICK_BIT 1

