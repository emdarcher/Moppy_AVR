//code in AVR C for interfacing with the Moppy client from an AVR mcu
//to make music on floppy drives

#include "main.h"
//#include "USART.h"
#include "usart.h"

uint8_t flag_store = 0;
#define FIRST_RUN_BIT 0 //bit of the first run flag

/*An array of maximum track positions for each step-control pin.  Even pins
 are used for control, so only even numbers need a value here.  3.5" Floppies have
 80 tracks, 5.25" have 50.  These should be doubled, because each tick is now
 half a position (use 158 and 98).
 */
const uint8_t MAX_POSITION[] PROGMEM = {
  0,0,158,0,158,0,158,0,158,0,158,0,158,0,158,0,158,0};
  
//Array to track the current position of each floppy head.  (Only even indexes (i.e. 2,4,6...) are used)
uint8_t currentPosition[] = {
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  
//prototypes
inline void tick(void);
void reset(void);
void resetAll(void);

void main(void)
{
    //setup stuff
    
    //setup step pins' port for output
    STEP_DDR = 0xFF;
    //setup direction pins' port for output
    DIR_DDR = 0xFF;
    
    
    
    //init the USART module and stuff
    //initUSART();
    
    //infinite loop
    while(1){
    
    }
}
