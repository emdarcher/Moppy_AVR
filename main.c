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
const uint8_t MAX_POSITION[8] = {
  158,158,158,158,158,158,158,158};
  
//Array to track the current position of each floppy head.  (Only even indexes (i.e. 2,4,6...) are used)
uint8_t currentPosition[8] = {
  0,0,0,0,0,0,0,0};
  
#define RESOLUTION_US 40

//Current period assigned to each pin.  0 = off.  Each period is of the length specified by the RESOLUTION
//variable above.  i.e. A period of 10 is (RESOLUTION x 10) microseconds long.
unsigned int currentPeriod[8] = {
  0,0,0,0,0,0,0,0,
};

//Current tick
unsigned int currentTick[8] = {
  0,0,0,0,0,0,0,0,
};


//prototypes
inline void tick(void);
inline void setup_timer0_tick(uint8_t us_delay);
void reset(void);
void resetAll(void);
void togglePin(uint8_t pin, uint8_t dir_pin);

void main(void)
{
    //setup stuff
    
    flag_store |= (1<<FIRST_RUN_BIT); //set the first run bit
    
    //setup step pins' port for output
    STEP_DDR = 0xFF;
    //setup direction pins' port for output
    DIR_DDR = 0xFF;
    
    
    
    //init the USART module and stuff
    //initUSART();
    uart0_init(UART_BAUD_SELECT(9600,F_CPU) );
    
    
    //setup timer0 tick
    setup_timer0_tick(RESOLUTION_US);
    
    sei(); //enable global interrupts
    //infinite loop
    while(1){
        
        //The first loop, reset all the drives, and wait 2 seconds...
        if ( bit_is_set(flag_store,FIRST_RUN_BIT) )
        {
        flag_store &= ~(1<<FIRST_RUN_BIT);
        resetAll();
        _delay_ms(2000);
        }
        
        //Only read if we have 
        if (uart0_available() > 2){
            //Watch for special 100-message to reset the drives
            if (uart0_peek() == 100) {
                resetAll();
                //Flush any remaining messages.
                while(uart0_available() > 0){
                    uart0_flush();
                }
            }    
            else{
                currentPeriod[(((uart0_getc())>>1)-1)] = (uart0_getc() << 8) | uart0_getc();
            }
        }
    }
}

inline void setup_timer0_tick(uint8_t us_delay){
    //configured for a 8MHz system clock divided by 8 for microsecond resolution
    
    TCCR0 |= ((1<<WGM01)|(1<<CS01)); //set to CTC mode, clk/8 divider.
    OCR0 = us_delay; //sets the delay in microseconds for tick
    TIMSK |= (1<<OCIE0); //sets the output compare interrupt enable bit in TIMSK
    
}

void resetAll(void){
    
    uint8_t i=7;
    while(i--){
        currentPeriod[i]=0;
    }
    
    // New all-at-once reset
  for (uint8_t s=0;s<80;s++){ // For max drive's position
    //for (byte p=FIRST_PIN;p<=PIN_MAX;p+=2){
      i=7;
        while(i--){
      //digitalWrite(p+1,HIGH); // Go in reverse
        DIR_PORT |= (1<<i);
        //digitalWrite(p,HIGH);
        STEP_PORT |= (1<<i);
        //digitalWrite(p,LOW);
        STEP_PORT &= ~(1<<i);
        }
    _delay_ms(5);
  }
    i=7;
    while(i--){
        
        currentPosition[i] = 0;
        DIR_PORT &= ~(1<<i);
        
        
    }
    
}

void togglePin(uint8_t pin, uint8_t dir_pin){
    
    //Switch directions if end has been reached
  if (currentPosition[pin] >= MAX_POSITION[pin]) {
    //currentState[direction_pin] = HIGH;
    //digitalWrite(direction_pin,HIGH);
    DIR_PORT |= (1<<dir_pin);
  } 
  else if (currentPosition[pin] <= 0) {
    //currentState[direction_pin] = LOW;
    //digitalWrite(direction_pin,LOW);
    DIR_PORT &= ~(1<<dir_pin);
  }

  //Update currentPosition
  if (bit_is_set(DIR_PORT,dir_pin)){
    currentPosition[pin]--;
  } 
  else {
    currentPosition[pin]++;
  }

  //Pulse the control pin
  //digitalWrite(pin,currentState[pin]);
  //currentState[pin] = ~currentState[pin];
  STEP_PORT ^= (1<<pin);
    
}

inline void tick(void){
    
    
    /* 
   If there is a period set for control pin 2, count the number of
   ticks that pass, and toggle the pin if the current period is reached.
   */
   /*
  if (currentPeriod[2]>0){
    currentTick[2]++;
    if (currentTick[2] >= currentPeriod[2]){
      togglePin(2,3);
      currentTick[2]=0;
    }
  }
  if (currentPeriod[4]>0){
    currentTick[4]++;
    if (currentTick[4] >= currentPeriod[4]){
      togglePin(4,5);
      currentTick[4]=0;
    }
  }
  if (currentPeriod[6]>0){
    currentTick[6]++;
    if (currentTick[6] >= currentPeriod[6]){
      togglePin(6,7);
      currentTick[6]=0;
    }
  }
  if (currentPeriod[8]>0){
    currentTick[8]++;
    if (currentTick[8] >= currentPeriod[8]){
      togglePin(8,9);
      currentTick[8]=0;
    }
  }
  if (currentPeriod[10]>0){
    currentTick[10]++;
    if (currentTick[10] >= currentPeriod[10]){
      togglePin(10,11);
      currentTick[10]=0;
    }
  }
  if (currentPeriod[12]>0){
    currentTick[12]++;
    if (currentTick[12] >= currentPeriod[12]){
      togglePin(12,13);
      currentTick[12]=0;
    }
  }
  if (currentPeriod[14]>0){
    currentTick[14]++;
    if (currentTick[14] >= currentPeriod[14]){
      togglePin(14,15);
      currentTick[14]=0;
    }
  }
  if (currentPeriod[16]>0){
    currentTick[16]++;
    if (currentTick[16] >= currentPeriod[16]){
      togglePin(16,17);
      currentTick[16]=0;
    }
  }*/
  
    uint8_t p=7;
    while(p--){
        if(currentPeriod[p]>0){
            currentTick[p]++;
            if(currentTick[p] >= currentPeriod[p]){
                togglePin(p,p);
                currentTick[p]=0;
            }
        }
    }
    
}

//isr for timer0 compare match
ISR(TIMER0_COMP_vect){
    //call tick() function
    tick();
    
}
