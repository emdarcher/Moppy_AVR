//code in AVR C for interfacing with the Moppy client from an AVR mcu
//to make music on floppy drives


#include "main.h" //main.h header file, change STEP, DIR, & DEBUG ports in there

//set these things in the Makefile if you need a bigger ring buffer size
//#define UART_RX0_BUFFER_SIZE 4
//#define UART_TX0_BUFFER_SIZE 4

#include "uart.h"

uint8_t flag_store = 0;
#define FIRST_RUN_BIT 0 //bit of the first run flag

#define NUM_DRIVES 8 //put the number of drives you want to use here

/*An array of maximum track positions for each step-control pin.  Even pins
 are used for control, so only even numbers need a value here.  3.5" Floppies have
 80 tracks, 5.25" have 50.  These should be doubled, because each tick is now
 half a position (use 158 and 98).
 */
const uint8_t MAX_POSITION[NUM_DRIVES] = {
  158,158,158,158,158,158,158,158,
};
  
//Array to track the current position of each floppy head.
volatile uint8_t currentPosition[NUM_DRIVES] = {
  0,0,0,0,0,0,0,0,
};
  
#define RESOLUTION_US 40

//Current period assigned to each pin.  0 = off.  Each period is of the length specified by the RESOLUTION
//variable above.  i.e. A period of 10 is (RESOLUTION x 10) microseconds long.
volatile unsigned int currentPeriod[NUM_DRIVES] = {
  0,0,0,0,0,0,0,0,
};

//Current tick
volatile unsigned int currentTick[NUM_DRIVES] = {
  0,0,0,0,0,0,0,0,
};


//prototypes
static inline void tick(void);
inline void setup_timer0_tick(uint8_t us_delay);
void reset(void);
void resetAll(void);
void togglePin(uint8_t pin, uint8_t dir_pin);

void main(void)
{
    //setup stuff
    
    flag_store |= (1<<FIRST_RUN_BIT); //set the first run bit
    
    //setup step pins' port for output
    //STEP_DDR = 0xFF;
    //setup direction pins' port for output
    //DIR_DDR = 0xFF;
    //STEP_PORT |= 0xFF; //so set all step pins to 1 (HIGH) 
    uint8_t nd = NUM_DRIVES;
    while(nd--){
        STEP_DDR    |=  (1<<nd);
        DIR_DDR     |=  (1<<nd);
        STEP_PORT   |=  (1<<nd);
    }   
    #if defined(__AVR_ATmega8515__)
    DEBUG_DDR |= (1<<DEBUG_TICK_BIT);//for debug
    #endif
    //init the USART module and stuff
    //initUSART();
    uart0_init( UART_BAUD_SELECT(9600,F_CPU) );
    //uart0_init( UART_BAUD_SELECT_DOUBLE_SPEED(9600,F_CPU) );
    
    //setup timer0 tick
    setup_timer0_tick(RESOLUTION_US);
    
    sei(); //enable global interrupts
    //infinite loop
    while(1){
        
        //The first loop, reset all the drives, and wait 2 seconds...
        if ( bit_is_set(flag_store,FIRST_RUN_BIT) ){
            flag_store &= ~(1<<FIRST_RUN_BIT);
            resetAll();
            _delay_ms(2000);
        }
        
        //Only read if we have 
        if (uart0_available() > 2){
            //Watch for special 100-message to reset the drives
            if ((uint8_t)uart0_peek() == 100) {
                resetAll();
                //Flush any remaining messages.
                while(uart0_available() > 0){
                    //uart0_flush();
                    uart0_getc();
                }
            }    
            else{
                /* get the 3 chars (bytes) from the serial RX buffer */
                uint8_t ch0,ch1,ch2;
                ch0=uart0_getc();
                ch1=uart0_getc();
                ch2=uart0_getc();
                currentPeriod[(((ch0)>>1)-1)] = ((ch1 << 8) | ch2);
                //uart0_flush();
            }
        }
    }
}

inline void setup_timer0_tick(uint8_t us_delay){
    //configured for a 8MHz system clock divided by 8 for microsecond resolution
    //max delay in microseconds is 255 because it is an 8-bit timer.
    
    TCCR0 |= ((1<<WGM01)|(1<<CS01)); //set to CTC mode, clk/8 divider.
    OCR0 = us_delay; //sets the delay in microseconds for tick
    TIMSK |= (1<<OCIE0); //sets the output compare interrupt enable bit in TIMSK
    
}

void resetAll(void){
    
    uint8_t i=NUM_DRIVES;
    while(i--){
        currentPeriod[i]=0;
    }
    
    // New all-at-once reset
    for (uint8_t s=0;s<80;s++){ // For max drive's position
    //for (byte p=FIRST_PIN;p<=PIN_MAX;p+=2){
        i=NUM_DRIVES;
        while(i--){
            //digitalWrite(p+1,HIGH); // Go in reverse
            DIR_PORT |= (1<<i);
            //digitalWrite(p,HIGH);
            STEP_PORT |= (1<<i);
            //digitalWrite(p,LOW);
            _delay_us(2);
            STEP_PORT &= ~(1<<i);
        }
        _delay_ms(5);
    }
    i=NUM_DRIVES;
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

static inline void tick(void){     
    //DEBUG_PORT ^= (1<<DEBUG_TICK_BIT); //toggle for debug
    /* 
    If there is a period set for control pin 0, count the number of
    ticks that pass, and toggle the pin if the current period is reached.
    */
    #if NUM_DRIVES>0
    if(currentPeriod[0]>0){
        currentTick[0]++;
        if(currentTick[0] >= currentPeriod[0]){
            togglePin(0,0);
            currentTick[0]=0;
        }
    }
    #if NUM_DRIVES>1
    if(currentPeriod[1]>0){
        currentTick[1]++;
        if(currentTick[1] >= currentPeriod[1]){
            togglePin(1,1);
            currentTick[1]=0;
        }
    }
    #if NUM_DRIVES>2
    if(currentPeriod[2]>0){
        currentTick[2]++;
        if(currentTick[2] >= currentPeriod[2]){
            togglePin(2,2);
            currentTick[2]=0;
        }
    }
    #if NUM_DRIVES>3
    if(currentPeriod[3]>0){
        currentTick[3]++;
        if(currentTick[3] >= currentPeriod[3]){
            togglePin(3,3);
            currentTick[3]=0;
        }
    }
    #if NUM_DRIVES>4
    if(currentPeriod[4]>0){
        currentTick[4]++;
        if(currentTick[4] >= currentPeriod[4]){
            togglePin(4,4);
            currentTick[4]=0;
        }
    }
    #if NUM_DRIVES>5
    if(currentPeriod[5]>0){
        currentTick[5]++;
        if(currentTick[5] >= currentPeriod[5]){
            togglePin(5,5);
            currentTick[5]=0;
        }
    }
    #if NUM_DRIVES>6
    if(currentPeriod[6]>0){
        currentTick[6]++;
        if(currentTick[6] >= currentPeriod[6]){
            togglePin(6,6);
            currentTick[6]=0;
        }
    }
    #if NUM_DRIVES>7
    if(currentPeriod[7]>0){
        currentTick[7]++;
        if(currentTick[7] >= currentPeriod[7]){
            togglePin(7,7);
            currentTick[7]=0;
        }
    }
        #if NUM_DRIVES>8
            #error "Too many Drives in NUM_DRIVES, you can't do more than 8 drives using current code."
        #endif
    #endif
    #endif
    #endif
    #endif
    #endif
    #endif
    #endif
    
    #else //if NUM_DRIVES is 0 or undefined
        #error "You must define more than 0 drives in NUM_DRIVES "
    
    #endif
    
    /*
    uint8_t p=NUM_DRIVES;
    while(p--){
        if(currentPeriod[p]>0){
            currentTick[p]++;
            if(currentTick[p] >= currentPeriod[p]){
                togglePin(p,p);
                currentTick[p]=0;
            }
        }
    }*/

}

//isr for timer0 compare match
ISR(TIMER0_COMP_vect){
    //call tick() function
    tick();
    
}
