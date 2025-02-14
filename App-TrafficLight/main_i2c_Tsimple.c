 /**
 * RP2040 FreeRTOS Template
 * 
 * @copyright 2022, Tony Smith (@smittytone)
 * @version   1.4.1
 * @licence   MIT
 *  cd ~/FreeRTOS-Play/build/App-TrafficLight
 * 
 * Simulate Traffic light (RED, YELLOW, GREEN). There will be
 * a short delay from Yellow to Red where both NS and EW are
 * RED to deter someone jumping the light.
 * Switch 5 LOW will cause Blinking RED in both directions. When returning
 * to normal, light changes will start where sequence was stopped.
 * For the simulation a 7 seg LED is used for the two different RED YELLOW GREEN
 * light modules
 *   
 */

#define HEX_INTERVALS 256
#define DEC_INTERVALS 400
#define MIN_COUNT 3
#define ALARM_BOUNDARY 8
#define SW_MASK 0B11111111
#define GREEN_PERIOD 4000
#define YELLOW_PERIOD 1500
#define PAUSE_PERIOD 800
#define GREEN_TIMER_ID             0
#define YELLOW_TIMER_ID            1
#define PAUSE_TIMER_ID             2
#define EMERG_TIMER_ID             3

#include <stdio.h>
#include "main.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../Common/Seven_Seg_i2c/seven_seg.h"  
#include "../Common/PCF8575_i2c/pcf8575i2c.h"


/*
 * GLOBALS
 */
 
// This is the inter-task queue
volatile QueueHandle_t xQblink = NULL;
volatile QueueHandle_t xQsw5 = NULL;
volatile QueueHandle_t xQgreen = NULL;
volatile QueueHandle_t xQyellow = NULL;
volatile QueueHandle_t xQpause = NULL;

volatile TimerHandle_t green_timer;
volatile TimerHandle_t yellow_timer;
volatile TimerHandle_t pause_timer;
volatile TimerHandle_t emerg_timer;

// Set a delay time of exactly 500ms
const TickType_t ms_delay = 500 / portTICK_PERIOD_MS;
 
// FROM 1.0.1 Record references to the tasks
TaskHandle_t blink_task_handle = NULL;
TaskHandle_t traffic_light_task_handle = NULL;
TaskHandle_t sw_debounce_task_handle = NULL;
/*
 * FUNCTIONS
 */
void switch5(uint8_t);

 
/**
 * @brief Repeatedly flash the Pico's built-in LED.
 *  Time delay of Flash defined by TaskDelayUntil()
 */
void blink_task(void* unused_arg) {
    
    int steps = 600;
    uint32_t pico_led_state = 0;
    
   // Initialize start time for vTaskDelayUntil
    TickType_t lastTickTime = xTaskGetTickCount();
        
    while (true) {

        pico_led_state = !pico_led_state ;  // Toggle pico led state
        
        gpio_put(PICO_LED_PIN, pico_led_state); 
        xQueueOverwrite(xQblink, &pico_led_state);
        vTaskDelayUntil(&lastTickTime, pdMS_TO_TICKS(steps));
        }
} 

void traffic_light_task(void* unused_arg) {  
    // This variable will take a copy of the value
    // added to the FreeRTOS Queue
    uint8_t now = 0;
    uint8_t sw5_buffer = 0;
    uint8_t sw5_state = 0;
    uint8_t sw_state = 0;
    uint8_t traffic_state = 6;
    struct op
    {
        uint8_t t_number;
        uint8_t t_state;
    }; 
    struct op t1_info = {1,1};
    
    while (true) {
                if(uxQueueMessagesWaiting(xQsw5)) { // check for ACK, turn ACK flag off
                    xQueuePeek(xQsw5, &sw5_buffer, 0);
                }
            switch(traffic_state){
            case 0:
                show_stoplight(traffic_state);
                if (green_timer != NULL) xTimerStart(green_timer, 0);  // define dot length 
                xQueueReceive(xQgreen, &t1_info, portMAX_DELAY);   // xQueueReceive empties queue
                traffic_state = 1;
               break;
            case 1:
                show_stoplight(traffic_state);
                if (yellow_timer != NULL) xTimerStart(yellow_timer, 0);  // define dot length 
                xQueueReceive(xQyellow, &t1_info, portMAX_DELAY);   // xQueueReceive empties queue
                traffic_state = 2; 
                break;
            case 2:
                show_stoplight(traffic_state);
                if (pause_timer != NULL) xTimerStart(pause_timer, 0);  // define dot length 
                xQueueReceive(xQpause, &t1_info, portMAX_DELAY);   // xQueueReceive empties queue
                now = traffic_state;
                traffic_state = 3;
                if(!sw5_buffer) traffic_state = 6;  // Blink RED while sw5 is LOW
                break;
            case 3:
                 show_stoplight(traffic_state);
                if (green_timer != NULL) xTimerStart(green_timer, 0);  // define dot length 
                xQueueReceive(xQgreen, &t1_info, portMAX_DELAY);   // xQueueReceive empties queue
                traffic_state = 4;
                break;
            case 4:
                show_stoplight(traffic_state);
                if (yellow_timer != NULL) xTimerStart(yellow_timer, 0);  // define dot length 
                xQueueReceive(xQyellow, &t1_info, portMAX_DELAY);   // xQueueReceive empties queue
                traffic_state = 5;
                break;
            case 5:
                show_stoplight(traffic_state);
                if (pause_timer != NULL) xTimerStart(pause_timer, 0);  // define dot length 
                xQueueReceive(xQpause, &t1_info, portMAX_DELAY);   // xQueueReceive empties queue
                now = traffic_state;
                traffic_state = 0;
                if(!sw5_buffer) traffic_state = 6;  // Blink RED while sw5 is LOW
                break;
            case 6:
                show_stoplight(traffic_state);
                if (pause_timer != NULL) xTimerStart(pause_timer, 0);  // define dot length 
                xQueueReceive(xQpause, &t1_info, portMAX_DELAY);   // xQueueReceive empties queue
                traffic_state = now;  //  return to case which activated case 6
                break;
             }
   }
}


/**
 * @brief Callback actioned when the CW timer fires.  Sends trigger to
 * initiate a new CW String TX.
 *
 * @param timer: The triggering timer.
 */
void green_timer_fired_callback(TimerHandle_t timer) {
    struct op
    {
        uint8_t t_number;
        uint8_t t_state;
    }; 
    struct op timer_info = {0,0};

    if (timer == green_timer) {
        // The timer fired
        xQueueOverwrite(xQgreen, &timer_info);
       }
}


/**
 * @brief Callback actioned when the CW timer fires.  Sends trigger to
 * initiate a new CW String TX.
 *
 * @param timer: The triggering timer.
 */
void yellow_timer_fired_callback(TimerHandle_t timer) {
    struct op
    {
        uint8_t t_number;
        uint8_t t_state;
    }; 
    struct op timer_info = {1,0};

    if (timer == yellow_timer) {
        // The timer fired
        xQueueOverwrite(xQyellow, &timer_info);
       }
}

/**
 * @brief Callback actioned when the CW timer fires.  Sends trigger to
 * initiate a new CW String TX.
 *
 * @param timer: The triggering timer.
 */
void pause_timer_fired_callback(TimerHandle_t timer) {
    uint8_t sw5_buffer = 0;
    uint8_t sw5_state = 0;
    struct op
    {
        uint8_t t_number;
        uint8_t t_state;
    }; 
    struct op timer_info = {2,0};

    if (timer == pause_timer) {
        // The timer fired
        xQueueOverwrite(xQpause, &timer_info);
       }
}



/**
 * @brief Switch Debounce Task  
 *  *** This task can be made smaller by removing all sub-functions you do not need. ***
 * Repeat check of Buffer[0] of the Port Extender. The port extender
 * Buffer[0] monitors eight switches.  The task scans the switches regularly but only changes the output
 * if a change is found.
 * Measures sw state, compares NOW state with PREVIOUS state. If states are different
 * sets count == 0 and looks for two states the same.  It then looks for MIN_COUNT counts
 * in a row or more where NOW and PREVIOUS states are the same. Once switch bounce is not observed
 * the Switch state is passed through a Queue to a function to be used as a control signal.
 * The Switch Queues are all set to hold the value continuously until changed, so read them with
 * a Peek.
 */
 void sw_debounce_task(void* unused_arg) {
    uint8_t sw_previous_state = 0b11111111;   // initialize sw_previous_state
    uint8_t sw_state = 0b11111111;            // initialize sw_state
    uint32_t count = 5;               // initialize sw_final_state
    uint8_t pcfbuffer[]={0b11111111,0b11111111};// data buffer, must be two bytes
    
    while (true) {
        // Measure SW and add the LED state
        // to the FreeRTOS xQUEUE if switch has changed
        sw_previous_state = sw_state;
        i2c_read_blocking(i2c0, I2C_ADDR, pcfbuffer, 2, false);
         sw_state = (pcfbuffer[0] & SW_MASK);
         
        if(sw_previous_state == sw_state) {
            if (count < 10) {
                count += 1;
            }
            else {  // reset cout to MIN_COUNT
                count = MIN_COUNT;
             }              //  End if (count < 10)
          vTaskDelay(ms_delay10);  // check switch state every 10 ms
         }
        else  { //  if sw_previous state |= sw_state switch has changed
        
             count = 0;  // Need at least MIN_COUNT consecutive same states
             while(count < MIN_COUNT) {
                sw_previous_state = sw_state;
                i2c_read_blocking(i2c0, I2C_ADDR, pcfbuffer, 2, false);
                 sw_state = (pcfbuffer[0] & SW_MASK);
                if(sw_previous_state == sw_state){
                     count++;
                }
                 else {
                     count = 0;
                 }
                vTaskDelay(ms_delay10);  // check switch state every 10 ms
            }

            switch5(sw_state);
        }   // end else(sw_previous_state |= sw_state)
        vTaskDelay(ms_delay50);  // check switch state every 50 ms
    }  // End while (true)    
}


/*
 * Switch5(uint8_t)
 *  Input Switch state value.  The function will Mask out its Switch value 
 * from sw_state and will send Qmessage to task requiring
 * switch change. The switch value is expected to remain in the gueue until
 * changed.
 */
void switch5(uint8_t sw_state) {
    uint8_t now = 0 ;
    
    if(sw_state & 0B00100000){
         now = 1;
     }
     else {
         now = 0;
     }
        xQueueOverwrite(xQsw5, &now);
}


/**
 * @brief Initialize GPIO Pins for input and output.
 *        Initialize seven segment display
 *        Initialize pcf8575 GPIO Extender
 */
void configure_gpio(void) {
    uint8_t pico_led_state = 0;

    // Configure PICO_LED_PIN for Initialization failure warning
    gpio_init(PICO_LED_PIN);
    gpio_disable_pulls(PICO_LED_PIN);  // remove pullup and pulldowns
    gpio_set_dir(PICO_LED_PIN, GPIO_OUT);
    
    // Configure GPIO Extender
    pcf8575_init();

    // Configure Seven Segment display
    config_seven_seg();

}


/*
 * RUNTIME START
 */
int main() {
    uint32_t error_state = 0;
    uint8_t pico_led_state = 0;
    struct op
    {
        uint8_t sw_number;
        uint8_t sw_state;
    }; 
    struct op sw_info = {0,0};
    
    stdio_usb_init(); 
    // Pause to allow the USB path to initialize
    sleep_ms(2000);
    
    configure_gpio();
    
        // label Program Screen
    printf("\x1B[2J");  // Clear Screen
    printf("\x1B[%i;%iH", 2,3);  // place curser
    printf("*** Play Program ***");
    printf("\x1B[%i;%iH",4,2);  // place curser
    printf("**************************************\n");
    printf("\x1B[%i;%ir",6,18); // set window top and bottom lines
    printf("\x1B[%i;%iH",8,0);  // place curser

// Timer creates green light delay
    green_timer = xTimerCreate("GREEN_TIMER", 
                            GREEN_PERIOD,
                            pdFALSE,
                            (void*)GREEN_TIMER_ID,
                            green_timer_fired_callback);

// Timer creates Yellow light delay
    yellow_timer = xTimerCreate("YELLOW_TIMER", 
                            YELLOW_PERIOD,
                            pdFALSE,
                            (void*)YELLOW_TIMER_ID,
                            yellow_timer_fired_callback);

// Timer creates RED pause between the YELLOW LED and next GREEN LED
    pause_timer = xTimerCreate("PAUSE_TIMER", 
                            PAUSE_PERIOD,
                            pdFALSE,
                            (void*)PAUSE_TIMER_ID,
                            pause_timer_fired_callback);

  
    // Set up five tasks
    // FROM 1.0.1 Store handles referencing the tasks; get return values
    // NOTE Arg 3 is the stack depth -- in words, not bytes
    BaseType_t blink_status = xTaskCreate(blink_task, 
                                         "BLINK_TASK", 
                                         128, 
                                         NULL, 
                                         7, 
                                         &blink_task_handle);
        if (blink_status != pdPASS) {
            error_state  += 1;
            }
             
    BaseType_t traffic_light_status = xTaskCreate(traffic_light_task, 
                                         "TRAFFIC_LIGHT_TASK", 
                                         128, 
                                         NULL, 
                                         6, 
                                         &traffic_light_task_handle);
    
        if (traffic_light_status != pdPASS) {
            error_state  += 1;
            }
             
    BaseType_t sw_status = xTaskCreate(sw_debounce_task, 
                                         "SW__DEBPIMCE_TASK", 
                                         256, 
                                         NULL, 
                                         5,     // Task priority
                                         &sw_debounce_task_handle);
        if (sw_status != pdPASS) {
           error_state  += 1;
            }
            
    
    // Set up the two queues
     
    xQblink = xQueueCreate(1, sizeof(uint32_t));
    if ( xQblink == NULL ) error_state += 1;

    xQsw5 = xQueueCreate(1, sizeof(uint8_t)); 
    if ( xQsw5 == NULL ) error_state += 1;

    xQgreen = xQueueCreate(1, sizeof(sw_info)); 
    if ( xQgreen == NULL ) error_state += 1;

    xQyellow = xQueueCreate(1, sizeof(sw_info)); 
    if ( xQyellow == NULL ) error_state += 1;

    xQpause = xQueueCreate(1, sizeof(sw_info)); 
    if ( xQpause == NULL ) error_state += 1; 

    // Start the FreeRTOS scheduler
    // FROM 1.0.1: Only proceed with valid tasks
    if (error_state == 0) {
        vTaskStartScheduler();
    }
    else {   // if tasks don't initialize, pico board led will light   
        pico_led_state = 1;
        gpio_put(PICO_LED_PIN, pico_led_state);
    }
    
    // We should never get here, but just in case...
    while(true) {
        // NOP
    };
}
