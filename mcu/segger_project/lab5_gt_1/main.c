/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : main.c
Purpose : Generic application start

*/

#include "STM32L432KC.h"

#define A_IN digitalRead(PA9)  // PA9
#define B_IN digitalRead(PA10) // PA10

#define _VAL2FLD(field, value) (((uint32_t)(value) << field ## _Pos) & field ## _Msk)

signed int quad_count = 0;
uint32_t direction  = 0;   // 0 is clockwise, 1 is counter-clockwise
float    ang_vel    = 0.0; // Angular velocity in rev/s


int main(void) {
    configureFlash();
    configureClock();

    // 1. Enable SYSCFG clock domain in RCC
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;	
    // Setting up clocks
    RCC->AHB2ENR |= (1 << 0);  // GPIOA
    RCC->APB2ENR |= (1 << 16); // TIM15

    // Initialize timer
    initTIM(TIM15);
    TIM15->DIER |= (1 << 0);

    // Setting up GPIOA
    gpioEnable(GPIO_PORT_A);
    pinMode(PA9, GPIO_INPUT); // PA9 as input
    pinMode(PA10, GPIO_INPUT); // PA10 as input
    
    GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD9, 0b00); // Set PA9 as pull-up
    GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD9, 0b01); // Set PA9 as pull-up
    GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD10, 0b00); // Set PA9 as pull-up
    GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD10, 0b01); // Set PA10 as pull-up


    // 2. Configure EXTICR for the input button interrupt
    SYSCFG->EXTICR[2] |= _VAL2FLD(SYSCFG_EXTICR3_EXTI9, 0b000); // Select PA2
    SYSCFG->EXTICR[2] |= _VAL2FLD(SYSCFG_EXTICR3_EXTI10, 0b000); // Select PA2

    // Enable interrupts globally
    __enable_irq();

    // TODO: Configure interrupt for falling edge of GPIO pin for button
    // 1. Configure mask bit
    EXTI->IMR1 |= (1 << gpioPinOffset(PA9)); // Configure the mask bit
    EXTI->IMR1 |= (1 << gpioPinOffset(PA10)); // Configure the mask bit
    // 2. Disable rising edge trigger
    EXTI->RTSR1 |= (1 << gpioPinOffset(PA9)); // Disable rising edge trigger
    EXTI->RTSR1 |= (1 << gpioPinOffset(PA10));// Disable rising edge trigger
    // 3. Enable falling edge trigger
    EXTI->FTSR1 |= (1 << gpioPinOffset(PA9));// Enable falling edge trigger
    EXTI->FTSR1 |= (1 << gpioPinOffset(PA10));// Enable falling edge trigger
    // 4. Turn on EXTI interrupt in NVIC_ISER
    __NVIC_EnableIRQ(EXTI9_5_IRQn);
    __NVIC_EnableIRQ(EXTI15_10_IRQn);



    while(1){
      ang_vel = quad_count / (4.0*408.0); 
      printf("Angular velocity: %.3f rev/s, Direction: %s\n", ang_vel, direction ? "CCW" : "CW");
      quad_count = 0; // Reset count
      delay_millis(TIM15, 1000);
      
    }

}

void EXTI9_5_IRQHandler(void){
    // Check that the button was what triggered our interrupt
    if (EXTI->PR1 & (1 << 9)){
        // printf("PA9, A_IN %d, B_IN %d \n", digitalRead(PA9), digitalRead(PA10));
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << 9);

        // Then toggle the LED
        if (A_IN != B_IN) {
            // Clockwise
            quad_count++;
            direction = 0;
        } else {
            // Counterclockwise
            quad_count--;
            direction = 1;
        }

    }
}

void EXTI15_10_IRQHandler(void){
  
    // Check that the button was what triggered our interrupt
    if (EXTI->PR1 & (1 << 10)){
        // printf("PA10, A_IN %d, B_IN %d \n", digitalRead(PA9), digitalRead(PA10));
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << 10);

        // Then toggle the LED
        if (A_IN == B_IN) {
            // Clockwise
            quad_count++;
            direction = 0;
        } else {
            // Counterclockwise
            quad_count--;
            direction = 1;
        }

    }
}

// Function used by printf to send characters to the laptop
int _write(int file, char *ptr, int len) {
  int i = 0;
  for (i = 0; i < len; i++) {
    ITM_SendChar((*ptr++));
  }
  return len;
}
