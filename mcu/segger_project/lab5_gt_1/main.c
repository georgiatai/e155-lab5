/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

/////////////////////////
// main.c
// Author: Georgia Tai, ytai@g.hmc.edu
// Date: Oct. 5, 2025
// 
// Main program of the MCU for motor speed measurement.
/////////////////////////

*/

#include "STM32L432KC.h"

#define A_IN digitalRead(PA9)  // PA9
#define B_IN digitalRead(PA10) // PA10

#define _VAL2FLD(field, value) (((uint32_t)(value) << field ## _Pos) & field ## _Msk)

signed int quad_count = 0;   // Quadrature count (rising and falling edges of both A and B)
uint32_t   direction  = 0;   // 0 is clockwise, 1 is counter-clockwise
float      ang_vel    = 0.0; // Angular velocity in rev/s


int main(void) {
    configureFlash();
    configureClock();

    // Enable clocks
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // SYSCFG clock
    RCC->AHB2ENR |= (1 << 0);             // GPIOA
    RCC->APB2ENR |= (1 << 16);            // TIM15

    // Initialize timer and timer interrupt
    initTIM(TIM15);
    TIM15->DIER |= (1 << 0); // UIE: Update interrupt enable

    // Setting up GPIOA
    gpioEnable(GPIO_PORT_A);
    pinMode(PA9, GPIO_INPUT);  // PA9 as input
    pinMode(PA10, GPIO_INPUT); // PA10 as input
    
    GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD9, 0b00);  // Clear bits
    GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD9, 0b01);  // Set PA9 as pull-up
    GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD10, 0b00); // Clear bits
    GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD10, 0b01); // Set PA10 as pull-up

    // Configure EXTICR for two GPIO pins interrupt
    SYSCFG->EXTICR[2] |= _VAL2FLD(SYSCFG_EXTICR3_EXTI9, 0b000);  // Select PA9
    SYSCFG->EXTICR[2] |= _VAL2FLD(SYSCFG_EXTICR3_EXTI10, 0b000); // Select PA10

    // Enable interrupts globally
    __enable_irq();

    // Configure EXTI for two GPIO pins interrupt
    // Configure mask bit
    EXTI->IMR1 |= (1 << gpioPinOffset(PA9));
    EXTI->IMR1 |= (1 << gpioPinOffset(PA10));
    // Enable rising edge trigger
    EXTI->RTSR1 |= (1 << gpioPinOffset(PA9));
    EXTI->RTSR1 |= (1 << gpioPinOffset(PA10));
    // Enable falling edge trigger
    EXTI->FTSR1 |= (1 << gpioPinOffset(PA9));
    EXTI->FTSR1 |= (1 << gpioPinOffset(PA10));
    // Turn on EXTI interrupt in NVIC_ISER
    __NVIC_EnableIRQ(EXTI9_5_IRQn);
    __NVIC_EnableIRQ(EXTI15_10_IRQn);

    // Main while loop
    while(1){
      ang_vel = quad_count / (4.0 * 408.0); // Angular velocity in rev/s (408 PPR, 4 edges per cycle)
      printf("Angular velocity: %.3f rev/s, Direction: %s\n", direction ? -ang_vel : ang_vel, direction ? "CCW" : "CW");
      quad_count = 0;                       // Reset count
      delay_millis(TIM15, 1000);            // Update velocity and direction every second
    }
}

// Interrupt Handler for PA9 (encoder A)
void EXTI9_5_IRQHandler(void){
    // Check that PA9 was what triggered the interrupt
    if (EXTI->PR1 & (1 << 9)){
        // Clear the interrupt
        EXTI->PR1 |= (1 << 9);

        // Use the states of A and B to determine direction and update count
        if (A_IN != B_IN) {  // Clockwise
            quad_count++;
            direction = 0;
        } else {             // Counterclockwise
            quad_count--;
            direction = 1;
        }
    }
}

// Interrupt Handler for PA10 (encoder B)
void EXTI15_10_IRQHandler(void){
    // Check that PA10 was what triggered the interrupt
    if (EXTI->PR1 & (1 << 10)){
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << 10);

        // Use the states of A and B to determine direction and update count
        if (A_IN == B_IN) {  // Clockwise
            quad_count++;
            direction = 0;
        } else {             // Counterclockwise
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
