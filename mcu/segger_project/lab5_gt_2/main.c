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

    while(1){     
      ang_vel = delay_polling(TIM15, 1000);
      printf("velocity (manual polling): %f rev/s\n", ang_vel);
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
