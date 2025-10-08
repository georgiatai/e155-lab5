
#include <stm32l432xx.h>
#include <stdio.h>

#define A_IN (GPIOA->IDR & (1 << 6)) // PA6
#define B_IN (GPIOA->IDR & (1 << 7)) // PA7

uint32_t quad_count = 0;
uint32_t direction  = 0;   // 0 is clockwise, 1 is counter-clockwise
float    ang_vel    = 0.0; // Angular velocity in rev/s


int main(void) {
    configureFlash();
    configureClock();
	
    // Setting up clocks
    RCC->AHB2ENR |= (1 << 0);  // GPIOA
    RCC->APB2ENR |= (1 << 16); // TIM15

    // Initialize timer
    initTIM(TIM15);
    TIM15->DIER |= _VAL2FLD(TIM15_DIER_UIE, 0b1);

    // Setting up GPIOA
    gpioEnable(GPIO_PORT_A);
    pinMode(PA6, GPIO_INPUT); // PA6 as input
    pinMode(PA7, GPIO_INPUT); // PA7 as input

    GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD6, 0b01); // Set PA6 as pull-up
    GPIOA->PUPDR |= _VAL2FLD(GPIO_PUPDR_PUPD7, 0b01); // Set PA7 as pull-up

    // 1. Enable SYSCFG clock domain in RCC
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    // 2. Configure EXTICR for the input button interrupt
    SYSCFG->EXTICR[1] |= _VAL2FLD(SYSCFG_EXTICR1_EXTI2, 0b000); // Select PA2

    // Enable interrupts globally
    __enable_irq();

    // TODO: Configure interrupt for falling edge of GPIO pin for button
    // 1. Configure mask bit
    EXTI->IMR1 |= (1 << gpioPinOffset(BUTTON_PIN)); // Configure the mask bit
    // 2. Disable rising edge trigger
    EXTI->RTSR1 &= ~(1 << gpioPinOffset(BUTTON_PIN));// Disable rising edge trigger
    // 3. Enable falling edge trigger
    EXTI->FTSR1 |= (1 << gpioPinOffset(BUTTON_PIN));// Enable falling edge trigger
    // 4. Turn on EXTI interrupt in NVIC_ISER
    NVIC->ISER[0] |= (1 << EXTI6_IRQn);
    NVIC->ISER[0] |= (1 << EXTI7_IRQn);


    while(1){
        delay_millis(TIM15, 1000); // Delay 1 second
        ang_vel = (float) quad_count / 400.0; // Calculate angular velocity in rev/s
        quad_count = 0; // Reset count
    };

}

void EXTI9_5_IRQHandler(void){
    // Check that the button was what triggered our interrupt
    if (EXTI->PR1 & (1 << 6)){
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << 6);

        // Then toggle the LED
        if (A_IN && !B_IN) {
            // Clockwise
            quad_count++;
            direction = 0;
        } else if (!A_IN && B_IN) {
            // Counterclockwise
            quad_count--;
            direction = 1;
        }
    }

    if (EXTI->PR1 & (1 << 7)){
        // If so, clear the interrupt (NB: Write 1 to reset.)
        EXTI->PR1 |= (1 << 7);

        // Then toggle the LED
        if (A_IN && !B_IN) {
            // Clockwise
            quad_count++;
            direction = 0;
        } else if (!A_IN && B_IN) {
            // Counterclockwise
            quad_count--;
            direction = 1;
        }

    }
}

void TIM1_UP_TIM15_IRQHandler(void){
    // Check that the button was what triggered our interrupt
    if (TIM15->SR & (1 << 0)){
        // If so, clear the interrupt (NB: Write 1 to reset.)
        TIM15->SR |= (1 << 0);

        printf("Angular velocity: %.2f rev/s, Direction: %s\n", ang_vel, direction ? "CCW" : "CW");
        quad_count = 0; // Reset count
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