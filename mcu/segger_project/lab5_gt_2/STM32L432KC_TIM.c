// STM32F401RE_TIM.c
// TIM functions

#include "STM32L432KC_TIM.h"
#include "STM32L432KC_RCC.h"

void initTIM(TIM_TypeDef *TIMx) {
    // Set prescaler value
    TIMx->PSC = 1999;

    // Clear UG bit (TIMx_EGR[0]) and setting it to generate an update event
    TIMx->EGR  &= ~(1 << 0);
    TIMx->EGR |= (1 << 0); 

    // Counter enable (CEN, TIMx_CR1[0])
    TIMx->CR1 |= (1 << 0);
}

// Delay in milliseconds using TIMx
void delay_millis(TIM_TypeDef * TIMx, uint32_t ms) {
    // Set auto-reload register (TIMx_ARR[15:0])
    // arr_val = wait_time / 1000 * freq_TIM
    int arr_val = ms * (80000 / (4999+1));
    TIMx->ARR = arr_val;

    // Clear UG bit (TIMx_EGR[0]) and setting it to generate an update event
    TIMx->EGR  &= ~(1 << 0);
    TIMx->EGR |=  (1 << 0);

    // Clear UIF (TIMx_SR[0])
    TIMx->SR  &= ~(1 << 0);

    // Reset count (TIMx_CNT[15:0])
    TIMx->CNT = 0;
  
    // Wait for update event with UIF (TIMx_SR[0]) set
    while(((TIMx->SR) & 1U) != 1);
}

float delay_polling(TIM_TypeDef* TIMX, uint32_t ms) {
  volatile uint32_t duration = ms * 80000 / (1999 + 1);  // convert to ARR val given input clk+psc
  
  TIMX->EGR |=  (1 << 0);       // reset main counter via event flag
  TIMX->ARR  = duration;
  TIMX->SR  &= ~(1 << 0);       // clear update interrupt flag'


  volatile float a_val = 0;
  volatile float b_val = 0;
  volatile float prev_a_val = 0;
  volatile float prev_b_val = 0;

  volatile int count = 0;

  volatile float vel = 0.0;
  volatile float dir = 0.0;     // 1.0 for CW, -1.0 for CCW

  while((TIMX->SR & 1) == 0){
  
    prev_a_val = a_val; 
    prev_b_val = b_val; 
    
    a_val = digitalRead(PA9);
    b_val = digitalRead(PA10);

    if (a_val != prev_a_val || b_val != prev_b_val) {
      if (a_val != b_val && prev_a_val < a_val)         // rising edge for A, first -> CW
        dir = 1.0;
      else if (a_val != b_val && prev_b_val < b_val)    // rising edge for B, first -> CCW
        dir = -1.0;
      count++;
    }
  }

  vel = count * (1.0 / 408.0) / 4.0;
  count = 0;

  return vel * dir;
}