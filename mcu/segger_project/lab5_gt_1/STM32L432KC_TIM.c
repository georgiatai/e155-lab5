// STM32F401RE_TIM.c
// TIM functions

#include "STM32L432KC_TIM.h"
#include "STM32L432KC_RCC.h"

void initTIM(TIM_TypeDef *TIMx) {
    // Set prescaler value
    TIMx->PSC = 4999;

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
