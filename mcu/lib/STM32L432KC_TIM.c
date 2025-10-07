/////////////////////////
// STM32L432KC_TIM.c
// Author: Georgia Tai, ytai@g.hmc.edu
// Date: Sep 28, 2025
// 
// Source code for TIM functions, including initialization, delay, and PWM generation.
/////////////////////////

#include "STM32L432KC_TIM.h"
#include <stdio.h>

// Initialize timer for basic counting (delay)
void initTIM(TIMx_TypeDef *TIMx, uint32_t psc_val) {
    // Set prescaler value
    TIMx->PSC = psc_val;

    // Clear UG bit (TIMx_EGR[0]) and setting it to generate an update event
    TIMx->EGR  &= ~(1 << 0);
    TIMx->EGR |= (1 << 0); 

    // Counter enable (CEN, TIMx_CR1[0])
    TIMx->CR1 |= (1 << 0);
}

// Delay in milliseconds using TIMx
void delay_millis(TIMx_TypeDef * TIMx, uint32_t ms, uint32_t psc_val) {
    // Set auto-reload register (TIMx_ARR[15:0])
    // arr_val = wait_time / 1000 * freq_TIM
    int arr_val = ms * (80000 / (psc_val+1));
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

// Initialize timer for PWM generation
void initPWM(TIMx_TypeDef *TIMx, uint32_t psc_val) {
    // Set prescaler value
    TIMx->PSC = psc_val;

    // Set output 1 polarity to active high
    TIMx->CCER |= (1 << 3);

    // Enable output on channel 1
    TIMx->CCER |= (1 << 0);

    // Set mode to PWM mode 1 (110) on channel 1
    TIMx->CCMR1 &= ~(0b111 << 4);
    TIMx->CCMR1 |=  (0b110 << 4);

    // Enable preload for CCR1 later for dutying cycle updates
    TIMx->CCMR1 |= (1 << 3);

    // Enable main output
    TIMx->BDTR |= (1 << 15);

    // force update
    TIMx->EGR |= (1 << 0);

    // Enable counter
    TIMx->CR1 = (1 << 0);
}

// PWM signal generation with input frequency and duty cycle
void PWM_setDutyCycle(TIMx_TypeDef *TIMx, uint32_t note_freq, uint32_t duty_cycle, uint32_t psc_val) {
    // Set auto-reload register
    // freq_TIM = 80 MHz / (psc_val + 1)
    // arr_val = freq_TIM / note_freq - 1
    int arr_val = (note_freq > 0) ? ((80000000 / (psc_val+1)) / note_freq - 1) : 0;
    TIMx->ARR = arr_val;

    // Set capture/compare register 1
    TIMx->CCR1 = arr_val * duty_cycle / 100;

    // Force update
    TIMx->EGR |= (1 << 0);

    // Reset count
    TIMx->CNT = 0;
}