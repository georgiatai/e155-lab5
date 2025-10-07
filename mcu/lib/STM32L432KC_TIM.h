/////////////////////////
// STM32L432KC_TIM.h
// Author: Georgia Tai, ytai@g.hmc.edu
// Date: Sep 27, 2025
// 
// Header file for STM32L432KC_TIM.c, including TIM struct.
/////////////////////////

#ifndef STM32L4_TIM_H
#define STM32L4_TIM_H

#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
// Definitions
///////////////////////////////////////////////////////////////////////////////

#define __IO volatile

// Base addresses
#define TIM15_BASE (0x40014000UL) // base address of TIM15
#define TIM16_BASE (0x40014400UL) // base address of TIM16

/**
  * @brief Reset and Clock Control
  */

typedef struct
{
  __IO uint32_t CR1;         /*!< TIMx control register 1,                             Address offset: 0x00 */
  __IO uint32_t CR2;         /*!< TIMx control register 2,                             Address offset: 0x04 */
  __IO uint32_t SMCR;        /*!< TIMx slave mode control register,                    Address offset: 0x08 */
  __IO uint32_t DIER;        /*!< TIMx DMA/interrupt enable register,                  Address offset: 0x0C */
  __IO uint32_t SR;          /*!< TIMx status register,                                Address offset: 0x10 */
  __IO uint32_t EGR;         /*!< TIMx event generation register,                      Address offset: 0x14 */
  __IO uint32_t CCMR1;       /*!< TIMx capture/compare mode register 1,                Address offset: 0x18 */
  __IO uint32_t CCMR2;       /*!< TIMx capture/compare mode register 2,                Address offset: 0x1C */
  __IO uint32_t CCER;        /*!< TIMx capture/compare enable register,                Address offset: 0x20 */
  __IO uint32_t CNT;         /*!< TIMx counter,                                        Address offset: 0x24 */
  __IO uint32_t PSC;         /*!< TIMx prescalar,                                      Address offset: 0x28 */
  __IO uint32_t ARR;         /*!< TIMx auto-related register,                          Address offset: 0x2C */
  __IO uint32_t RCR;         /*!< TIMx repetition counter register,                    Address offset: 0x30 */
  __IO uint32_t CCR1;        /*!< TIMx capture/compare register 1,                     Address offset: 0x34 */
  __IO uint32_t CCR2;        /*!< TIMx capture/compare register 2,                     Address offset: 0x38 */
  uint32_t      RESERVED1;   /*!< Reserved,                                            Address offset: 0x3C */
  uint32_t      RESERVED2;   /*!< Reserved,                                            Address offset: 0x40 */
  __IO uint32_t BDTR;        /*!< TIMx break and dead-time register,                   Address offset: 0x44 */
  __IO uint32_t DCR;         /*!< TIMx DMA control register,                           Address offset: 0x48 */
  __IO uint32_t DMAR;        /*!< TIMx DMA address for full transfer,                  Address offset: 0x4C */
  __IO uint32_t OR1;         /*!< TIMx option register 1,                              Address offset: 0x50 */
  uint32_t      RESERVED3;   /*!< Reserved,                                            Address offset: 0x54 */
  uint32_t      RESERVED4;   /*!< Reserved,                                            Address offset: 0x58 */
  uint32_t      RESERVED5;   /*!< Reserved,                                            Address offset: 0x5C */
  __IO uint32_t OR2;         /*!< TIMx option register 2,                              Address offset: 0x60 */

} TIMx_TypeDef; // TIMx struct for TIM15 and TIM16

#define TIM15 ((TIMx_TypeDef *) TIM15_BASE)
#define TIM16 ((TIMx_TypeDef *) TIM16_BASE)

///////////////////////////////////////////////////////////////////////////////
// Function prototypes
///////////////////////////////////////////////////////////////////////////////

void initTIM(TIMx_TypeDef *TIMx, uint32_t psc_val);
void delay_millis(TIMx_TypeDef * TIMx, uint32_t ms, uint32_t psc_val);
void initPWM(TIMx_TypeDef *TIMx, uint32_t psc_val);
void PWM_setDutyCycle(TIMx_TypeDef *TIMx, uint32_t note_freq, uint32_t duty_cycle, uint32_t psc_val);

#endif