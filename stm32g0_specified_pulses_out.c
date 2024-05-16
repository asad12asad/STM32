/**
  ******************************************************************************
  * @file    specified_pulses_out.c
  * @author  asadshafi
  * @brief   Description of the file.
  *
  * This C file contains functions to generate a specific number of pulses using 
  * Timer 14 (TIM14) and Timer 3 (TIM3) peripherals on an STM32G030
  * microcontroller.
  * TIM3_IRQHandler():
  * - Interrupt service routine for Timer 3 update event.
  * Perform relevant logic to stop pulses 
  * @note    Additional notes or considerations.
  ******************************************************************************
  */


#include "stm32g030xx.h"
#include "stm32g0xx.h"
#include <stdio.h>
#include <stdlib.h>

uint16_t prescaler = 700 - 1; // Set prescaler for desired clock
uint16_t arr_value = 100 - 1; // Set auto-reload value for desired frequency 
uint16_t duty_cycle = 50;     // Set duty cycle to 50% (0-1000 represents 0-100%)
/**
  * @brief  Initializes Timer 3 to count pulses.
  * @param  pulse_count: Desired number of pulses to count.
  * @retval None
  */
void tim3_pulse_counter_init(uint16_t pulse_count)
{
    // 1. Enable TIM3 and TIM14 clocks
    RCC->APBENR1 |= RCC_APBENR1_TIM3EN;
    // RCC->APBENR1 |= RCC_APBENR1_TIM14EN;

    // 2. Configure TIM3 in slave mode with ITR3 as trigger input
    TIM3->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0 | TIM_SMCR_TS_0 | TIM_SMCR_TS_1; // Trigger mode, ITR2 selected

    // 3. Configure TIM3 to count upon each trigger
    TIM3->CR1 |= TIM_CR1_CEN; // Enable TIM3 counter

    // 4. Set the auto-reload value to the desired pulse count
    TIM3->ARR = pulse_count - 1;

    // 5. Enable update interrupt for TIM3
    TIM3->DIER |= TIM_DIER_UIE;

    // 6. Configure NVIC for TIM3 interrupt
    NVIC_EnableIRQ(TIM3_IRQn);

    // 7. Start TIM14
    TIM14->CR1 |= TIM_CR1_CEN;
}
/**
  * @brief  Initializes Timer 14 for PWM generation.
  * @retval None
  */
void tim14_init()
{

    // Enable TIM14 clock
    RCC->APBENR2 |= RCC_APBENR2_TIM14EN;

    // Configure TIM14
    TIM14->PSC = prescaler;
    TIM14->ARR = arr_value;
    TIM14->CCMR1 &= ~TIM_CCMR1_OC1M;                     // Clear output compare mode
    TIM14->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // Set to PWM mode 1
    TIM14->CCR1 = duty_cycle;                            // Set compare value for duty cycle
    TIM14->CCER |= TIM_CCER_CC1E;                        // Enable capture/compare output
    TIM14->EGR |= TIM_EGR_UG;                            // Generate update event to apply settings

    // Enable TIM14 counter
    TIM14->CR1 |= TIM_CR1_CEN;
}
void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF)
    {
        // Pulse count reached, perform actions here
        TIM14->CR1 &= ~TIM_CR1_CEN;

        TIM3->SR &= ~TIM_SR_UIF; // Clear update interrupt flag
    }
}
