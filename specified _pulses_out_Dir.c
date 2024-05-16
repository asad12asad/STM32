/**
  ******************************************************************************
  * @file    specified_pulses_out_dir.c
  * @author  asadshafi
  * @brief   Description of the file.
  *
  * This C file contains functions to generate a specific number of pulses using 
  * Timer 14 (TIM14) and Timer 3 (TIM3) peripherals on an STM32G030
  * microcontroller. with direction 
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

    // Enable TIM14 counter
    // TIM14->CR1 |= TIM_CR1_CEN;
}
void tim3_pulse_counter_init(uint16_t pulse_count)
{
    // 1. Enable TIM3 and TIM14 clocks
    RCC->APBENR1 |= RCC_APBENR1_TIM3EN;

    // 4. Configure TIM3 in slave mode with ITR2 as trigger input
    TIM3->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0 | TIM_SMCR_TS_0 | TIM_SMCR_TS_1; // Trigger mode, ITR2 selected

    // 5. Configure TIM3 to count up on each trigger
    TIM3->CR1 |= TIM_CR1_CEN; // Enable TIM3 counter

    // 6. Set the auto-reload value to the desired pulse count
    TIM3->ARR = pulse_count - 1;

    // 7. Enable update interrupt for TIM3
    TIM3->DIER |= TIM_DIER_UIE;

    // 8. Configure NVIC for TIM3 interrupt
    NVIC_EnableIRQ(TIM3_IRQn);

    // 9. Start TIM14
    // TIM14->CR1 |= TIM_CR1_CEN;
}

char cnt = 0;
// TIM3 interrupt handler
void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF)
    {
        if (cnt == 0)
        {
            GPIOB->ODR &= ~GPIO_ODR_OD4;
        }
        else
        {

            GPIOB->ODR |= GPIO_ODR_OD4;
        }

        if (cnt == 1)
        {
            TIM14->CR1 &= ~TIM_CR1_CEN; // reset
            cnt = -1;
            motor_en_low;
        }
        cnt++;
        // Pulse count reached, perform actions here
        // ...

        GPIOA->ODR ^= (0x7f & ~sevenSegment[2]);
        TIM3->SR &= ~TIM_SR_UIF; // Clear update interrupt flag
    }
}

void GPIOA_init()
{
    /***********GPIOA segment init***********/
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;

    /*motor enable*/
    GPIOB->MODER &= ~(GPIO_MODER_MODE8_0 | GPIO_MODER_MODE8_1); // en
    GPIOB->MODER |= (GPIO_MODER_MODE8_0);                       // en

    // Configure PA7 as an alternate function mode this is clock out
    GPIOA->MODER &= ~(GPIO_MODER_MODE7_0 | GPIO_MODER_MODE7_1); // Clear mode bits
    GPIOA->MODER |= GPIO_MODER_MODE7_1;                         // Set to alternate function mode

    // Set PA7 to AF4 (TIM14_CH1)
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL7);           // Clear alternate function selection bits
    GPIOA->AFR[0] |= (4UL << GPIO_AFRL_AFSEL7_Pos); // Set to AF4

    
    GPIOB->MODER &= ~(GPIO_MODER_MODE4_0 | GPIO_MODER_MODE4_1); // Dir
    GPIOB->MODER |= (GPIO_MODER_MODE4_0);                       // Dir
}
char s = 0;
int time = 0;
char statmotor;


int main(void)
{
    GPIOA_init();
    tim14_init();
    tim3_pulse_counter_init(50);// total number of out 100 (50 Forward and 50 Reverse)

    int i;
    while (1)
    {
        if (statmotor == 1)// start pulse out
        {
            motor_en_hi;
            TIM14->CR1 |= TIM_CR1_CEN;
            statmotor = 0;
        }
        for (i = 0; i < 1000000; i++)
        {
        }
        statmotor = 1;
    }
}
