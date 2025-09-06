/**
 ******************************************************************************
 * @file    motor_driver.c
 * @brief   Implementation of motor driver and encoder interface
 ******************************************************************************
 */

#include "motor_driver.h"
#include "stm32h7xx_hal.h"   // HAL types & TIM register defs

/* -------------------------------------------------------------------------- */
/*                            Internal PWM helpers                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Return pointer to the proper CCR register for the configured channel.
 * @note   Returns NULL if channel is invalid for this timer.
 */
static inline volatile uint32_t *MD_GetCCR(MotorDriver *md)
{
    switch (md->pwm_channel) {
    case TIM_CHANNEL_1: return &md->pwm_tim->CCR1;
    case TIM_CHANNEL_2: return &md->pwm_tim->CCR2;
    case TIM_CHANNEL_3: return &md->pwm_tim->CCR3;
    case TIM_CHANNEL_4: return &md->pwm_tim->CCR4;
    default:            return NULL;
    }
}

/**
 * @brief  Write duty to the configured PWM channel, clamped to [0..ARR].
 * @param  duty  Raw compare value (same scale as timer ARR)
 */
static inline void MD_WriteDuty(MotorDriver *md, int duty)
{
    volatile uint32_t *ccr = MD_GetCCR(md);
    if (!ccr) return;                 // invalid channel; nothing to do

    int arr = (int)md->pwm_tim->ARR;  // timer auto-reload value
    if (duty < 0)   duty = 0;
    if (duty > arr) duty = arr;

    *ccr = (uint32_t)duty;
}

/* -------------------------------------------------------------------------- */
/*                                  API                                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialize MotorDriver object.
 *         Resets counters and binds struct to real hardware peripherals.
 */
void MotorDriver_Init(MotorDriver *md,
                      TIM_TypeDef *pwm_tim, uint32_t pwm_channel,
                      TIM_TypeDef *enc_tim,
                      GPIO_TypeDef *dir_port, uint16_t pin_A, uint16_t pin_B,
                      GPIO_TypeDef *encA_port, uint16_t encA_pin,
                      GPIO_TypeDef *encB_port, uint16_t encB_pin)
{
    /* Assign config */
    md->pwm_tim     = pwm_tim;
    md->pwm_channel = pwm_channel;
    md->enc_tim     = enc_tim;

    md->dir_port    = dir_port;
    md->pin_A       = pin_A;
    md->pin_B       = pin_B;

    md->encA_port   = encA_port;
    md->encA_pin    = encA_pin;
    md->encB_port   = encB_port;
    md->encB_pin    = encB_pin;

    /* Reset runtime state */
    md->position    = 0;
    md->speed_pps   = 0;
    md->last_cnt    = 0;
    md->curr_cnt    = 0;

    /* Reset hardware encoder counter */
    if (md->enc_tim != NULL) {
        md->enc_tim->CNT = 0;
    }

    /* Ensure PWM starts at 0 */
    MD_WriteDuty(md, 0);
}

/**
 * @brief  Coast motor (disable outputs, free-wheel).
 */
void MotorDriver_Stop(MotorDriver *md)
{
    HAL_GPIO_WritePin(md->dir_port, md->pin_A, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(md->dir_port, md->pin_B, GPIO_PIN_RESET);
    MD_WriteDuty(md, 0);
}

/**
 * @brief  Drive motor clockwise (A=0, B=1).
 * @param  duty PWM duty cycle [0..ARR]
 */
void MotorDriver_CW(MotorDriver *md, int duty)
{
    HAL_GPIO_WritePin(md->dir_port, md->pin_A, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(md->dir_port, md->pin_B, GPIO_PIN_SET);
    MD_WriteDuty(md, duty);
}

/**
 * @brief  Drive motor counter-clockwise (A=1, B=0).
 * @param  duty PWM duty cycle [0..ARR]
 */
void MotorDriver_CCW(MotorDriver *md, int duty)
{
    HAL_GPIO_WritePin(md->dir_port, md->pin_A, GPIO_PIN_SET);
    HAL_GPIO_WritePin(md->dir_port, md->pin_B, GPIO_PIN_RESET);
    MD_WriteDuty(md, duty);
}

/**
 * @brief  Short-brake motor (A=1, B=1, PWM=0).
 */
void MotorDriver_Brake(MotorDriver *md)
{
    HAL_GPIO_WritePin(md->dir_port, md->pin_A, GPIO_PIN_SET);
    HAL_GPIO_WritePin(md->dir_port, md->pin_B, GPIO_PIN_SET);
    MD_WriteDuty(md, 0);
}

/**
 * @brief  Update encoder counts and speed.
 *         To be called at fixed intervals (e.g. SysTick 1ms).
 */
void MotorDriver_EncoderUpdate(MotorDriver *md)
{
    /* Read current counter value (hardware 16-bit on many timers) */
    md->curr_cnt = (int16_t)(md->enc_tim->CNT);

    /* Compute signed delta since last read */
    int16_t diff = md->curr_cnt - md->last_cnt;
    md->last_cnt = md->curr_cnt;

    /* Handle 16-bit wrap/underflow */
    if (diff >  0x7FFF) diff -= 0x10000;
    if (diff < -0x7FFF) diff += 0x10000;

    /* Accumulate and compute speed (pps) for 1ms tick */
    md->position += diff;
    md->speed_pps = diff * 1000;
}
