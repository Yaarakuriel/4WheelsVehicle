/**
 ******************************************************************************
 * @file    motor_driver.h
 * @brief   Motor driver and encoder interface
 ******************************************************************************
 * @attention
 * Pure user code extracted and modularized from main.c
 * Provides motor control (direction + PWM) and encoder handling as a reusable
 * C module. Supports cases where encoder A/B pins may be on different ports.
 ******************************************************************************
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

/* -------------------------- Data Types ----------------------------------- */

/**
 * @brief MotorDriver object structure
 *
 * Encapsulates:
 * - PWM timer + channel for motor speed
 * - GPIO ports/pins for motor direction (A/B)
 * - Encoder timer (hardware quadrature decoder)
 * - GPIO ports/pins for encoder A/B signals
 * - Runtime state: position counter and speed
 */
typedef struct {
    /* PWM config */
    TIM_TypeDef   *pwm_tim;       /*!< Timer instance used for PWM (e.g. TIM2) */
    uint32_t       pwm_channel;   /*!< PWM channel (e.g. TIM_CHANNEL_2) */

    /* Encoder config */
    TIM_TypeDef   *enc_tim;       /*!< Timer instance used for encoder (e.g. TIM1) */
    GPIO_TypeDef  *encA_port;     /*!< GPIO port for encoder channel A */
    uint16_t       encA_pin;      /*!< GPIO pin for encoder channel A */
    GPIO_TypeDef  *encB_port;     /*!< GPIO port for encoder channel B */
    uint16_t       encB_pin;      /*!< GPIO pin for encoder channel B */

    /* Motor direction pins */
    GPIO_TypeDef  *dir_port;      /*!< GPIO port for motor A/B pins */
    uint16_t       pin_A;         /*!< GPIO pin for motor A */
    uint16_t       pin_B;         /*!< GPIO pin for motor B */

    /* Runtime state */
    volatile int32_t position;    /*!< Accumulated encoder position (pulses) */
    volatile int32_t speed_pps;   /*!< Speed in pulses per second */
    int16_t          last_cnt;    /*!< Previous encoder counter snapshot */
    int16_t          curr_cnt;    /*!< Current encoder counter snapshot */
} MotorDriver;

/* -------------------------- API Functions -------------------------------- */

/**
 * @brief  Initialize the MotorDriver object.
 * @param  md Pointer to MotorDriver instance
 * @param  pwm_tim Timer for PWM
 * @param  pwm_channel Timer channel for PWM
 * @param  enc_tim Timer for encoder
 * @param  dir_port GPIO port for motor A/B pins
 * @param  pin_A Motor direction pin A
 * @param  pin_B Motor direction pin B
 * @param  encA_port GPIO port for encoder channel A
 * @param  encA_pin GPIO pin for encoder channel A
 * @param  encB_port GPIO port for encoder channel B
 * @param  encB_pin GPIO pin for encoder channel B
 */
void MotorDriver_Init(MotorDriver *md,
                      TIM_TypeDef *pwm_tim, uint32_t pwm_channel,
                      TIM_TypeDef *enc_tim,
                      GPIO_TypeDef *dir_port, uint16_t pin_A, uint16_t pin_B,
                      GPIO_TypeDef *encA_port, uint16_t encA_pin,
                      GPIO_TypeDef *encB_port, uint16_t encB_pin);

/**
 * @brief  Stop motor (coast, free-wheel).
 */
void MotorDriver_Stop(MotorDriver *md);

/**
 * @brief  Drive motor clockwise.
 * @param  duty PWM duty cycle [0..ARR]
 */
void MotorDriver_CW(MotorDriver *md, int duty);

/**
 * @brief  Drive motor counter-clockwise.
 * @param  duty PWM duty cycle [0..ARR]
 */
void MotorDriver_CCW(MotorDriver *md, int duty);

/**
 * @brief  Actively brake motor (short brake).
 */
void MotorDriver_Brake(MotorDriver *md);

/**
 * @brief  Update encoder counts and compute speed.
 *         Call this from 1ms SysTick or timer interrupt.
 */
void MotorDriver_EncoderUpdate(MotorDriver *md);

#endif /* MOTOR_DRIVER_H */
