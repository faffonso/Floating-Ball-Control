#ifndef ACTUATORS_H  
#define ACTUATORS_H

/**
 * @file actuators.h
 * @brief Implementation of the actuators driver
 *
 * This file contains the implementation of the driver for an H-bridge motor controller. 
 * The driver initializes GPIO pins for motor control, configures PWM for motor speed control,
 * and provides functions to update the motor speed and direction. The H-bridge motor driver
 * can control motors connected to an H-bridge motor driver circuit, which allows bidirectional
 * control of DC motors.
 *
 * Authors: Francisco Affonso Pinto
 *          
 */

/* Bib */
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

/* Macros */

//ESP-LOG Tag
#define TAG "Actuators"

//GPIOs levels
#define LOW     0
#define HIGH    1

// GPIOs of H-Bridge
#define H_BRIDGE_INPUT_1    GPIO_NUM_18
#define H_BRIDGE_INPUT_2    GPIO_NUM_19

// PWM Config 
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT   // Set duty resolution to 13 bits (MAX=8192)
#define LEDC_FREQUENCY          (4000)              // Frequency of 4kHz

#define LEDC_OUTPUT             GPIO_NUM_32
#define LEDC_CHANNEL            LEDC_CHANNEL_0



/* Functions */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize GPIOs
 * 
 * Initialize h-bridge pins
 * 
 * @return esp_err_t
 */
esp_err_t init_gpio();

/**
 * @brief Initialize PWM configuration
 * 
 * Initialize PWM configuration of timers and channels using ledc
 * 
 * @return esp_err_t 
 */
esp_err_t init_pwm();

/**
 * @brief Update motor speed and direction
 * 
 * Update motor speed and diretion by using the motor we want to change and applying action control
 * 
 * @param u Action control (Angular Velocity)
 * @param debug Print action control
 * @return esp_err_t 
 */
esp_err_t update_motor(int u, int debug);

/**
 * @brief Set h-bridge for clockwise rotation
 * 
 * Set input pins on the h-bridge to rotate the wheel clockwise
 * 
 * @return esp_err_t 
 */
esp_err_t _set_forward();

/**
 * @brief Set h-bridge for counter-clockwise rotation
 * 
 * Set input pins on the h-bridge to rotate the wheel counter clockwise
 * @return esp_err_t 
 */
esp_err_t _set_backward();

#ifdef __cplusplus
}
#endif

#endif