#include "PID_motor.h"
#include "PID_motor_cfg.h"
#include <stdlib.h>

// Define some PID variables
uint16_t time_frame = 20;    // Time frame to mili-second

// ----------------------------------------------------- Static functions hidden from users ----------------------------------------------
// Function prototypes
static void outputSpeedPID(PID_motor* motor);
static void resetEncoder(PID_motor* motor);
static uint32_t readEncoder(PID_motor* motor);

// Function to get the output value of the PID speed controller
static void outputSpeedPID(PID_motor* motor)
{
    float error, output, prop;
    // Get number of the encoder pulse in the last time frame
    if(motor->direction == 0 && (motor->current_encoder < motor->prev_encoder) && (motor->prev_encoder - motor->current_encoder > 16000))
    {
        motor->real_speed = (65535 / 4) - motor->prev_encoder;
        motor->real_speed += motor->current_encoder;
    }
    else if(motor->direction == 1 && (motor->current_encoder > motor->prev_encoder) && (motor->current_encoder - motor->prev_encoder > 16000) )
    {
        motor->real_speed = motor->current_encoder - (65535 / 4);
        motor->real_speed -= motor->prev_encoder;
    }
    else
        motor->real_speed = motor->current_encoder - motor->prev_encoder;
    // Get the error of the number of encoder per time frame
    error = motor->targetPulsePerFrame - motor->real_speed;
    // Get the output of the PID controller with the new formula to avoid derivative kick as well as accumulation error when updating PID parameters
    prop = motor->speed_controller.Kp * error;
    motor->integral_error += motor->speed_controller.Ki * error;
    // Anti integral wind-up 
    if(motor->MAX_PWM > prop)
        motor->lim_max_integ = motor->MAX_PWM - prop;
    else 
        motor->lim_max_integ = 0;
    
    if(0 < prop)
        motor->lim_min_integ = 0 - prop;
    else
        motor->lim_min_integ = 0;
    // Constraint the integral
    if(motor->integral_error > motor->lim_max_integ)
        motor->integral_error = motor->lim_max_integ;
    else if(motor->integral_error < motor->lim_min_integ)
        motor->integral_error = motor->lim_min_integ;

    output = prop + motor->speed_controller.Kd * (motor->real_speed - motor->prev_encoder_feedback) + motor->integral_error;
    // Update the parameters
    motor->prev_encoder = motor->current_encoder;
    motor->prev_encoder_feedback = motor->real_speed;
    // Limit the output velocity of the motor
    if(output > motor->MAX_PWM)
        output = motor->MAX_PWM;
    else if(output < 0)
        output = 0;
    motor->output = output;
}

// Function to get the encoder value of the motor
static uint32_t readEncoder(PID_motor* motor)
{
    return motor->encoder_tim->CNT / 4;
}

// Function to reset the encoder value of the motor
static void resetEncoder(PID_motor* motor)
{
    motor->encoder_tim->CNT = 0;
}

// -------------------------------------------------------- General function used by users -----------------------------------------------

// Function to initiate the motor GPIO pins
void motorInit(PID_motor motor)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // Initiate the GPIO pins of the motor
    for(int i = 0; i < 2; ++i)
    {
        HAL_GPIO_WritePin(motor.motor_ports[i], motor.motor_pins[i], 0);
        if(motor.motor_ports[i] == GPIOA)
            __HAL_RCC_GPIOA_CLK_ENABLE();
        else if (motor.motor_ports[i] == GPIOB)
            __HAL_RCC_GPIOB_CLK_ENABLE();
        else if (motor.motor_ports[i] == GPIOC)
            __HAL_RCC_GPIOC_CLK_ENABLE();
        else if (motor.motor_ports[i] == GPIOD)
            __HAL_RCC_GPIOD_CLK_ENABLE();
        else if (motor.motor_ports[i] == GPIOE)
            __HAL_RCC_GPIOE_CLK_ENABLE();
        GPIO_InitStruct.Pin = motor.motor_pins[i];
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init(motor.motor_ports[i], &GPIO_InitStruct);
    }
}

// Function to update the PWM duty cycle of the motor
void dutyCycleUpdate(uint16_t duty_cycle, PID_motor* motor)
{
    switch (motor->pwm_channel)
    {
        case PWM_CHANNEL_1:
            motor->pwm_tim->CCR1 = duty_cycle;
            break;
        case PWM_CHANNEL_2:
            motor->pwm_tim->CCR2 = duty_cycle;
            break;
        case PWM_CHANNEL_3:
            motor->pwm_tim->CCR3 = duty_cycle;
            break;
        case PWM_CHANNEL_4:
            motor->pwm_tim->CCR4 = duty_cycle;
            break;
    default:
        break;
    }
}

// Function to brake the motor immediately
void motorBrake(PID_motor* motor)
{
    HAL_GPIO_WritePin(motor->motor_ports[0], motor->motor_pins[0], 0);
    HAL_GPIO_WritePin(motor->motor_ports[1], motor->motor_pins[1], 0);
}

// Function to reset the PID value of the motor when it is not moving
void resetPID(PID_motor* motor)
{
    // Re-initiate the PID parameters
    resetEncoder(motor);
    motor->current_encoder = readEncoder(motor);
    motor->prev_encoder = motor->current_encoder;
    motor->integral_error = 0;
    motor->output = 0;
    motor->prev_encoder_feedback = 0;
    motor->targetPulsePerFrame = 0.0;
    motor->moving = 0;
    motor->direction = 0;
    motor->real_speed = 0;
}

// Function to handle the speed input of the PID controller
void inputSpeedHandling(PID_motor* motor, float speed)
{
    // Rescale the input rpm speed
    if(speed > motor->MAX_INPUT_SPEED)
        speed = motor->MAX_INPUT_SPEED;
    else if(speed < 0)
        speed = 0;

    if(speed >= 0)
        motor->direction = 0;
    else
        motor->direction = 1;
    // Check whether the motor is already moving
    if(! motor->moving)
    {
        motor->moving = 1;
    }
    // Convert the desired speed to pulse per frame and input to the motor
    motor->targetPulsePerFrame = (speed * motor->encoder_rev) * time_frame / 60000.0;
    return;
}

// Function to control the speed of the motor by PID algorithm
void speedControlPID(PID_motor* motor)
{
    // Update the motor encoder value
    motor->current_encoder = readEncoder(motor);
    // Update the PID output of the controller
    outputSpeedPID(motor);

    // Get the absolute value of the motor
    uint16_t pwm_dutycycle = abs(motor->output);
    if(pwm_dutycycle < motor->DEAD_BAND)
        pwm_dutycycle = 0;

    // Control the direction of the motor
    if(motor->direction == 0)
    {
        HAL_GPIO_WritePin(motor->motor_ports[0], motor->motor_pins[0], 1);
        HAL_GPIO_WritePin(motor->motor_ports[1], motor->motor_pins[1], 0);
    }
    else if(motor->direction == 1)
    {
        HAL_GPIO_WritePin(motor->motor_ports[0], motor->motor_pins[0], 0);
        HAL_GPIO_WritePin(motor->motor_ports[1], motor->motor_pins[1], 1);
    }

    // Feed the value of the PWM duty cycle
    dutyCycleUpdate(pwm_dutycycle, motor);

    if(pwm_dutycycle == 0)
        motorBrake(motor);
}

