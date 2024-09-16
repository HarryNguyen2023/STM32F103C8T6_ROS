#include <PID_motor.h>
#include <PID_motor_cfg.h>

#include <stdlib.h>

// Define some PID variables
uint16_t time_frame = 20;    // Time frame to mili-second

// ----------------------------------------------------- Static functions hidden from users ----------------------------------------------
// Function prototypes
static void outputSpeedPID(PID_motor* motor);
static uint32_t readEncoder(PID_motor* motor);

// Function to get the output value of the PID speed controller
static void outputSpeedPID(PID_motor* motor)
{
    float error, output, prop;
    // Get number of the encoder pulse in the last time frame
    if(motor->direction == CCW_DIRECTION && (motor->current_encoder < motor->prev_encoder) && (motor->prev_encoder - motor->current_encoder > 16000))
    {
        motor->real_speed = (65535 / 4) - motor->prev_encoder;
        motor->real_speed += motor->current_encoder;
    }
    else if(motor->direction == CW_DIRECTION && (motor->current_encoder > motor->prev_encoder) && (motor->current_encoder - motor->prev_encoder > 16000) )
    {
        motor->real_speed = motor->current_encoder - (65535 / 4);
        motor->real_speed -= motor->prev_encoder;
    }
    else
        motor->real_speed = motor->current_encoder - motor->prev_encoder;

    // Check the state of the motor
	if(motor->moving == MOTOR_STOP && motor->real_speed == 0 && motor->targetPulsePerFrame != 0)
	{
		motor->moving = MOTOR_MOVING;
		motor->integral_error = 0;
	}

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
    
    if(prop > 0)
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
    else if(output < - motor->MAX_PWM)
        output = - motor->MAX_PWM;
    motor->output = output;
}

// Function to get the encoder value of the motor
static uint32_t readEncoder(PID_motor* motor)
{
    return motor->encoder_tim->CNT / 4;
}

// -------------------------------------------------------- General function used by users -----------------------------------------------

// Function to initiate the motor GPIO pins
void motorInit(PID_motor motor)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // Initiate the GPIO pins of the motor
    for(int i = 0; i < 2; ++i)
    {
        HAL_GPIO_WritePin(motor.motor_ports[i], motor.motor_pins[i], GPIO_PIN_RESET);
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
	dutyCycleUpdate(0, motor);
    HAL_GPIO_WritePin(motor->motor_ports[0], motor->motor_pins[0], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->motor_ports[1], motor->motor_pins[1], GPIO_PIN_RESET);
}

// Function to handle the speed input of the PID controller
void inputSpeedHandling(PID_motor* motor, float speed)
{
    // Convert speed rpm into rad/s
    if(speed > motor->MAX_INPUT_SPEED)
        speed = motor->MAX_INPUT_SPEED;
    else if(speed < - motor->MAX_INPUT_SPEED)
        speed = - motor->MAX_INPUT_SPEED;
    else if(speed == 0)
    	motor->moving = MOTOR_STOP;

    // Check whether the motor is already moving
    if(motor->moving == MOTOR_STOP && speed != 0)
    {
        motor->moving = MOTOR_MOVING;
    }

    // Check if the motor change direction immediately & stop the motor before changing direction
    if(speed >= 0)
    {
    	if(motor->direction == CW_DIRECTION)
    	{
    		motor->moving = MOTOR_STOP;
    		motorBrake(motor);
    	}
        motor->direction = CCW_DIRECTION;
    }
    else
    {
    	if(motor->direction == CCW_DIRECTION)
    	{
    		motor->moving = MOTOR_STOP;
    		motorBrake(motor);
    	}
        motor->direction = CW_DIRECTION;
    }

    // Convert the desired speed to pulse per frame and input to the motor
    motor->targetPulsePerFrame = (speed * motor->encoder_rev) * time_frame / 60000.0;
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
    if(motor->moving == MOTOR_MOVING)
    {
    	if(motor->direction == CCW_DIRECTION)
		{
			HAL_GPIO_WritePin(motor->motor_ports[0], motor->motor_pins[0], GPIO_PIN_SET);
			HAL_GPIO_WritePin(motor->motor_ports[1], motor->motor_pins[1], GPIO_PIN_RESET);
		}
		else if(motor->direction == CW_DIRECTION)
		{
			HAL_GPIO_WritePin(motor->motor_ports[0], motor->motor_pins[0], GPIO_PIN_RESET);
			HAL_GPIO_WritePin(motor->motor_ports[1], motor->motor_pins[1], GPIO_PIN_SET);
		}
    }

    // Feed the value of the PWM duty cycle
    if(pwm_dutycycle == 0 || motor->moving == MOTOR_STOP)
        motorBrake(motor);
    else if(pwm_dutycycle != 0 && motor->moving == MOTOR_MOVING)
    	dutyCycleUpdate(pwm_dutycycle, motor);
}

