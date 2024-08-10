/*
 * main.cpp

 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>
#include <PID_motor.h>
#include <PID_motor_cfg.h>

#include <ros.h>
#include <std_msgs/UInt32.h>


ros::NodeHandle nh;

std_msgs::UInt32 robot_wheels_pos;

SendBuffer send_buffer;
RcvBufffer rcv_buffer;

volatile uint8_t tick = 0;
static int16_t left_enc_ticks = 0;
static int16_t right_enc_ticks = 0;

// ******************* Motor Velocity Callback ************************* 
void robotCmdVelCallBack(const std_msgs::UInt32& robot_vel)
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	rcv_buffer.robot_vel_msg = robot_vel.data;
	inputSpeedHandling(&motor_left, (float)rcv_buffer.robot_wheels_vel[0]);
	inputSpeedHandling(&motor_right, (float)rcv_buffer.robot_wheels_vel[1]);
}

// ************************** Declare subscribers **********************************
ros::Subscriber<std_msgs::UInt32> robot_vel_sub("robot_wheel_vel", &robotCmdVelCallBack);

// ************************** Declare publishers **********************************
ros::Publisher robot_pos_pub("robot_wheel_pos", &robot_wheels_pos);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

// Function for timer interrupt callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	speedControlPID(&motor_left);
	speedControlPID(&motor_right);

	left_enc_ticks += motor_left.real_speed;
	right_enc_ticks += motor_right.real_speed;

	// Publish new data every 100 ms
	if(++tick == 5)
	{
		tick = 0;
		send_buffer.robot_wheels_pos[0] = left_enc_ticks;
		send_buffer.robot_wheels_pos[1] = right_enc_ticks;
		robot_wheels_pos.data = send_buffer.robot_pos_msg;
		robot_pos_pub.publish(&robot_wheels_pos);
		// Reset the encoders tick
		left_enc_ticks = 0;
		right_enc_ticks = 0;
	}
}

void setup(void)
{
  nh.initNode();
  // Initiate publishers and subscribers
  nh.advertise(robot_pos_pub);
  nh.subscribe(robot_vel_sub);

  motorInit(motor_left);
  motorInit(motor_right);

  inputSpeedHandling(&motor_left, 50.0);
  inputSpeedHandling(&motor_right, 50.0);
}

void loop(void)
{
#ifdef STM32F4xx
  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
#endif
#ifdef STM32F3xx
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
#endif

  nh.spinOnce();
}

