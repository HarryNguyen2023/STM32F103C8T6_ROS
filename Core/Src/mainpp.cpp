/*
 * main.cpp

 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <mainpp.h>
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle nh;

std_msgs::UInt16 left_motor_pos;
std_msgs::UInt16 right_motor_pos;

volatile uint16_t left_motor_vel = 0;
volatile uint16_t right_motor_vel = 0;

// ******************* Motor Velocity Callback ************************* 
void leftCmdVelCallBack(const std_msgs::UInt16& left_vel)
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
	left_motor_vel = left_vel.data;
}

void rightCmdVelCallBack(const std_msgs::UInt16& right_vel)
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	right_motor_vel = right_vel.data;
}

// ************************** Declare subscribers **********************************
ros::Subscriber<std_msgs::UInt16> left_motor_vel_sub("left_motor_vel", &leftCmdVelCallBack);
ros::Subscriber<std_msgs::UInt16> right_motor_vel_sub("right_motor_vel", &rightCmdVelCallBack);

// ************************** Declare publishers **********************************
ros::Publisher left_motor_pub("left_motor_pos", &left_motor_pos);
ros::Publisher right_motor_pub("right_motor_pos", &right_motor_pos);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->flush();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  nh.getHardware()->reset_rbuf();
}

void setup(void)
{
  nh.initNode();
  // Initiate publishers and subscribers
  nh.advertise(left_motor_pub);
  nh.advertise(right_motor_pub);

  nh.subscribe(left_motor_vel_sub);
  nh.subscribe(right_motor_vel_sub);
}

void loop(void)
{
#ifdef STM32F4xx
  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
#endif
#ifdef STM32F3xx
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
#endif
  // Publish new data
  left_motor_pos.data = left_motor_vel;
  left_motor_pub.publish(&left_motor_pos);

  right_motor_pos.data = right_motor_vel;
  right_motor_pub.publish(&right_motor_pos);

  nh.spinOnce();
  HAL_Delay(20);
}

