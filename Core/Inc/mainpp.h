/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */

#ifndef MAINPP_H_
#define MAINPP_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"


union SendBuffer
{
	int16_t robot_wheels_pos[2];
	uint32_t robot_pos_msg;
};

union RcvBufffer
{
	int16_t robot_wheels_vel[2];
	uint32_t robot_vel_msg;
};

void setup(void);
void loop(void);

#ifdef __cplusplus
}
#endif


#endif /* MAINPP_H_ */
