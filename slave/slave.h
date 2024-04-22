/*
 * slave.h
 *
 *  Created on: Apr 19, 2023
 *      Author: liuchongyang
 */

#ifndef OPENRC_SLAVE_SLAVE_H_
#define OPENRC_SLAVE_SLAVE_H_

#include <vector>

#include "../project_base/EcMasterApi.h"
#include "../ec_config.h"

class Slave
{
public:
	Slave(const uint8_t &dom, const offset_t &off);

	//EC_T_WORD*                 	 word_0x6061;        //存储错误码地址

	uint16_t*                 	 error_code;        //存储错误码地址
	int32_t*                 actual_position;        //存储当前位置地址
	int16_t*                  actual_torque;        //存储转矩实际值地址
	uint16_t*                   status_word;        //存储状态字地址
	int32_t*                 actual_velocity;        //存储速度实际值地址

	uint16_t*                   control_word;        //存储控制字地址
	int32_t*                 target_position;     //存储目标位置地址
	uint8_t*                   mode_of_operation;   //存储操作模式地址
	int16_t*                  target_torq;   //存储目标转矩地址
};



#endif /* OPENRC_SLAVE_SLAVE_H_ */
