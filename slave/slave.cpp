/*
 * slave.cpp
 *
 *  Created on: Apr 19, 2023
 *      Author: liuchongyang
 */
#include <stdio.h>

#include "slave.h"

Slave::Slave(const uint8_t &dom, const offset_t &off)
{
	actual_position = nullptr;
	actual_torque = nullptr;
	status_word = nullptr;
	actual_velocity = nullptr;

	control_word = nullptr;
	target_position = nullptr;
	mode_of_operation = nullptr;
	target_torq = nullptr;

	control_word = (uint16_t *)(dom + off.ctrl_word);
	printf("control_word addr = %p\n", control_word);

	target_position = (int32_t *)(dom + off.target_position);
	printf("target_position addr = %p\n", target_position);

	mode_of_operation = (uint8_t *)(dom + off.operation_mode);
	printf("mode_of_operation addr = %p\n", mode_of_operation);

	actual_position = (int32_t *)(dom + off.position_actual_value);
	printf("actual_position addr = %p\n", actual_position);

	actual_torque = (int16_t *)(dom + off.actual_torque);
	printf("actual_torque addr = %p\n", actual_torque);

	status_word = (uint16_t *)(dom + off.status_word);
	printf("status_word addr = %p\n", status_word);
}
