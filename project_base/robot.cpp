/*
 * robot.cpp
 *
 *  Created on: Apr 20, 2021
 *      Author: liuchongyang
 */
#include "robot.h"
#include "../slave/slave.h"
using namespace std;

extern std::vector<Slave> slave_vector;

Robot::Robot()
{
	axis_sum = 6;
	dh = DH_param();
	decare = Decare_Para();
	motor_param = Motor_Param();
	encoder_param = Encoder_Param();
	poweronstatus = false;
	tarjectory_prt = new Trajectory(this);
}
void Robot::set_DH_param(DH_param param_set)
{
	for(int i = 0;i < 6;i++)
	{
		dh.alpha[i] = param_set.alpha[i];
		dh.d[i] = param_set.d[i];
		dh.a[i] = param_set.a[i];
		dh.theta[i] = param_set.theta[i];
	}
}
DH_param Robot::get_DH_param()
{
	return dh;
}
void Robot::set_decare_param(Decare_Para param_set)
{
	decare.maxvel = param_set.maxvel;
	decare.maxacc = param_set.maxacc;
	decare.maxdec = param_set.maxdec;
	decare.maxjerk = param_set.maxjerk;
}
Decare_Para Robot::get_decare_param()
{
	return decare;
}
void Robot::set_motor_param(Motor_Param param_set)
{
	for(int i = 0;i < 6;i++)
	{
		motor_param.RatedVel_rpm[i] = param_set.RatedVel_rpm[i];
		motor_param.RatedVel[i] = param_set.RatedVel[i];
		motor_param.DeRatedVel[i] = param_set.DeRatedVel[i];
		motor_param.maxAcc[i] = param_set.maxAcc[i];
		motor_param.maxDecel[i] = param_set.maxDecel[i];
		motor_param.maxRotSpeed[i] = param_set.maxRotSpeed[i];
		encoder_param.direction[i] = param_set.encoder.direction[i];
		encoder_param.reducRatio[i] = param_set.encoder.reducRatio[i];
		encoder_param.encoderResolution[i] = param_set.encoder.encoderResolution[i];
		encoder_param.singleTurnEncoder[i] = param_set.encoder.singleTurnEncoder[i];
		encoder_param.deviation[i] = param_set.encoder.deviation[i];
	}
}
Motor_Param Robot::get_motor_param()
{
	motor_param.encoder = encoder_param;
	return motor_param;
}

void Robot::incToAngle(signed int inc,double& angle,int i)
{
	if(i >= axis_sum)
		return;
	angle = ( ( ((double)inc) * 360 / (1 << encoder_param.encoderResolution[i]) - encoder_param.singleTurnEncoder[i]) / encoder_param.reducRatio[i] / encoder_param.direction[i]);
}
void Robot::angleToInc(double angle,signed int& inc,int i)
{
	if(i >= axis_sum)
		return;
	inc = (angle * encoder_param.direction[i] * encoder_param.reducRatio[i] + encoder_param.singleTurnEncoder[i] + encoder_param.deviation[i]) * (1 << encoder_param.encoderResolution[i]) / 360.0;
}
void Robot::set_angle_deque(std::deque<double>& deque)
{
	angle_deque = deque;
}
std::deque<double>& Robot::get_angle_deque()
{
	return angle_deque;
}
unsigned int Robot::get_angle_deque_size()
{
	return angle_deque.size();
}
void Robot::power_on()
{
	for (unsigned int  i=0; i < axis_sum; i++)
	{
		//*(slave_vector[slave_num[i]].control_word) = 0;
		EC_WRITE_U16(slave_vector[slave_num[i]].control_word, 0 );
	}
	OsSleep(10);
	for (unsigned int  i=0; i < axis_sum; i++)
	{
		//*(slave_vector[slave_num[i]].control_word) = 0x80;
		EC_WRITE_U16(slave_vector[slave_num[i]].control_word, 0x80 );

	}
    OsSleep(10);
    int32_t actual_pos[axis_sum];
    for(unsigned int i = 0; i < axis_sum; i++)
    {
    	//actual_pos[i] = *(slave_vector[slave_num[i]].actual_position);
		actual_pos[i] = EC_READ_S32(slave_vector[slave_num[i]].actual_position);
		//*(slave_vector[slave_num[i]].target_position) = actual_pos[i];
		EC_WRITE_S32(slave_vector[slave_num[i]].target_position, actual_pos[i] ); 
    }

    for(int i = 0;i < axis_sum;i++)
    {
		//*(slave_vector[slave_num[i]].control_word) = 6;
		EC_WRITE_U16(slave_vector[slave_num[i]].control_word, 0x06 );
		//*(slave_vector[slave_num[i]].target_position) = actual_pos[i];
		EC_WRITE_S32(slave_vector[slave_num[i]].target_position, actual_pos[i] ); 
    }
    OsSleep(1000);

    for(int i=0;i<axis_sum;i++)
    {
		//*(slave_vector[slave_num[i]].control_word) = 7;
		EC_WRITE_U16(slave_vector[slave_num[i]].control_word, 0x07 );
		//*(slave_vector[slave_num[i]].target_position) = actual_pos[i];
		EC_WRITE_S32(slave_vector[slave_num[i]].target_position, actual_pos[i] );
    }
    OsSleep(1000);

    for(int i=0;i<axis_sum;i++)
    {
		//*(slave_vector[slave_num[i]].control_word) = 15;
		EC_WRITE_U16(slave_vector[slave_num[i]].control_word, 0x0f );
		//*(slave_vector[slave_num[i]].target_position) = actual_pos[i];
		EC_WRITE_S32(slave_vector[slave_num[i]].target_position, actual_pos[i] );
    }
    poweronstatus = true;
	return;
}
void Robot::power_off()
{
    int32_t actual_pos[axis_sum];

    for(unsigned int i = 0; i < axis_sum; i++)
    {
    	//actual_pos[i] = *(slave_vector[slave_num[i]].actual_position);
		actual_pos[i] = EC_READ_S32(slave_vector[slave_num[i]].actual_position);
    }

    for(int i = 0;i < axis_sum;i++)
    {
		//*(slave_vector[slave_num[i]].control_word) = 7;
		EC_WRITE_U16(slave_vector[slave_num[i]].control_word, 0x07 );
		//*(slave_vector[slave_num[i]].target_position) = actual_pos[i];
		EC_WRITE_S32(slave_vector[slave_num[i]].target_position, actual_pos[i] ); 
    }
	OsSleep(1000);

	for(int i=0;i<axis_sum;i++)
	{
		//*(slave_vector[slave_num[i]].control_word) = 6;
		EC_WRITE_U16(slave_vector[slave_num[i]].control_word, 0x06 );
		//*(slave_vector[slave_num[i]].target_position) = actual_pos[i];
		EC_WRITE_S32(slave_vector[slave_num[i]].target_position, actual_pos[i] );
    }
	poweronstatus = false;
	return;
}
double Robot::get_actual_position(int axis)
{
	if(axis >= axis_sum)
		return -1;
	//EC_T_SDWORD actual_pos = *(slave_vector[slave_num[axis]].actual_position);
	int32_t actual_pos = EC_READ_S32(slave_vector[slave_num[i]].actual_position);
	double angle = 0;
	incToAngle(actual_pos, angle, axis);
	return angle;
}
void Robot::set_target_position(int axis, double targetPosition)
{
	if(axis >= axis_sum)
		return;
	int32_t target_pos_inc;
	angleToInc(targetPosition, target_pos_inc, axis);
	EC_WRITE_S32(slave_vector[slave_num[i]].target_position, target_pos_inc);
	//(*slave_vector[slave_num[axis]].target_position) = target_pos_inc;
}
bool Robot::get_power_on_status()
{
	return poweronstatus;
}
int16_t Robot::get_actual_torque(int axis)
{
	if(axis >= axis_sum)
		return -1;
	//int16_t actual_torq = *(slave_vector[slave_num[axis]].actual_torque);
	int16_t actual_torq = EC_READ_S16(slave_vector[slave_num[axis]].actual_torque);
	return actual_torq;
}
int Robot::get_status_word(int axis)
{
	if(axis >= axis_sum)
		return -1;
	//EC_T_WORD status_word = *(slave_vector[slave_num[axis]].status_word);
	uint16_t status_word = EC_READ_U16(slave_vector[slave_num[axis]].status_word);
	return status_word;
}
int Robot::get_actual_velocity(int axis)
{
	if(axis >= axis_sum)
		return -1;
	//EC_T_SDWORD actual_vel = *(slave_vector[slave_num[axis]].actual_velocity);
	int32_t actual_vel = EC_READ_S32(slave_vector[slave_num[axis]].actual_velocity);

	return actual_vel;
}
void Robot::set_target_torque(int axis,int targetTorque)
{
	if(axis >= axis_sum)
		return;
	//(*slave_vector[slave_num[axis]].target_torq) = targetTorque;
	EC_WRITE_S16(slave_vector[slave_num[axis]].target_torq, targetTorque);
}
void Robot::move_joint_interp(const VectorXd &targetPoint,
		const VectorXd &originPoint, const VectorXd &velCurrent, const VectorXd &accCurrent, double Ts, double velPerc,
		double accPerc, double decPerc, double jerkPerc,std::deque<double> &nAglSeqPtr)
{
	tarjectory_prt->move_joint_interp(targetPoint,
			originPoint, velCurrent, accCurrent, Ts, velPerc,
			accPerc, decPerc, jerkPerc, nAglSeqPtr);
}
void Robot::move_line_interp(const VectorXd &targetPoint,
		const VectorXd &originPoint, const VectorXd &originACS, double velCurrent, double accCurrent,
		double Ts, double maxVelL, double maxAccL, double maxDecelL,
		double maxJerk, std::deque<double> &nAglSeqPtr)
{
	tarjectory_prt->move_line_interp(targetPoint,
			originPoint, originACS, velCurrent, accCurrent,
			Ts, maxVelL, maxAccL, maxDecelL,
			maxJerk, nAglSeqPtr);
}
void Robot::cycle_run()
{
	if(!angle_deque.empty()&& poweronstatus)
	{
		double temp_pos[axis_sum];

		for(int i=0;i<axis_sum;i++)
		{
			temp_pos[i] = angle_deque.front();
			//2024.4.11增加各轴限位
			if(i==1 && (temp_pos[1]<-70 || temp_pos[1]> 70))
			{
				return;
			}
			if(i==2 && (temp_pos[2]<-70 || temp_pos[2]> 70))
			{
				return;
			}
			if(i==3 && (temp_pos[3]<-70 || temp_pos[2]> 70))
			{
				return;
			}
			if(i==4 && (temp_pos[4]<-70 || temp_pos[4]> 70))
			{
				return;
			}

			angle_deque.pop_front();
			set_target_position(i, temp_pos[i]);
		}
	}
}
