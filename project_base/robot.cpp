/*
 * robot.cpp
 *
 *  Created on: Apr 20, 2021
 *      Author: liuchongyang
 */
#include "robot.h"
#include "../slave/slave.h"
using namespace std;

#define     ETHERCAT_STATUS_OP                0x08

extern std::vector<Slave> slave_vector;

Robot::Robot()
{
	axis_sum = Number;
	dh = DH_param();
	decare = Decare_Para();
	motor_param = Motor_Param();
	encoder_param = Encoder_Param();
	poweronstatus = false;
	tarjectory_prt = new Trajectory(this);
}
void Robot::set_DH_param(DH_param param_set)
{
	for(int i = 0;i < Number;i++)
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
	//angle = ( ( ((double)inc) * 360 / (1 << encoder_param.encoderResolution[i]) - encoder_param.singleTurnEncoder[i]) / encoder_param.reducRatio[i] / encoder_param.direction[i]);
	angle = (double)(((inc - encoder_param.singleTurnEncoder[i]) % encoder_param.encoderResolution[i]) / (encoder_param.direction[i] * (encoder_param.encoderResolution[i] / 360.0)));
}
void Robot::angleToInc(double angle,signed int& inc,int i)
{
	if(i >= axis_sum)
		return;
	//inc = (angle * encoder_param.direction[i] * encoder_param.reducRatio[i] + encoder_param.singleTurnEncoder[i] + encoder_param.deviation[i]) * (1 << encoder_param.encoderResolution[i]) / 360.0;
	inc = ((int)(angle * encoder_param.direction[i] * (encoder_param.encoderResolution[i] / 360.0)) + encoder_param.singleTurnEncoder[i]) % encoder_param.encoderResolution[i];
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
	gSysRunning.m_gWorkStatus = SYS_WORKING_OP_MODE;
	printf("  SYS_WORKING_OP_MODE\n");
}
void Robot::power_off()
{
    gSysRunning.m_gWorkStatus = SYS_WORKING_LINK_DOWN;
	printf("  SYS_WORKING_LINK_DOWN\n");
}
void Robot::test_demo(void) 
{
	gSysRunning.m_gWorkStatus = SYS_WORKING_TEST_MODE;
}
double Robot::get_actual_position(int axis)
{
	if(axis >= axis_sum)
		return -1;
	//EC_T_SDWORD actual_pos = *(slave_vector[slave_num[axis]].actual_position);
	int32_t actual_pos = EC_READ_S32(slave_vector[axis].actual_position);
	double angle = 0;
	incToAngle(actual_pos, angle, axis);
	return angle;
}
int32_t Robot::get_actual_position_int(int axis)
{
	if(axis >= axis_sum)
		return -1;
	//EC_T_SDWORD actual_pos = *(slave_vector[slave_num[axis]].actual_position);
	int32_t actual_pos = EC_READ_S32(slave_vector[axis].actual_position);
	//double angle = 0;
	//incToAngle(actual_pos, angle, axis);
	return actual_pos;
}
void Robot::set_target_position(int axis, double targetPosition)
{
	if(axis >= axis_sum)
		return;
	int32_t target_pos_inc;
	angleToInc(targetPosition, target_pos_inc, axis);
	EC_WRITE_S32(slave_vector[axis].target_position, target_pos_inc);
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
	int16_t actual_torq = EC_READ_S16(slave_vector[slave_num[axis]].actual_torque);
	return actual_torq;
}
uint16_t Robot::get_status_word(int axis)
{
	if(axis >= axis_sum)
		return -1;
	uint16_t status_word = EC_READ_U16(slave_vector[axis].status_word);
	return status_word;
}
int Robot::get_actual_velocity(int axis)
{
	if(axis >= axis_sum)
		return -1;
	int32_t actual_vel = EC_READ_S32(slave_vector[slave_num[axis]].actual_velocity);

	return actual_vel;
}
void Robot::set_target_torque(int axis,int targetTorque)
{
	if(axis >= axis_sum)
		return;
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

/**		2024-5-3		
 *		Author: Szigit
 *		@brief 机器人运行循环函数
 * 		通信周期1ms，每一次通信周期运行一次
 */
void Robot::cycle_run()
{
	switch (gSysRunning.m_gWorkStatus) {
	case SYS_WORKING_SAFE_MODE:
		//检查主站是否处于 OP 模式, 若不是，则调整为 OP 模式
        check_master_state();
        check_slave_config_states();
        if((master_state.al_states & ETHERCAT_STATUS_OP))
        {
            int tmp = true;

			for(int i = 0;i < axis_sum;i++) {
				if(sc_state[i].al_state != ETHERCAT_STATUS_OP) {
					tmp = false;
					break ;
				}
			}

            if(tmp)
            {
                gSysRunning.op_count = 0;
                gSysRunning.m_gWorkStatus = SYS_WORKING_OP_MODE;
                printf("  SYS_WORKING_OP_MODE\n");
            }
        }
		break;

	case SYS_WORKING_OP_MODE:
		gSysRunning.op_count++;
		if(gSysRunning.op_count <= 900 && !poweronstatus) {
			switch (gSysRunning.op_count) {
			case 1:
				for(int i = 0;i < axis_sum;i++) {
                	EC_WRITE_U8(slave_vector[i].mode_of_operation, 8);
				}
                break;
            case 200:
			    for(int i = 0;i < axis_sum;i++) {
					EC_WRITE_U16(slave_vector[i].control_word, 0x80);    //错误复位  
					printf("set control %d: 0x80 \n", i);
				}
                break;
            case 300:
				for(int i = 0;i < axis_sum;i++) {
					int32_t curpos = EC_READ_S32(slave_vector[i].actual_position);  
					EC_WRITE_S32(slave_vector[i].target_position, curpos);
				}
                break;
            case 400:
				for(int i = 0;i < axis_sum;i++)
				{
					EC_WRITE_U16(slave_vector[i].control_word, 0x06 );
					printf("set control %d: 0x06 \n", i);
				}
                break;
            case 500:
				for(int i=0; i < axis_sum;i++)
				{
					EC_WRITE_U16(slave_vector[i].control_word, 0x07 );
					printf("set control %d: 0x07 \n", i);
				}
                break;
            case 600:
				for(int i=0;i < axis_sum;i++)
				{
					EC_WRITE_U16(slave_vector[i].control_word, 0x0f );
					printf("set control %d: 0x0f \n", i);
				}
                break; 
			}
		}
		else {
			printf("power on!");
			poweronstatus = true;
			gSysRunning.op_count = 0;
			gSysRunning.m_gWorkStatus = SYS_WORKING_IDLE_STATUS;
		}
		break;
	
	case SYS_WORKING_LINK_DOWN:
		gSysRunning.op_count++;
		int32_t actual_pos[Number];
		if(gSysRunning.op_count <= 900 && !poweronstatus) {
			switch (gSysRunning.op_count) {
			case 1:
				for(unsigned int i = 0; i < axis_sum; i++)
				{
					actual_pos[i] = EC_READ_S32(slave_vector[i].actual_position);
				}
                break;
            case 200:
				for(int i = 0;i < axis_sum;i++)
				{
					EC_WRITE_U16(slave_vector[i].control_word, 0x07 );
					EC_WRITE_S32(slave_vector[i].target_position, actual_pos[i] ); 
				}
                break;
            case 400:
				for(int i=0;i<axis_sum;i++)
				{
					EC_WRITE_U16(slave_vector[i].control_word, 0x06 );
					EC_WRITE_S32(slave_vector[i].target_position, actual_pos[i] );
				}
                break;
			}
		}
		else {
			printf("power off!");
			poweronstatus = false;
			gSysRunning.op_count = 0;
		}
		break;

	case SYS_WORKING_TEST_MODE:
		gSysRunning.demo_count++;
		if(gSysRunning.demo_count <= 500 && poweronstatus) {
			int32_t curpos = EC_READ_S32(slave_vector[1].actual_position);
			EC_WRITE_S32(slave_vector[1].target_position, curpos + 10 );
		}
		else {
			printf("finish test turning!");
			gSysRunning.m_gWorkStatus = SYS_WORKING_IDLE_STATUS;
			gSysRunning.demo_count = 0;
		}
		break;
	
	default:
		if(!angle_deque.empty() && poweronstatus) {
			double temp_pos[axis_sum];
			for(int i=0;i<axis_sum;i++) {
				temp_pos[i] = angle_deque.front();
				//2024.4.11增加各轴限位
				if(i==1 && (temp_pos[1]<-70 || temp_pos[1]> 70)) {
					return;
				}
				angle_deque.pop_front();
				set_target_position(i, temp_pos[i]);
			}
		}
		break;
	}


}
