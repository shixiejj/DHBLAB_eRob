/*
 * robotApi.h
 *
 *  Created on: Nov 24, 2020
 *      Author: liuchongyang
 */
#ifndef ROBOT_H_
#define ROBOT_H_

#include<deque>
#include<memory>
#include<stdio.h>
#include<unistd.h>

#include "../eigen/Eigen/Eigen"
#include "struct_define.h"
#include "trajectory.h"

using namespace Eigen;

#define rad2deg(r) ((r)*180.0/M_PI)
#define deg2rad(d) ((d)*M_PI/180.0)

class Trajectory;
class Robot
{
public:
	Robot();
	virtual ~Robot(){};
	friend class Trajectory;
	unsigned int axis_sum; //机器人总轴数
	std::vector<unsigned int> slave_num; //机器人各轴对应的从站序号, 从站序号从0开始
	/**
	* @brief 设置DH参数
	*/
	void set_DH_param(DH_param param_set);
	/**
	* @brief 获取DH参数
	*/
	DH_param get_DH_param();
	/**
	* @brief 设置笛卡尔参数
	*/
	void set_decare_param(Decare_Para param_set);
	/**
	* @brief 获取笛卡尔参数
	*/
	Decare_Para get_decare_param();
	/**
	* @brief 设置电机参数
	*/
	void set_motor_param(Motor_Param param_set);
	/**
	* @brief 获取电机参数
	*/
	Motor_Param get_motor_param();
	/**
	* @brief 设置运动队列
	*/
	void set_angle_deque(std::deque<double>& deque);
	/**
	* @brief 获取运动队列
	*/
	std::deque<double>& get_angle_deque();

	unsigned int get_angle_deque_size();


	void incToAngle(signed int inc,double& angle,int i);
	void angleToInc(double angle,signed int& inc,int i);

	/**
	* @brief 使能
	*/
	void power_on();
	/**
	* @brief 使能关闭
	*/
	void power_off();
	/**
	* @brief 获取实际位置角度值
	*/
	double get_actual_position(int axis);

	int32_t get_actual_position_int(int axis);

	void test_demo(void);

	/**
	* @brief 设置目标位置
	*/
	void set_target_position(int axis,double targetPosition);

	/**
	* @brief 获取机器人上电状态
	*/
	bool get_power_on_status();

	/**
	* @brief 获取实际转矩
	*/
	int16_t get_actual_torque(int axis);

	/**
	* @brief 获取状态字
	*/
	uint16_t get_status_word(int axis);

	/**
	* @brief 获取实际速度,速度单位取决于伺服输出的单位
	*/
	int get_actual_velocity(int axis);
	/**
	* @brief 设置目标转矩
	*/
	void set_target_torque(int axis,int targetTorque);

	/**
	* @brief 机器人关节运动插补
	*/
	void move_joint_interp(const VectorXd &targetPoint,
			const VectorXd &originPoint, const VectorXd &velCurrent, const VectorXd &accCurrent, double Ts, double velPerc,
			double accPerc, double decPerc, double jerkPerc,std::deque<double> &nAglSeqPtr);
	/**
	* @brief 机器人直线运动正解
	*/
	void move_line_interp(const VectorXd &targetPoint,
			const VectorXd &originPoint, const VectorXd &originACS, double velCurrent, double accCurrent,
			double Ts, double maxVelL, double maxAccL, double maxDecelL,
			double maxJerk, std::deque<double> &nAglSeqPtr);

	void cycle_run();

	/**
	* @brief 机器人正解,传入的关节值单位是角度
	*/
	virtual void calc_forward_kin(const VectorXd& posACS,MatrixXd& transMatrix) = 0;
	/**
	* @brief 机器人逆解,传出的关节值单位是角度
	*/
	virtual void calc_inverse_kin(const MatrixXd& transMatrix,const VectorXd& posLast, VectorXd& posACS) = 0;


protected:
	DH_param dh; ///DH参数对象
	Decare_Para decare;  ///笛卡尔参数对象
	Motor_Param motor_param;  ///电机参数对象
	Encoder_Param encoder_param;
	std::deque<double> angle_deque;  ///机器人运动队列
	bool poweronstatus;
	Trajectory* tarjectory_prt;
	GSysRunningParm gSysRunning;
};


#endif /* ROBOT_H_ */
