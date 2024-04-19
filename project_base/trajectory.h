/*
 * trajectory.h
 *
 *  Created on: Feb 22, 2023
 *      Author: liuchongyang
 */

#ifndef OPENRC_TRAJECTORY_H_
#define OPENRC_TRAJECTORY_H_

#include<deque>

#include "../eigen/Eigen/Eigen"
#include "struct_define.h"
#include "robot.h"

using namespace Eigen;

class Robot;
class Trajectory
{
public:
	Trajectory(Robot *prt) : robot_ptr(prt){};
	virtual ~Trajectory(){};

	/**
	* @brief 机器人关节运动插补
	*/
	virtual void move_joint_interp(const VectorXd &targetPoint,
			const VectorXd &originPoint, const VectorXd &velCurrent, const VectorXd &accCurrent, double Ts, double velPerc,
			double accPerc, double decPerc, double jerkPerc,std::deque<double> &nAglSeqPtr);
	/**
	* @brief 机器人直线运动正解
	*/
	virtual void move_line_interp(const VectorXd &targetPoint,
			const VectorXd &originPoint, const VectorXd &originACS, double velCurrent, double accCurrent,
			double Ts, double maxVelL, double maxAccL, double maxDecelL,
			double maxJerk, std::deque<double> &nAglSeqPtr);
private:
	Robot *robot_ptr;
};



#endif /* OPENRC_TRAJECTORY_H_ */
