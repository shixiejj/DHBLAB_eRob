/*
 * calculate_method.h
 *
 *  Created on: Feb 22, 2023
 *      Author: liuchongyang
 */

#ifndef OPENRC_CALCULATE_METHOD_H_
#define OPENRC_CALCULATE_METHOD_H_


#include<deque>

#include "../eigen/Eigen/Eigen"
#include "struct_define.h"

using namespace Eigen;


/**
 * @brief 转移矩阵转直角坐标
 * @param m 转移矩阵
 * @return 返回六维直角坐标值
 */
VectorXd tr_2_MCS(MatrixXd m);

double calcRealAngle(double curAng, double candidateAng1, double candidateAng2);
/**
 * @brief 位姿转转移矩阵
 */
MatrixXd rpy_2_r(Vector3d rpy);
/**
 * @brief 直角坐标转转移矩阵
 */
MatrixXd rpy_2_tr(VectorXd posMCS);
/**
 * @brief 转移矩阵转位姿
 */
Vector3d tr_2_rpy(MatrixXd m);

/**
 * @brief 计算五次多项式插补系数
 * @param q0 初始位置
 * @param qf 目标位置
 * @param dq0 初始速度
 * @param dqf 目标速度
 * @param ddq0 初始加速度
 * @param ddqf 目标加速度
 * @param tf 插补周期
 * @param Coe 五次多项式结构体参数
 */
void get_Quintic_Coe(double q0, double qf, double dq0, double dqf,
		double ddq0, double ddqf, double tf, Quintic_Coe &Coe);
/**
 * @brief 取符号
 * @param d 需要取符号的参数
 * @return d 大于0时返回1，d小于0时返回-1
 */
int sgn_Positive(double d);

/**
 * @brief 五次多项式求解
 * @param t 运动队列中每个点的时间点
 * @param coe get_Quintic_Coe()计算得到的五次多项式系数
 * @param qptr 插补得到的运动队列指针
 */
void Quintic_Polynomi(double r, Quintic_Coe& coe, double *qptr);
/**
 * @brief 计算直线插补系数
 * @param q0 初始位置
 * @param q1 目标位置
 * @param Ts 插补周期
 * @param maxVel 最大速度
 * @param maxAcc 最大加速度
 * @param maxDecel 最大减速度
 * @param maxJerk 最大雅可比速度
 * @param seq 插补得到的运动队列
 * @param beginVel 初始速度
 * @param beginAcc 初始加速度
 */
void calc_Interp_5_1_5(double q0, double q1, double Ts,
		double maxVel, double maxAcc, double maxDecel, double maxJerk,std::deque<double> &seq, double beginVel, double beginAcc);



#endif /* OPENRC_CALCULATE_METHOD_H_ */
