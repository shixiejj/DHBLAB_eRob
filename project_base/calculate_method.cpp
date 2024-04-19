/*
 * calculate_method.cpp
 *
 *  Created on: Feb 22, 2023
 *      Author: liuchongyang
 */

#include "calculate_method.h"

VectorXd tr_2_MCS(MatrixXd m)  //转移矩阵转直角坐标
{
	VectorXd posMCS(6);
	posMCS << m.block(0,3,3,1),tr_2_rpy(m);
	return posMCS;
}

double calcRealAngle(double curAng, double candidateAng1,
		double candidateAng2)
{
	double curAngCopy(curAng);
	double* allAng[3] =
	{ &candidateAng1, &candidateAng2, &curAngCopy };
	for (int i = 0; i < 3; ++i)
	{
		while (*allAng[i]> M_PI)
		{
			*allAng[i] -= 2*M_PI;
		}
		while (*allAng[i] < -M_PI)
		{
			*allAng[i] += 2*M_PI;
		}
	}
	double gap[2] =
	{ 0, 0 };
	bool dir[2] =
	{ true, true };
	for (int i = 0; i < 2; ++i)
	{
		if (*allAng[i] >= curAngCopy)/*=*/
		{
			gap[i] = *allAng[i]-curAngCopy;
			dir[i] = true;
		}
		else
		{
			gap[i] = curAngCopy-*allAng[i];
			dir[i] = false;
		}
		if (gap[i]> M_PI)
		{
			gap[i] = 2*M_PI-gap[i];
			dir[i] = !dir[i];
		}
	}

	if (gap[0] <= gap[1])/*=*/
	{
		return dir[0] ? curAng+gap[0] : curAng-gap[0];
	}
	else
	{
		return dir[1] ? curAng+gap[1] : curAng-gap[1];
	}
}


MatrixXd rpy_2_tr(VectorXd posMCS)  //直角坐标转转移矩阵
{
	MatrixXd tr(4, 4);
	tr << rpy_2_r(posMCS.segment(3,3)), posMCS.head(3), MatrixXd::Zero(1, 3), 1;
	return tr;
}
Vector3d tr_2_rpy(MatrixXd m)  //转移矩阵转位姿
{
	Vector3d rpy;
	rpy(2) = atan2(-m(1, 2), m(2, 2));
	double sr = sin(rpy(0, 0));
	double cr = cos(rpy(0, 0));
	rpy(1) = atan2(m(0, 2), cr*m(2, 2)-sr*m(1, 2));
	rpy(0) = atan2(-m(0, 1), m(0, 0));
	return rpy;
}
// MatrixXd rpy_2_r(Vector3d rpy)  //位姿转旋转矩阵
// {
// 	return Eigen::AngleAxisd(rpy(0), Vector3d::UnitX()).toRotationMatrix() * Eigen::AngleAxisd(rpy(1), Vector3d::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(rpy(2), Vector3d::UnitZ()).toRotationMatrix();
// }

/*     2023.7.26修改RPY_ZYX欧拉角得到旋转矩阵的方式      */
MatrixXd rpy_2_r(Vector3d rpy)  //位姿转旋转矩阵
{
	return Eigen::AngleAxisd(rpy(2), Vector3d::UnitX()).toRotationMatrix() * Eigen::AngleAxisd(rpy(1), Vector3d::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(rpy(0), Vector3d::UnitZ()).toRotationMatrix();
}


void get_Quintic_Coe(double q0, double qf, double dq0, double dqf,
		double ddq0, double ddqf, double tf, Quintic_Coe &Coe)
{
	Coe.a0=q0;
	Coe.a1=dq0;
	Coe.a2=ddq0/2.0;
	Coe.a3=(20*qf-20*q0-(8*dqf+12*dq0)*tf-(3*ddq0-ddqf)*tf *tf)/(2*pow(tf, 3));
	Coe.a4=(30*q0-30*qf+(14*dqf+16*dq0)*tf+(3*ddq0-2*ddqf) *tf*tf)/(2 *pow(tf,
			4));
	Coe.a5=(12*qf-12*q0-(6*dq0+6*dqf)*tf-(ddq0-ddqf)*tf*tf) /(2*pow(tf, 5));
}

int sgn_Positive(double d)
{
    return d <-10e-5?-1:1;
}
//五次多项式求解
void Quintic_Polynomi(double r, Quintic_Coe& coe, double *qptr)
{
	if (qptr)
		*qptr = coe.a0+coe.a1*r+coe.a2*pow(r, 2)+coe.a3*pow(r, 3)+coe.a4*pow(r,
				4) +coe.a5*pow(r, 5);
}
void calc_Interp_5_1_5(double q0, double q1, double Ts,
		double maxVel, double maxAcc, double maxDecel, double maxJerk,
		std::deque<double> &seq, double beginVel, double beginAcc)
{
	double dis = q1-q0;
	if (!maxVel)
		return;

    //第一步：确定加（减）速段时间tAcc
    double tAcc = fabs((maxVel - beginVel)/maxAcc);
    double tDecel = fabs(maxVel/maxDecel);

    if(tAcc > maxVel / maxAcc)
    {
        tAcc = maxVel / maxAcc;
    }

	//第二步：若全程视作匀速运动，确定最大运动时间T
	double T = fabs((dis-tAcc/2*beginVel)/maxVel);
	if (!T)
	{
		return;
	}

	//第三步：确定匀速段时间tConst
	double tf = tAcc/2 + T + tDecel/2; //最终整个过程运行时间为tf
	double tConst = tf - tAcc - tDecel;

	//判断是否有匀速段
	if (tConst<0.75*Ts)
		tConst =0;
	{
		//第四步：确定加速段、匀速段、减速段的插补步数（N1,N2,N3）,并确定最终运动时间
		size_t N1 = static_cast<size_t>(tAcc/Ts);
		size_t N2 = static_cast<size_t>(tConst/Ts);
		size_t N3 = static_cast<size_t>(tDecel/Ts);
		if (N1 == 0)
		{
		    N1 = 1;
		}
		tAcc = N1 * Ts;
		tDecel = N3 * Ts;
		tConst = N2 * Ts;

		T = tConst + tAcc/2 + tDecel/2;

		//第五步：计算轴最终运行时的平均速度，就是有符号的maxVel
		double velConst = (dis-tAcc/2*beginVel) / T;//有正负号

		//第六步：轨迹插补，其中：用五次多项式对加速段和减速段进行插补
		seq.clear();
		double posInterp =0;//用于暂存轴某周期的插补点位置

			//(1)加速段插补 [ 第 0 ~ N1 点]
			//计算五次多项式系数
			double accPosEnd;//轴加速段的结束点位置
			Quintic_Coe coe;//轴的五次多项式
			accPosEnd = q0+(velConst+beginVel)*tAcc/2;
			get_Quintic_Coe(q0, accPosEnd, beginVel, velConst, beginAcc, 0, tAcc, coe);

			//生成链表
			for (size_t i=0; i!=(N1+1); ++i) //i用于记录加速段插补点的序号
			{
				Quintic_Polynomi(i*Ts, coe, &posInterp);
				seq.push_back(posInterp);
			}

			//(2)匀速段插补 [ 第 (N1+1) ~ (N1+N2) 点]
			//生成链表
			for (size_t j=1; j!=(N2+1); ++j) //j用于记录匀速段插补点的序号
			{
				posInterp += velConst*Ts;
				seq.push_back(posInterp);
			}

			//(3)减速段插补 [ 第 (N1+N2+1) ~ (N1+N2+N3) 点]
			//计算五次多项式系数
			get_Quintic_Coe(posInterp, q1, velConst, 0, 0, 0, tDecel, coe);

			//生成链表
			for (size_t k=1; k!=(N3+1); ++k) //k用于记录减速段插补点的序号
			{
				Quintic_Polynomi(k*Ts, coe, &posInterp);
				seq.push_back(posInterp);
			}
	}
	return;
}
