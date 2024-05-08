/*
 * trajectory.cpp
 *
 *  Created on: Feb 22, 2023
 *      Author: liuchongyang
 */

#include "trajectory.h"
#include "calculate_method.h"
#include "../ec_config.h"

void Trajectory::move_joint_interp(const VectorXd &targetPoint,
		const VectorXd &originPoint, const VectorXd &velCurrent, const VectorXd &accCurrent, double Ts, double velPerc,
		double accPerc, double decPerc, double jerkPerc , std::deque<double> &deque) //输入的是角度值
{
	int n = Number;
	VectorXd posacs(n);
	int mcsDimension = Number;
	VectorXd posmcs(mcsDimension);	//正解求直角坐标位置

	//	若有位置命令未执行完毕，等待各轴进入空闲状态
	//	MC_WaitIsFree();

	VectorXd posOrigin(originPoint);
	VectorXd posTarget(targetPoint);

	//计算各轴角位移偏移量
	VectorXd posOffset(n);
	posOffset = posTarget - posOrigin;

	VectorXd t(n); //若全程匀速，各轴时间
	double T=0; //关节运动最长的那个时间
	VectorXd tA(n); //各轴加速时间
	double tAcc=0; //关节加速过程最长的那个时间
	VectorXd tD(n); //各轴加速时间
    double tDec=0; //关节加速过程最长的那个时间

	_AdjustVel:

    velPerc=velPerc/100;
    accPerc=accPerc/100;
    decPerc=decPerc/100;

    //第一步：确定加（减）速段时间tAcc               v=a*t
    for (int ai=0; ai!=n; ++ai) /*1-n轴*/
    {
        tA[ai] = fabs((velPerc * 1.0 * (robot_ptr->motor_param.RatedVel[ai]) * sgn_Positive(posOffset[ai]) - velCurrent[ai])
                / (accPerc * (robot_ptr->motor_param.maxAcc[ai]) * (robot_ptr->motor_param.RatedVel[ai])));
        tD[ai] = (velPerc * 1.0)
                        / (decPerc * fabs(robot_ptr->motor_param.maxDecel[ai]));

        if (tA[ai] > (velPerc * 1.0) / (accPerc * fabs(robot_ptr->motor_param.maxAcc[ai])))
        {
            tA[ai] = (velPerc * 1.0) / (accPerc * fabs(robot_ptr->motor_param.maxAcc[ai]));
        }
    }

    tAcc=tA[0];
    tDec=tD[0];
    for (int ai=1; ai!=n; ++ai) //2-n轴
    {
        if ((tAcc > tA[ai] || tAcc < 2 * Ts) && (fabs(posOffset[ai]) > 0.1) && tA[ai] > Ts)
        {
            tAcc=tA[ai];
        }
        if (tDec > tD[ai] && (fabs(posOffset[ai]) > 0.1))
        {
            tDec=tD[ai];
        }
    }

    for (int ai=0; ai!=n; ++ai) //1-n轴
    {
        if ((tAcc < tA[ai]) && (fabs(posOffset[ai]) > 0.1))
        {
            tAcc=tA[ai];
        }

        if ((tDec < tD[ai]) && (fabs(posOffset[ai]) > 0.1))
        {
            tDec=tD[ai];
        }
    }
    if (!tAcc || !tDec)
    {
        return;
    }

	//第二步：若全程视作匀速运动，确定各轴最大运动时间T

	for (int ai=0; ai<n; ++ai) /*1-n轴*/
	{
	    if (velPerc > 0.998 && (robot_ptr->motor_param.maxRotSpeed[ai] < 1.05))
	    {
	        t[ai]=fabs(posOffset[ai] - tA[ai] / 2 * velCurrent[ai])/(velPerc*(robot_ptr->motor_param.RatedVel[ai]) * 0.998);
	    }
	    else
	    {
	        t[ai]=fabs(posOffset[ai] - tA[ai] / 2 * velCurrent[ai])/(velPerc*(robot_ptr->motor_param.RatedVel[ai]));
	    }
	}
	//比较时间大小，取最大值赋值给T
	T=t[0];
	for (int ai=1; ai!=n; ++ai) //2-n轴
	{
		if (T<t[ai])
		{
			T=t[ai];
		}
	}
	if (!T)
	{
		return;
	}

	//第三步：确定匀速段时间tConst
	double tf=T+tAcc / 2 + tDec / 2; //最终整个过程运行时间为tf              //2023.8.8:  tAcc是0-v匀加速的加速时间，但实际加速度变过程应该是0-a-0，所以加速段时间应该是tAcc/2;减速段同理
	double tConst=tf - tAcc - tDec;
	if (tConst<0)
	{
		//todo: 不采用速度减半方法，而是改为只有变速段，没有匀速段处理
		//速度减半
		velPerc = velPerc*98;
		accPerc = accPerc*98;
		decPerc = decPerc * 98;
		goto _AdjustVel;
		//return PLCOPEN_MOVE_INSTRUCT_NOT_APPLICABLE;
	}

	//第四步：确定加速段、匀速段、减速段的插补步数（N1,N2,N3），并确定最终的运行时间
	size_t N1 = static_cast<size_t>(tAcc/Ts);
	size_t N2 = static_cast<size_t>(tConst/Ts);
	size_t N3 = static_cast<size_t>(tDec/Ts);
	if (N1 == 0)
    {
        N1 = 1;
    }
	tAcc = N1 * Ts;
	tConst = N2 * Ts;
	tDec = N3 * Ts;
	T = tConst + tAcc / 2 + tDec / 2;

	//第五步：计算各轴最终运行时的平均速度(也即匀速段的速度velConst，注意这个速度与设定的速度可能不一样)
	VectorXd velConst(n);//有正负号
	for (int ai=0; ai!=n; ++ai) /*1-n轴*/
	{
		velConst[ai] = (posOffset[ai] - tAcc / 2 * velCurrent[ai])/T;
	}

	//第六步：轨迹插补，其中：用五次多项式对加速段和减速段进行插补
	VectorXd posInterp(n);//用于暂存各轴某周期的插补点位置
	VectorXd pLast(n);

	pLast = deg2rad(originPoint);

		//(1)加速段插补 [ 第 0 ~ N1 点]
		//计算五次多项式系数
	VectorXd accPosEnd(n);//各轴加速段的结束点位置
		std::deque<Quintic_Coe> coeQuintic(n);//各个轴的五次多项式

		for (int ai=0; ai!=n; ++ai) /*1-n轴*/
		{
			accPosEnd[ai] = posOrigin[ai]+(velConst[ai] + velCurrent[ai])*tAcc/2;
			get_Quintic_Coe(posOrigin[ai], accPosEnd[ai], velCurrent[ai], velConst[ai], accCurrent[ai], 0,
					tAcc, coeQuintic[ai]);
		}
		//生成链表
		for (size_t i=0; i!=(N1+1); ++i) //i用于记录加速段插补点的序号
		{
			double t = i*Ts; //时间t
			for (int ai=0; ai!=n; ++ai) /*1-n轴*/
			{
				Quintic_Polynomi(t, coeQuintic[ai], &(posInterp[ai]));
				posacs[ai]=deg2rad(posInterp[ai]);
			}

			for (int ai=0; ai!=n; ++ai) /*1-n轴*/
			{
				deque.push_back(posInterp[ai]);
			}
			pLast = deg2rad(posInterp);
		}

		//(2)匀速段插补 [ 第 (N1+1) ~ (N1+N2) 点]
		//生成链表
		for (size_t j=1; j!=(N2+1); ++j) //j用于记录匀速段插补点的序号
		{
			for (int ai=0; ai!=n; ++ai) /*1-n轴*/
			{
				posInterp[ai] += velConst[ai]*Ts;
				posacs[ai]=deg2rad(posInterp[ai]);
			}

			for (int ai=0; ai!=n; ++ai) /*1-n轴*/
			{
				deque.push_back(posInterp[ai]);
			}
			pLast = deg2rad(posInterp);
		}

		//(3)减速段插补 [ 第 (N1+N2+1) ~ (N1+N2+N3) 点]
		//计算五次多项式系数
		for (int ai=0; ai!=n; ++ai) /*1-n轴*/
		{
			get_Quintic_Coe(posInterp[ai], posTarget[ai], velConst[ai], 0, 0, 0,
			        tDec, coeQuintic[ai]);
		}
		//生成链表
		for (size_t k=1; k!=(N3+1); ++k) //k用于记录减速段插补点的序号
		{
			double t = k*Ts; //时间t
			for (int ai=0; ai!=n; ++ai) /*1-n轴*/
			{
				Quintic_Polynomi(t, coeQuintic[ai], &(posInterp[ai]));
				posacs[ai]=deg2rad(posInterp[ai]);
			}

			for (int ai=0; ai!=n; ++ai) /*1-n轴*/
			{
				deque.push_back(posInterp[ai]);
			}
			pLast = deg2rad(posInterp);
		}
	return;
}


void Trajectory::move_line_interp(const VectorXd &targetPoint,
		const VectorXd &originPoint, const VectorXd &originACS, double velCurrent, double accCurrent,
		double Ts, double maxVelper, double maxAccper, double maxDecelper,
		double maxJerk, std::deque<double> &deque)
{
	int n = 6;
	int mcsDimension = 6;

	int axisNum;
	VectorXd targetACS(n);

	bool moveDirection = true;

	//	若有位置命令未执行完毕，等待各轴进入空闲状态
	//	MC_WaitIsFree();

	//计算路径长度
	double displacement = sqrt(pow((targetPoint[0]-originPoint[0]), 2)+pow(
			(targetPoint[1]-originPoint[1]), 2)+pow((targetPoint[2]
			-originPoint[2]), 2));
	std::deque<double> rlst;
    std::deque<double> rlstMcs;
    std::deque<double> rlstAcs;
    double maxVelAcs,maxAccAcs,maxDecelAcs,displacementAcs;

	if (displacement > 0.2)
	{
	        if (sqrt(maxAccper * displacement) < (0.98 * maxVelper))
	        {
	            for (int i=0;i<1000;i++)
	            {
	                maxVelper = 0.98 * maxVelper;
	                maxAccper = 0.98 * maxAccper;
	                maxDecelper = 0.98 * maxDecelper;
	                if (sqrt(maxAccper * displacement) >= maxVelper)
	                {
	                    break;
	                }
	            }
	        }
	    calc_Interp_5_1_5(0, 1, Ts, maxVelper/displacement, maxAccper/displacement,
	            maxDecelper/displacement, maxJerk/displacement, rlstMcs, velCurrent / displacement, accCurrent / displacement);
	    robot_ptr->calc_inverse_kin(rpy_2_tr(targetPoint),originACS,targetACS);
        displacementAcs = fabs(targetACS[0] - originACS[0]);
        axisNum = 0;
        for (int ai=1; ai!=n; ++ai) //2-n轴
        {
            if (displacementAcs < fabs(targetACS[ai] - originACS[ai]))
            {
                displacementAcs = fabs(targetACS[ai] - originACS[ai]);
                axisNum = ai;
            }
        }

        if (displacementAcs < 0.0001 )
        {
            return;
        }
        else
        {
            maxVelAcs = robot_ptr->motor_param.RatedVel[axisNum];
            maxAccAcs = robot_ptr->motor_param.maxAcc[axisNum] * robot_ptr->motor_param.RatedVel[axisNum];
            maxDecelAcs = robot_ptr->motor_param.maxDecel[axisNum] * robot_ptr->motor_param.DeRatedVel[axisNum];

            if (sqrt(maxAccAcs * displacementAcs) < maxVelAcs)
            {
                maxVelAcs = sqrt(maxAccAcs * displacementAcs);
            }
        }
        calc_Interp_5_1_5(0, 1, Ts, maxVelAcs/displacementAcs, maxAccAcs/displacementAcs,
                maxDecelAcs/displacementAcs, maxJerk/displacementAcs, rlstAcs, velCurrent / displacement, accCurrent / displacement );
        rlst = rlstMcs;
	}
	else
	{
		robot_ptr->calc_inverse_kin(rpy_2_tr(targetPoint),originACS,targetACS);
		displacement = fabs(targetACS[0] - originACS[0]);
		axisNum = 0;
		for (int ai=1; ai!=n; ++ai) //2-n轴
		{
			if (displacement < fabs(targetACS[ai] - originACS[ai]))
			{
				displacement = fabs(targetACS[ai] - originACS[ai]);
				axisNum = ai;
			}
		}
		if (displacement < 0.0001)
		{
			return;
		}
		else
		{
			maxVelper = maxVelper/robot_ptr->decare.maxvel*(robot_ptr->motor_param.RatedVel[axisNum]);
			maxAccper = maxAccper/(robot_ptr->decare.maxacc * robot_ptr->decare.maxvel) * (robot_ptr->motor_param.maxAcc[axisNum] * robot_ptr->motor_param.RatedVel[axisNum]);
			maxDecelper = maxDecelper/(robot_ptr->decare.maxdec * robot_ptr->decare.maxvel)*(robot_ptr->motor_param.maxDecel[axisNum] * robot_ptr->motor_param.DeRatedVel[axisNum]);
			if (sqrt(maxAccper * displacement) < maxVelper)
			{
				maxVelper = sqrt(maxAccper * displacement);
			}
			if (maxAccper / maxVelper > 2)
			{
				maxAccper = 2 * maxVelper;
				maxDecelper = 2 * maxVelper;
			}
		}
	    calc_Interp_5_1_5(0, 1, Ts, maxVelper/displacement, maxAccper/displacement,
	            maxDecelper/displacement, maxJerk/displacement, rlst, velCurrent / displacement, accCurrent / displacement);
	}

	//球面线性插补
	Matrix3d mOrigin = rpy_2_r(originPoint.segment(3,3));
	Matrix3d mTarget = rpy_2_r(targetPoint.segment(3,3));

	Vector3d pOrigin = originPoint.head(3);
	Vector3d pTarget = targetPoint.head(3);
	Vector3d pr;
	MatrixXd Tr(4, 4);
	VectorXd posr(mcsDimension);
	VectorXd posACS(n);
	VectorXd pLast(n);
	Quaternion<double> qr;
	Quaternion<double> qTarget(mTarget);
	Quaternion<double> qOrigin(mOrigin);

	for(int i = 0; i < 11; i++)
	{
	    double r= i / 10.0;
	    qr = qOrigin.slerp(r, qTarget);
	    pr = pOrigin*(1-r) + r*pTarget;
	    Tr << qr.toRotationMatrix(), pr, MatrixXd::Zero(1, 3), 1;
	    posr.head(6) <<  tr_2_MCS(Tr);
        if (i == 0)
        {
            pLast=originACS;
        }
        robot_ptr->calc_inverse_kin(rpy_2_tr(posr),pLast,posACS);
	    pLast = posACS;
	}
	for (std::deque<double>::iterator rit = rlst.begin(); rit != rlst.end(); ++rit)
	{
		double r=*rit;
		if (moveDirection)
		{
		    qr = qOrigin.slerp(r, qTarget);
		}
		else
		{
		    qr = qOrigin.slerpLongWay(r, qTarget);
		}
		pr = pOrigin*(1-r) + r*pTarget;
		Tr << qr.toRotationMatrix(), pr, MatrixXd::Zero(1, 3), 1;

        posr.head(6) <<  tr_2_MCS(Tr);

		if (rit==rlst.begin())
		{
			pLast=originACS;
		}
		robot_ptr->calc_inverse_kin(rpy_2_tr(posr),pLast,posACS);

	    for (int ai=0; ai!=n; ++ai) //1-n轴
	    {
	    	deque.push_back(posACS[ai]);
	    }
	    pLast = posACS;
	}
	return;
}



