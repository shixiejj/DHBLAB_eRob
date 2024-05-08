/*
 * UR.cpp
 *
 *  Created on: Feb 22, 2023
 *      Author: liuchongyang
 */

#include "robot_type.h"
#include "ec_config.h"


#include <iostream>
using namespace std;


void UR::calc_forward_kin(const VectorXd& posACS,MatrixXd& transMatrix)
{
	int n = Number;

	VectorXd acs_rad(n);

	transMatrix = (MatrixXd::Identity( 4, 4));

	for(int i=0;i<n;i++)
	{
		acs_rad[i] = deg2rad(posACS[i]);
	    MatrixXd X(4, 4); /**< @brief Gets or sets the matrix regarding X axis transformations. */
	    MatrixXd Z(4, 4); /**< @brief Gets or sets the matrix regarding Z axis transformations. */
	    Z << Eigen::AngleAxisd(dh.theta[i]+acs_rad[i], Vector3d::UnitZ()).toRotationMatrix(), (MatrixXd(3,1)<<0.0,0.0,dh.d[i]).finished(),
	    		MatrixXd::Zero(1, 3), 1;
	    X << Eigen::AngleAxisd(dh.alpha[i], Vector3d::UnitX()).toRotationMatrix(), (MatrixXd(3,1)<<dh.a[i],0.0,0.0).finished(),
	    		MatrixXd::Zero(1, 3), 1;
	    transMatrix = transMatrix*Z*X;
	}
}


void UR::calc_inverse_kin(const MatrixXd& transMatrix, const VectorXd& posLast, VectorXd& posACS)
{
	/*       2023.7.25以能量和最小原则寻找最优解           */
    int n=Number;
	double d1{}, d2{}, d5{}, d6{}, a2{}, a3{};
    posACS=posLast;

	d1 = 143.5;
	d2 = 86.5;
	a2 = 243.5;


	MatrixXd T(transMatrix);    //T是末端位姿矩阵


	Eigen::MatrixXd Result(n, 2);
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			Result(i, j) = 100;
		}
	}
	
	/*		2021-11-26新增		*/
	VectorXd pLast(n); //读到的角度，经下面处理后，成为theta值
	for (int i = 0; i < n; i++)
		pLast(i) = deg2rad(posLast(i));
	/*		2021-11-26新增结束		*/

	/*		  2023.7.24新增	      	*/
	//计算时加上偏移量（弧度）
	pLast(1) = pLast(1) + 1.57;
	pLast(2) = pLast(2) + (-1.57);

	/*		2023.7.24新增结束		*/

	double nx(T(0, 0)), ny(T(1, 0)), nz(T(2, 0));
	double ox(T(0, 1)), oy(T(1, 1)), oz(T(2, 1));
	double ax(T(0, 2)), ay(T(1, 2)), az(T(2, 2));
	double px(T(0, 3)), py(T(1, 3)), pz(T(2, 3));

	//solve for theta1
	double theta11, theta12;
	double A = ax;
	double B = -ay;
	double C = 1;
	double D = pow(A, 2) + pow(B, 2) - pow(C, 2);
	if(fabs(D)<0.00001) D=0;
	
	if (D >= 0.0L)
	{
		if ((fabs(A) < 10e-13) && (fabs(B) < 10e-13))
		{
			theta11 = pLast(0);
			theta12 = theta11;
		}
		else
		{
			theta11 = atan2(C, sqrt(D)) - atan2(B, A);
			theta12 = atan2(C, -sqrt(D)) - atan2(B, A);
			if (theta11 > 3.14)
			{
				theta11 = theta11 - 2 * M_PI;
			}
			else if (theta11 < -3.14)
			{
				theta11 = theta11 + 2 * M_PI;
			}

			if (theta12 > 3.14)
			{
				theta12 = theta12 - 2 * M_PI;
			}
			else if (theta12 < -3.14)
			{
				theta12 = theta12 + 2 * M_PI;
			}
		}

		Result(0, 0) = theta11;
		Result(0, 1) = theta12;

	}

	//solve for theta2
    double sin2_1=(pz-d1)/a2;
    double cos2_1=(px*cos(theta11)+py*sin(theta11))/a2;
    double theta21=atan2(sin2_1,cos2_1);

    double sin2_2=(pz-d1)/a2;
    double cos2_2=(px*cos(theta12)+py*sin(theta12))/a2;
    double theta22=atan2(sin2_2,cos2_2);

    Result(1, 0) = theta21;
    Result(1, 1) = theta22;

    //solve for theta3
    double sin23=nz;
    double cos23=oz;
    double theta31=atan2(sin23,cos23)-theta21;
    double theta32=atan2(sin23,cos23)-theta22; 

    if (theta31 > 3.14)
    {
        theta31 = theta31 - 2 * M_PI;
    }
    else if (theta31 < -3.14)
    {
        theta31 = theta31 + 2 * M_PI;
    }

    if (theta32 > 3.14)
    {
        theta32 = theta32 - 2 * M_PI;
    }
    else if (theta32 < -3.14)
    {
        theta32 = theta32 + 2 * M_PI;
    }

    Result(2, 0) = theta31;
    Result(2, 1) = theta32;

	//求2组解的角度变化和，寻找最优解
	//元素取正
	Eigen::MatrixXd Result_abs(n, 2);
	Eigen::VectorXd pos_0(2);  
	//pos_0 << 0, M_PI / 2, -M_PI / 2, M_PI / 2, M_PI / 2, 0;      //机械臂初始偏置
	pos_0 = pLast;
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			Result_abs(i, j) = pow((Result(i, j) - pos_0(i)),2);
		}
	}
	//解矩阵按列求和
	Eigen::VectorXd histogram1 = Result_abs.colwise().sum();
	//求角度变化最小的列角度变化之和min_h与索引minIndex
	Eigen::MatrixXd::Index minIndex;
	double min_h = histogram1.minCoeff(&minIndex);

    VectorXd tempPos=pLast;
	//输出能量和最小的一组最优解
	for (int i = 0; i < n; i++)
	{
		tempPos[i] = rad2deg(Result(i, minIndex));   
		if (fabs(tempPos[i]) > 180)
		{
            cout<<"  Error  "<<endl;
			return;
		}
	}

    posACS=tempPos;
	//输出时减去偏移量（角度）
	posACS(1) = posACS(1) - 90;
	posACS(2) = posACS(2) - (-90);
}

void UR::move_line_interp(const VectorXd &targetPoint,
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
	        if (sqrt(maxAccper * displacement) < (0.98 * maxVelper))     //maxAccper加速度百分比；maxVelper速度百分比
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
	    calc_inverse_kin(rpy_2_tr(targetPoint),originACS,targetACS);

		//2023.7.27测试目标
		//std::cout<<"targetACS："<<targetACS<<std::endl;


        displacementAcs = fabs(targetACS[0] - originACS[0]);
        axisNum = 0;
        for (int ai=1; ai!=n; ++ai) //2-n轴
        {
            if (displacementAcs < fabs(targetACS[ai] - originACS[ai]))
            {
                displacementAcs = fabs(targetACS[ai] - originACS[ai]);
                axisNum = ai;
            }
        }   //搜寻最小的displacementAcs

        if (displacementAcs < 0.0001 )
        {
            return;
        }
        else
        {
            maxVelAcs = motor_param.RatedVel[axisNum];
            maxAccAcs = motor_param.maxAcc[axisNum] * motor_param.RatedVel[axisNum];
            maxDecelAcs = motor_param.maxDecel[axisNum] * motor_param.DeRatedVel[axisNum];

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
		calc_inverse_kin(rpy_2_tr(targetPoint),originACS,targetACS);
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
			std::cout << "else displacementAcs < 0.0001" << std::endl;
			return;
		}
		else
		{
			maxVelper = maxVelper/decare.maxvel*(motor_param.RatedVel[axisNum]);
			maxAccper = maxAccper/(decare.maxacc * decare.maxvel) * (motor_param.maxAcc[axisNum] * motor_param.RatedVel[axisNum]);
			maxDecelper = maxDecelper/(decare.maxdec * decare.maxvel)*(motor_param.maxDecel[axisNum] * motor_param.DeRatedVel[axisNum]);
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
	Matrix3d mOrigin = rpy_2_r(originPoint.segment(3,3));        //得到初始位置的姿态旋转矩阵
	Matrix3d mTarget = rpy_2_r(targetPoint.segment(3,3));        //得到目标位置的姿态旋转矩阵

	Vector3d pOrigin = originPoint.head(3);                      //提取前3个元素，得到xyz坐标
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
	    qr = qOrigin.slerp(r, qTarget);              //slerp四元数平滑插值，r是插值参数：0-1
	    pr = pOrigin*(1-r) + r*pTarget;
	    Tr << qr.toRotationMatrix(), pr, MatrixXd::Zero(1, 3), 1;
	    posr.head(6) <<  tr_2_MCS(Tr);
        if (i == 0)
        {
            pLast=originACS;
        }
        calc_inverse_kin(rpy_2_tr(posr),pLast,posACS);
	    pLast = posACS;
	}
	for (std::deque<double>::iterator rit = rlst.begin(); rit != rlst.end(); ++rit)
	{
		double r=*rit;
		if (moveDirection)
		{
		    qr = qOrigin.slerp(r, qTarget);                         //旋转四元数的球面插值Rw1.slerp(t,Rw2),t在(0.0 - 1.0)范围,当成标量的话相当于Rw1 + t*(Rw2-Rw1)
		}
		else
		{
		    qr = qOrigin.slerpLongWay(r, qTarget);
		}
		pr = pOrigin*(1-r) + r*pTarget;                              //得到位置向量
		
		//末端齐次变换矩阵
		Tr << qr.toRotationMatrix(), pr, MatrixXd::Zero(1, 3), 1;    //.toRotationMatrix():旋转向量转旋转矩阵；

        posr.head(6) <<  tr_2_MCS(Tr);

		if (rit==rlst.begin())
		{
			pLast=originACS;
		}
		calc_inverse_kin(rpy_2_tr(posr),pLast,posACS);

	    for (int ai=0; ai!=n; ++ai) //1-n轴
	    {
	    	deque.push_back(posACS[ai]);     
	    }
		pLast = posACS;

	    
	}
	return;
}



/*    2024.1.22 转移矩阵转xyzRPY  (RPY->zyx  绕x,y,z动轴旋转)      */
double calcAtan2(double sr, double cr)
{
	if (fabs(sr) > 1e-7 && fabs(cr) > 1e-7)
	{
		return atan2(sr, cr);
	}
	else if (fabs(sr) < 1e-7 && fabs(cr) > 1e-7)
	{
		return atan2(0, cr);
	}
	else if (fabs(sr) > 1e-7 && fabs(cr) < 1e-7)
	{
		return atan2(sr, 0);
	}
	else
	{
		return atan2(0, 0);
	}
}


Vector3d UR::tr_2_rpy(MatrixXd m)  //转移矩阵转位姿
{
	Vector3d rpy;
	rpy(2) = calcAtan2(-m(1, 2), m(2, 2));
	double sr = sin(rpy(2));
	double cr = cos(rpy(2));
	rpy(1) = calcAtan2(m(0, 2), -sr*m(1, 2)+cr*m(2,2));
	rpy(0) = calcAtan2(-m(0, 1), m(0, 0));
	return rpy;
}


VectorXd UR::tr_2_MCS(MatrixXd m)  //转移矩阵转直角坐标
{
	VectorXd posMCS(6);
	posMCS << m.block(0, 3, 3, 1), tr_2_rpy(m);
	return posMCS;
}


MatrixXd UR::rpy_2_tr(VectorXd posMCS) //直角坐标转转移矩阵
{
	MatrixXd tr(4, 4);
	tr << rpy_2_r(posMCS.segment(3, 3)), posMCS.head(3), MatrixXd::Zero(1, 3), 1;
	return tr;
}

MatrixXd UR::rpy_2_r(Vector3d rpy)  //位姿转旋转矩阵
{
	return Eigen::AngleAxisd(rpy(2), Vector3d::UnitX()).toRotationMatrix() * Eigen::AngleAxisd(rpy(1), Vector3d::UnitY()).toRotationMatrix() * Eigen::AngleAxisd(rpy(0), Vector3d::UnitZ()).toRotationMatrix();
}


void UR::get_Quintic_Coe(double q0, double qf, double dq0, double dqf,
								 double ddq0, double ddqf, double tf, Quintic_Coe &Coe)
{
	Coe.a0 = q0;
	Coe.a1 = dq0;
	Coe.a2 = ddq0 / 2.0;
	Coe.a3 = (20 * qf - 20 * q0 - (8 * dqf + 12 * dq0) * tf - (3 * ddq0 - ddqf) * tf * tf) / (2 * pow(tf, 3));
	Coe.a4 = (30 * q0 - 30 * qf + (14 * dqf + 16 * dq0) * tf + (3 * ddq0 - 2 * ddqf) * tf * tf) / (2 * pow(tf,
																										   4));
	Coe.a5 = (12 * qf - 12 * q0 - (6 * dq0 + 6 * dqf) * tf - (ddq0 - ddqf) * tf * tf) / (2 * pow(tf, 5));
}


//五次多项式求解
void UR::Quintic_Polynomi(double r, Quintic_Coe &coe, double *qptr)
{
	if (qptr)
		*qptr = coe.a0 + coe.a1 * r + coe.a2 * pow(r, 2) + coe.a3 * pow(r, 3) + coe.a4 * pow(r, 4) + coe.a5 * pow(r, 5);
}


void UR::calc_Interp_5_1_5(double q0, double q1, double Ts,
								   double maxVel, double maxAcc, double maxDecel, double maxJerk,
								   std::deque<double> &seq, double beginVel, double beginAcc)
{
	double dis = q1 - q0;
	if (!maxVel)
		return;

	//第一步：确定加（减）速段时间tAcc
	double tAcc = fabs((maxVel - beginVel) / maxAcc);
	double tDecel = fabs(maxVel / maxDecel);

	if (tAcc > maxVel / maxAcc)
	{
		tAcc = maxVel / maxAcc;
	}

	//第二步：若全程视作匀速运动，确定最大运动时间T
	double T = fabs((dis - tAcc / 2 * beginVel) / maxVel);
	if (!T)
	{
		return;
	}

	//第三步：确定匀速段时间tConst
	double tf = tAcc / 2 + T + tDecel / 2; //最终整个过程运行时间为tf
	double tConst = tf - tAcc - tDecel;

	//判断是否有匀速段
	if (tConst < 0.75 * Ts)
		tConst = 0;
	{
		//第四步：确定加速段、匀速段、减速段的插补步数（N1,N2,N3）,并确定最终运动时间
		size_t N1 = static_cast<size_t>(tAcc / Ts);
		size_t N2 = static_cast<size_t>(tConst / Ts);
		size_t N3 = static_cast<size_t>(tDecel / Ts);

		if (N1 == 0)
		{
			N1 = 1;
		}
		tAcc = N1 * Ts;
		tDecel = N3 * Ts;
		tConst = N2 * Ts;

		T = tConst + tAcc / 2 + tDecel / 2;

		//第五步：计算轴最终运行时的平均速度，就是有符号的maxVel
		double velConst = (dis - tAcc / 2 * beginVel) / T; //有正负号

		//第六步：轨迹插补，其中：用五次多项式对加速段和减速段进行插补
		seq.clear();
		double posInterp = 0; //用于暂存轴某周期的插补点位置

		//(1)加速段插补 [ 第 0 ~ N1 点]
		//计算五次多项式系数
		double accPosEnd; //轴加速段的结束点位置
		Quintic_Coe coe;  //轴的五次多项式
		accPosEnd = q0 + (velConst + beginVel) * tAcc / 2;
		get_Quintic_Coe(q0, accPosEnd, beginVel, velConst, beginAcc, 0, tAcc, coe);

		//生成链表
		for (size_t i = 0; i != (N1 + 1); ++i) //i用于记录加速段插补点的序号
		{
			Quintic_Polynomi(i * Ts, coe, &posInterp);
			seq.push_back(posInterp);
		}

		//(2)匀速段插补 [ 第 (N1+1) ~ (N1+N2) 点]
		//生成链表
		for (size_t j = 1; j != (N2 + 1); ++j) //j用于记录匀速段插补点的序号
		{
			posInterp += velConst * Ts;
			seq.push_back(posInterp);
		}

		//(3)减速段插补 [ 第 (N1+N2+1) ~ (N1+N2+N3) 点]
		//计算五次多项式系数
		get_Quintic_Coe(posInterp, q1, velConst, 0, 0, 0, tDecel, coe);

		//生成链表
		for (size_t k = 1; k != (N3 + 1); ++k) //k用于记录减速段插补点的序号
		{
			Quintic_Polynomi(k * Ts, coe, &posInterp);
			seq.push_back(posInterp);
		}
	}

	return;
}

