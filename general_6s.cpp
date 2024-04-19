#include "robot_type.h"

void General_6S::calc_forward_kin(const VectorXd& posACS,MatrixXd& transMatrix)
{
	VectorXd acs_rad(6);

	transMatrix = (MatrixXd::Identity( 4, 4));

	for(int i=0;i<6;i++)
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

void General_6S::calc_inverse_kin(const MatrixXd& transMatrix,const VectorXd& posLast, VectorXd& posACS)
{
	double d1,d2,d4,d6,a1,a2,a3;
	if ( (transMatrix.rows()!=4) || (transMatrix.cols() !=4)||(posLast.rows()
			!=6)||(posACS.rows() !=6))
	{
		return;
	}
	//vars defined temporarily
	d1=dh.d[0];
	d2=dh.d[1];
	d4=dh.d[3];
	d6=dh.d[5];
	a1=dh.a[0];
	a2=dh.a[1];
	a3=dh.a[2];
	MatrixXd T(transMatrix);

	/*		2021-11-26新增		*/
	VectorXd pLast(6);//读到的角度，经下面处理后，成为theta值
	for(int i=0;i<6;i++)
		pLast(i) = deg2rad(posLast(i));
	/*		2021-11-26新增结束		*/

	double nx(T(0,0)), ny(T(1,0)), nz(T(2,0));
	double ox(T(0,1)), oy(T(1,1)), oz(T(2,1));
	double ax(T(0,2)), ay(T(1,2)), az(T(2,2));
	double px(T(0,3)), py(T(1,3)), pz(T(2,3));

	//读取theat5偏移量
	double theta5_offset = dh.theta[4];

	//solve for theta1
	double theta1_1,theta1_2;
	double n = px-d6*ax;
	double m = py-d6*ay;
	double temp_var = pow(m,2) + pow(n,2) - pow(d2,2);
	if(temp_var < 0.0L)
	{
		return;
	}
	if((fabs((py-d6*ay)) < 10e-13 ) && (fabs((px-d6*ax)) < 10e-13))
	{
		theta1_1 = pLast(0);
		theta1_2 = theta1_1;
	}
	else
	{
		theta1_1 = atan2(n*d2+m*sqrt(temp_var),-m*d2+n*sqrt(temp_var));
		theta1_2 = atan2(n*d2-m*sqrt(temp_var),-m*d2-n*sqrt(temp_var));
	}
	double theta1;

	/*if (theta1_1 <= 0.0L)
		theta1_2 = theta1_1 + M_PI;
	else
		theta1_2 = theta1_1 - M_PI;*/

	if(fabs(theta1_1-pLast(0))>fabs(theta1_2-pLast(0)))
		theta1 = theta1_2;
	else
		theta1 = theta1_1;

	// the limit of theta1 according to the reference
	posACS(0) = theta1;
	//solve for theta3

	double k1 = cos(theta1)*px+sin(theta1)*py-a1-d6*(cos(theta1)*ax+sin(theta1)*ay);
	double k2 =  pz - d1 - d6*az;
	double k3 = pow(k1, 2) + pow(k2, 2) - pow(a2, 2) - pow(a3, 2) - pow(d4, 2);
	double k4 = k3/(2*a2);
	temp_var = pow(a3, 2) + pow(d4, 2) - pow(k4, 2);
	if(temp_var < 0.0L)
	{
		return;
	}
	double delta = sqrt(pow(a3, 2) + pow(d4, 2) - pow(k4, 2));
	double theta3_1 = atan2(d4, a3) + atan2(delta, k4);
	double theta3_2 = atan2(d4, a3) - atan2(delta, k4);
	double theta3;

	if(fabs(theta3_1-pLast(2))>fabs(theta3_2-pLast(2)))
		theta3 = theta3_2;
	else
		theta3 = theta3_1;

	// the limit of theta3 according to the reference
	posACS(2) = theta3;

	//solve for theta2
	k1 = cos(theta1)*px + sin(theta1)*py - a1 - d6*(cos(theta1)*ax + sin(theta1)*ay);
	k2 = -d1 + pz - d6*az;
	double a = d4*cos(theta3) - a3*sin(theta3);
	double b = d4*sin(theta3) + a2 +a3*cos(theta3);
	//已经加入了theta2的offset -pi/2    theta(运算) = theta(电机) - pi/2
	double theta2_1;
	if((fabs(a*k1 + b*k2) < 10e-13)  && (fabs(b*k1 - a*k2) < 10e-13))
		theta2_1 = pLast(1);
	else
		theta2_1 = atan2((a*k1 + b*k2),(b*k1 - a*k2)) - M_PI/2.0;
	double theta2;

	theta2 = theta2_1;
	// the limit of theta2 according to the reference
	posACS(1) = theta2;

	//solve for theta4

	k1 = sin(theta1)*ax - cos(theta1)*ay;
	k2 = cos(theta1)*cos(theta2 + M_PI/2.0 + theta3)*ax + sin(theta1)*
			cos(theta2 + M_PI/2.0 + theta3)*ay + sin(theta2 + M_PI/2.0 + theta3)*az;

	double theta4;
	double theta4_2;
	//此处的判断阈值不能过小，过小的话，当0/0时，它无法识别出来

	if((fabs(k1) < 10e-13) && (fabs(k2) < 10e-13))
	{
		theta4 = pLast(3);
		//cout << "A" << endl;
	}
	else
	{
		double theta4_1 = atan2(k1,k2);
		if(theta4_1 > 0.0L)
			theta4_2 = theta4_1 - M_PI;
		else
			theta4_2 = theta4_1 + M_PI;
		if(fabs(theta4_1-pLast(3))>fabs(theta4_2-pLast(3)))
			theta4 = theta4_2;
		else
			theta4 = theta4_1;
	}

	// the limit of theta4 according to the reference
	posACS(3) = theta4;

	//solve for theta5
	double k1_1 = sin(theta1)*sin(theta4) + cos(theta1)*cos(theta4)*cos(theta2 + M_PI/2.0 + theta3);
	double k1_2 = -cos(theta1)*sin(theta4) + sin(theta1)*cos(theta4)*cos(theta2 + M_PI/2.0 + theta3);
	double k1_3 = cos(theta4)*sin(theta2 + M_PI/2.0 + theta3);
	k1 = k1_1*ax + k1_2*ay + k1_3*az;
	k2 = cos(theta1)*sin(theta2 + M_PI/2.0 + theta3)*ax + sin(theta1)*sin(theta2 + M_PI/2.0 + theta3)*
			ay - cos(theta2 + M_PI/2.0 + theta3)*az;
	double theta5_1;
	if ((fabs(k1) < 10e-13) && (fabs(k2) < 10e-13))
	{
		theta5_1 = pLast(4);
	}
	else
	{
		if (fabs(theta5_offset - M_PI / 2) < 10e-4)	//判断第五轴是垂直还是平行
		{
			theta5_1 = atan2(-k1, k2) - M_PI / 2.0;
		}
		else if (fabs(theta5_offset) < 10e-4)
		{
			theta5_1 = atan2(-k1, k2);
		}
	}
	double theta5;

	theta5 = theta5_1;
	// the limit of theta5 according to the reference
	posACS(4) = theta5;

	//solve for theta6
	double k1_4=sin(theta4)*cos(theta2+ M_PI/2.0+theta3)*cos(theta1)-cos(theta4)*sin(theta1);
	double k1_5=sin(theta4)*cos(theta2+ M_PI/2.0+theta3)*sin(theta1)+cos(theta4)*cos(theta1);
	double k1_6=sin(theta4)*sin(theta2+ M_PI/2.0+theta3);
	k2=-k1_4*ox - k1_5*oy - k1_6*oz;
	k1=-k1_4*nx - k1_5*ny - k1_6*nz;

	double theta6_1;
	if((fabs(k1) < 10e-13) && (fabs(k2) < 10e-13))
		theta6_1 = pLast(5);
	else
		theta6_1 = atan2(k1, k2);
	double theta6;

	theta6 = theta6_1;
	// the limit of theta6 according to the reference
	posACS(5) = theta6;

	for(int i=0;i<6;i++)
		posACS[i] = rad2deg(posACS[i]);
}


