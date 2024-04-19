/*
 * tcpsocket.cpp
 *
 *  Created on: Oct 20, 2021
 *      Author: inexbot
 */
#include "tcpsocket.h"
#include <iostream>
#include <vector>
#include <string>
#include "stdio.h"
#include "robot_type.h"

tcpsocket* g_tcp = nullptr;
extern UR* g_UR;
extern General_6S* g_General_6s;

using namespace std;

vector<string> split(const string &s, const string &seperator)
		{
	vector<string> result;
	typedef string::size_type string_size;
	string_size i = 0;

	while(i != s.size())
	{
		//找到字符串中首个不等于分隔符的字母；
		int flag = 0;
		while(i != s.size() && flag == 0)
		{
			flag = 1;
			for(string_size x = 0; x < seperator.size(); ++x)
				if(s[i] == seperator[x]){
					++i;
					flag = 0;
					break;
				}
		}

		//找到又一个分隔符，将两个分隔符之间的字符串取出；
		flag = 0;
		string_size j = i;
		while(j != s.size() && flag == 0){
			for(string_size x = 0; x < seperator.size(); ++x)
				if(s[j] == seperator[x]){
					flag = 1;
					break;
				}
			if(flag == 0)
				++j;
		}
		if(i != j)
		{
			result.push_back(s.substr(i, j-i));
			i = j;
		}
	}
	return result;
		}

string doubleToString(double num)
{
	char str[256];
	sprintf(str, "%f", num);
	string result = str;
	return result;
}

void tcpsocket::initSocket()
{
	printf("initSocket\n");
	bool socketStatus =true;
	//创建一个TCP的socket
	if( (listenfd = socket(AF_INET,SOCK_STREAM,0)) == -1) {
		printf(" create socket error: %s (errno :%d)\n",strerror(errno),errno);
		return ;
	}
	//先把地址清空，检测任意IP
	memset(&servaddr,0,sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	servaddr.sin_port = htons(6789);

	unsigned int value = 1;
	setsockopt(listenfd,SOL_SOCKET,SO_REUSEADDR,(void *)&value,sizeof(value));

	if ( bind(listenfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) == -1) {
		printf(" bind socket error: %s (errno :%d)\n",strerror(errno),errno);
		return ;
	}
	//监听listenfd
	if( listen(listenfd,10) == -1) {
		printf(" listen socket error: %s (errno :%d)\n",strerror(errno),errno);
		return ;
	}

	if( (connfd = accept(listenfd, (struct sockaddr *)NULL, NULL))  == -1)
	{
		printf(" accpt socket error: %s (errno :%d)\n",strerror(errno),errno);
		return ;
	}

	while(socketStatus)
	{
		bzero(buff, sizeof(buff));
		//5、聊天
		if(0 == read(connfd, buff, sizeof(buff)))
		{
			if(strerror(errno) !=  strerror(4))
			{
				printf("connect  fail ");
				socketStatus =false;
				break;
			}
		}
		else
		{
			printf("recvbuf:%s\n", buff);
			std::string str =buff;
			doAction(str);
		}
		sleep(1);
	}
	if(socketStatus ==false )
	{
		close(listenfd);
		close(connfd);
		initSocket();
	}
}

void tcpsocket::moveAxis(int axis , int direction)
{
	printf("moveAxis:%d %d   \n", axis,direction);
	VectorXd target_point_joint_test(6);  ///目标位置,角度制
	VectorXd origin_point_joint_test(6);  ///初始位置,角度制
	VectorXd vel_current_joint_test(6);   ///当前速度,角度制
	VectorXd acc_current_joint_test(6);   ///当前加速度,角度制
	double pos_cur_ang[6];  ///当前位置角度值,角度制
	for(int i=0;i<6;i++)
	{
		pos_cur_ang[i] = g_General_6s->get_actual_position(i);  ///获取当前位置角度值
	}
	for(int i=0;i<6;i++)
	{
		origin_point_joint_test(i) = pos_cur_ang[i];  ///当前位置作为起始位置
		printf("pos_cur_ang:%f \n", pos_cur_ang[i]);
	}

	for(int i=0;i<6;i++)
	{
		target_point_joint_test(i)  = pos_cur_ang[i];
	}
	if(direction == -1)
	{
		target_point_joint_test(axis) = pos_cur_ang[axis] - 10;
	}else if(direction == 1)
	{
		target_point_joint_test(axis) = pos_cur_ang[axis] + 10;
	}

	vel_current_joint_test<<0,0,0,0,0,0;  ///设置当前速度
	acc_current_joint_test<<0,0,0,0,0,0;  ///设置当前加速度
	double Ts_joint_test = 0.001;  ///设置运动周期
	double velPerc_joint_test = 25;  ///设置速度百分比
	double accPerc_joint_test = 25;  ///设置加速度百分比
	double decPerc_joint_test = 25;  ///设置减速度百分比
	double jerkPerc_joint_test = 25;  ///设置雅可比速度百分比
	std::deque<double> trajectory_joint_test;
	///计算关节插补
	g_General_6s->move_joint_interp(target_point_joint_test,
			origin_point_joint_test, vel_current_joint_test, acc_current_joint_test, Ts_joint_test, velPerc_joint_test,
			accPerc_joint_test, decPerc_joint_test, jerkPerc_joint_test,trajectory_joint_test);

	if(!g_General_6s->get_power_on_status())  ///判断使能状态
		g_General_6s->power_on();  ///开启使能
	sleep(2);
	///插补轨迹写入运动队列
	g_General_6s->set_angle_deque(trajectory_joint_test);  ///设置运动轨迹

}

void tcpsocket::doAction(std::string str)
{
	vector<string> v = split(str, " "); //可按多个字符来分隔;
	for(vector<string>::size_type i = 0; i != v.size(); ++i)
	{
		cout << endl;
	}
	if(v.at(0) == "deadman")
	{
		if(v.at(1) == "1")
		{
			g_General_6s->power_on();
		}
		if(v.at(1) == "0")
		{
			g_General_6s->power_off();
		}
	}
	else if(v.at(0) == "Axis")
	{
		moveAxis( atoi(v.at(1).c_str())-1, atoi(v.at(2).c_str()) );
	}
	else if(v.at(0) == "programName") //open programme
	{
		currentFile = v.at(1); //xxxx.text
	}
	else if(v.at(0) == "createFile") //createFile
	{
		createTextFile(v.at(1)); //xxxx.text
	}
	else if(v.at(0) == "remove")
	{
		deleteFile(v.at(1));
	}
	else if(v.at(0) == "runCurProgarmme")
	{
		if(v.at(1) =="start")
		{
			if(currentFile ==" ")
			{
				return;
			}
			runFile(currentFile);
		}
		else if(v.at(1) =="stop")
		{

		}
	}
	else if(v.at(0) =="insert")
	{
		if(v.at(1) =="MOVJ")
		{
			if(v.size() < 5)
			{
				return;
			}

			double cur_angle_double[6];


			for(int i=0;i<6;i++)
			{
				if(g_General_6s->get_actual_position(i) ==0)
				{
					  cout<<"insert position error  ";
                      return;
				}
				cur_angle_double[i] = g_General_6s->get_actual_position(i);  ///获取当前位置角度值
			}

			string saveStr = doubleToString(cur_angle_double[0])+" "+doubleToString(cur_angle_double[1])+" "+doubleToString(cur_angle_double[2])
							 +" "+doubleToString(cur_angle_double[3])+" "+doubleToString(cur_angle_double[4])+" "+doubleToString(cur_angle_double[5]);

			std::string insertstr =v.at(1)+" "+v.at(2)+" "+v.at(3)+" "+v.at(4)+" "+saveStr;
			storeFile(currentFile,insertstr);
		}
		else if(v.at(1) =="MOVL")
		{
			if(v.size() < 5)
			{
				return;
			}

			double cur_angle_double[6];

			for(int i=0;i<6;i++)
			{
				if(g_General_6s->get_actual_position(i) ==0)
				{
					cout<<"insert position error  ";
					return;
				}
				cur_angle_double[i] = g_General_6s->get_actual_position(i);  ///获取当前位置角度值
			}

			string saveStr = doubleToString(cur_angle_double[0])+" "+doubleToString(cur_angle_double[1])+" "+doubleToString(cur_angle_double[2])
							 +" "+doubleToString(cur_angle_double[3])+" "+doubleToString(cur_angle_double[4])+" "+doubleToString(cur_angle_double[5]);

			std::string insertstr =v.at(1)+" "+v.at(2)+" "+v.at(3)+" "+v.at(4)+" "+saveStr;
			storeFile(currentFile,insertstr);
		}
		else if(v.at(1) =="DIN")
		{
			if(v.size() < 4)
			{
				return;
			}
			std::string insertstr =v.at(1)+" "+v.at(2)+" "+v.at(3);
			storeFile(currentFile,insertstr);
		}
		else if(v.at(1) =="DOUT")
		{
			if(v.size() < 4)
			{
				return;
			}
			std::string insertstr =v.at(1)+" "+v.at(2)+" "+v.at(3);
			storeFile(currentFile,insertstr);
		}
	}
}

void tcpsocket::runCMD(const std::string &temp)
{
	vector<string> strlist = split(temp, " "); //可按多个字符来分隔;
	if(strlist.at(0) =="MOVJ")
	{
		VectorXd targetPoint(6);
		if(strlist.size()<10)
		{
			cout<<" MOVJ list  size error        ";
			return;
		}
		for(int i=0;i<6;i++)
		{
			targetPoint(i) = atof(strlist.at(4+i).c_str());
		}
		runAxistoPOS_MOVEJ(targetPoint);
	}
	else if(strlist.at(0) =="MOVL")
	{
		VectorXd targetPoint(6);
		if(strlist.size()<10)
		{
			cout<<" MOVL list  size error        ";
			return;
		}
		for(int i=0;i<6;i++)
		{
			targetPoint(i) =atof(strlist.at(4+i).c_str());
		}
		runAxistoPOS_MOVEL(targetPoint);
	}
	else if(strlist.at(0) =="DIN")
	{

	}
	else if(strlist.at(0) =="DOUT")
	{

	}

}

void tcpsocket::runFile(std::string filepath)
{

	ifstream ous(filepath.c_str());
	if (access(filepath.c_str(), 0) != 0)
	{
		cout<<"access  No file         "<<endl;
	}
	string line;
	if(ous) // 有该文件
	{
		while (getline (ous, line)) // line中不包括每行的换行符
		{
			cout << line << endl;
			runCMD(line);
		}
	}
	else // 没有该文件
	{
		cout <<"no such file" << endl;
	}

	 ous.close();
}

void tcpsocket::runAxistoPOS_MOVEL(VectorXd targetPoint)
{

	double pos_cur_ang[6];  ///当前位置角度值,角度制
	VectorXd pos_acs(6);

	for(int i=0;i<6;i++)
	{
		pos_cur_ang[i] = g_General_6s->get_actual_position(i);  ///获取当前位置角度值
	}
	for(int i=0;i<6;i++)
	{
		pos_acs(i) = pos_cur_ang[i];
	}
	MatrixXd trans_matrix_line_test;
	g_General_6s->calc_forward_kin(pos_acs,trans_matrix_line_test);  ///计算当前位置的正解矩阵
	VectorXd origin_point_line_test = tr_2_MCS(trans_matrix_line_test);  ///计算当前位置的直角坐标

	MatrixXd target_point_line_test ;
	g_General_6s->calc_forward_kin(targetPoint,target_point_line_test);  ///计算当前位置的正解矩阵
	VectorXd target_point_line =tr_2_MCS(target_point_line_test);


	VectorXd origin_acs_line_test = pos_acs;  ///起始位置的关节坐标

	double velc_line_test = 0;
	double accc_line_test = 0;
	double Ts_line_test = 0.001;
	double maxVel_line_test = 2;
	double maxAcc_line_test = 6;
	double maxDecel_line_test = -6;
	double maxJerk_line_test = 200;///设置直线插补的参数

	std::deque<double> trajectory_line_test;
	///直线插补
	g_General_6s->move_line_interp(target_point_line,
			origin_point_line_test,origin_acs_line_test, velc_line_test, accc_line_test,
			Ts_line_test, maxVel_line_test, maxAcc_line_test, maxDecel_line_test,
			maxJerk_line_test, trajectory_line_test);

	VectorXd pos_cur(6);
	double cur_angle_double[6];
	VectorXd mcs_cur;
	if(!g_General_6s->get_power_on_status())
		g_General_6s->power_on();
	sleep(2);
	///插补轨迹写入运动队列
	g_General_6s->set_angle_deque(trajectory_line_test);
	while(g_General_6s->get_power_on_status())
	{
		for(int i=0;i<6;i++)
		{
			cur_angle_double[i] = g_General_6s->get_actual_position(i);  ///获取当前位置角度值
		}
		for(int i=0;i<6;i++)
		{
			pos_cur[i] = cur_angle_double[i];
		}
		g_General_6s->calc_forward_kin(pos_cur,trans_matrix_line_test);
		mcs_cur = tr_2_MCS(trans_matrix_line_test);
		printf("line_move_test mcs ");
		for(int i=0;i<3;i++)
		{
			printf("%lf ",trans_matrix_line_test(i,3));  ///运动过程中输出实时直角坐标
		}
		printf("\n");
		if(g_General_6s->get_angle_deque().empty() && g_General_6s->get_power_on_status())
			g_General_6s->power_off();
		sleep(2);
	}

}

void tcpsocket::runAxistoPOS_MOVEJ( VectorXd targetPoint)
{
	VectorXd target_point_joint_test(6);  ///目标位置,角度制
	VectorXd origin_point_joint_test(6);  ///初始位置,角度制
	VectorXd vel_current_joint_test(6);   ///当前速度,角度制
	VectorXd acc_current_joint_test(6);   ///当前加速度,角度制

	for(int i=0;i<6;i++)
	{
		origin_point_joint_test[i] = g_General_6s->get_actual_position(i);  ///获取当前位置角度值
	}


	///目标位置设置为当前位置增加偏移
	target_point_joint_test(0) = targetPoint[0];
	target_point_joint_test(1) = targetPoint[1];
	target_point_joint_test(2) = targetPoint[2];
	target_point_joint_test(3) = targetPoint[3];
	target_point_joint_test(4) = targetPoint[4];
	target_point_joint_test(5) = targetPoint[5];  ///从当前位置开始运动固定角度
	vel_current_joint_test<<0,0,0,0,0,0;  ///设置当前速度
	acc_current_joint_test<<0,0,0,0,0,0;  ///设置当前加速度
	double Ts_joint_test = 0.001;  ///设置运动周期
	double velPerc_joint_test = 25;  ///设置速度百分比
	double accPerc_joint_test = 25;  ///设置加速度百分比
	double decPerc_joint_test = 25;  ///设置减速度百分比
	double jerkPerc_joint_test = 25;  ///设置雅可比速度百分比
	std::deque<double> trajectory_joint_test;
	///计算关节插补
	g_General_6s->move_joint_interp(target_point_joint_test,
			origin_point_joint_test, vel_current_joint_test, acc_current_joint_test, Ts_joint_test, velPerc_joint_test,
			accPerc_joint_test, decPerc_joint_test, jerkPerc_joint_test,trajectory_joint_test);

	if(!g_General_6s->get_power_on_status())  ///判断使能状态
		g_General_6s->power_on();  ///开启使能
	sleep(2);
	///插补轨迹写入运动队列
	g_General_6s->set_angle_deque(trajectory_joint_test);  ///设置运动轨迹
	double cur_angle_double[6];
	while(g_General_6s->get_power_on_status())  ///循环检测使能状态
	{
		for(int i=0;i<6;i++)
		{
			cur_angle_double[i] = g_General_6s->get_actual_position(i);  ///获取当前位置角度值
		}
		printf("joint_move_test %lf %lf %lf %lf %lf %lf \n",cur_angle_double[0],cur_angle_double[1],cur_angle_double[2],cur_angle_double[3],cur_angle_double[4],cur_angle_double[5]);
		if(g_General_6s->get_angle_deque().empty() && g_General_6s->get_power_on_status())  ///判断运动状态
			g_General_6s->power_off();  ///关闭使能
		sleep(1);
	}
}

void tcpsocket::deleteFile(std::string filepath)
{
	remove(filepath.c_str());
}

void tcpsocket::storeFile(std::string filepath, std::string  fileText)
{
	std::ofstream  temp(filepath,ios::app);
	temp<<fileText.c_str()<<std::endl;
	temp.close();
}

void tcpsocket::createTextFile(std::string jobFileName)
{
	ofstream outfile(jobFileName, ios::trunc);
	outfile.close();

}





