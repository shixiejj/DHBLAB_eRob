#include<unistd.h>
#include<sys/socket.h>
#include<sys/types.h>
#include <sys/ioctl.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "server.cpp"

#include <fcntl.h>
#include <unistd.h>	
#include <errno.h>

#include <cmath>
#include <iostream>
#include<vector>
#include <map>
#include <time.h>
// #include "server.cpp"
#include <pthread.h>

#include "slave/slave.h"
#include "robot_type.h"
#include "tcpsocket.h"
#include "force_sensor.h"
#include "force_sensor_uart.h"


using namespace Eigen;
using namespace std;



UR* g_UR = nullptr;
General_6S* g_General_6s = nullptr;
std::vector<Slave> slave_vector;
pthread_t thread;

T521G2 sensor;
SerialPort sp;

typedef void (*FnPtr)(void);
typedef string (*StrFnPtr)(void);
typedef string (*StrFnStrPtr)(string);

string joint_tcp_action(string control_str);


void joints_move(VectorXd target_point_joint);



#define PI 3.1415926

extern tcpsocket* g_tcp;
extern void start_ecm(int argc, char* argv[]);
extern void loop_display();


// 机械臂是否在动作
bool robot_is_moving = false;

// // 机械臂各个关节额定力矩 平均负载力矩
// const double joints_rated_torque[] = {133, 133, 133, 13.5, 13.5, 13.5};

// // 机械臂各个关节额定力矩 启停峰值扭矩
// const double joints_rated_torque[] = {155, 155, 155, 27, 27, 27};

// 机械臂各个关节额定力矩 启停峰值扭矩
const double joints_rated_torque[] = {465, 465, 465, 81, 81, 81};

// 机械臂各个关节减速比 
const double joints_reduce_ratio[] = {101, 101, 101, 101, 101, 101};

// 机械臂各个关节初始偏置
const double joint_bias[] = {
    0,
    0,
    0,
    0,
    0,
    0,
};

//3:-90, 5:90

// 机械臂各个关节初始偏置
// const double joint_bias[] = {
//     13.1463,
//     -17.7446,
//     -81.168,
//     0,
//     16.3205 + 90,
//     140.364,
// };

void tcp_server(void)
{
    RbServer server(8000, '>', joint_tcp_action);
}

// 编码器分辨率
const double encoder_resolution = 524288;

// 机械臂物理方向，逆时针为正，顺时针为负
const int joint_direction[] = {1, 1, 1, 1, 1, 1};

const int refer_direction[] = {1, -1, -1, 1, 1, 1};

// const int
// 调整关节加和
double joint_add(int axis, double initial, double deg)
{
    return initial + joint_direction[axis]*deg;
}

// 获取机械臂真实角度
double joint_deg(int axis, double delta_deg)
{
    return joint_bias[axis] + delta_deg * joint_direction[axis];
}

// 获取机械臂变角
double joint_delta(int axis, double real_joint_deg)
{
    return (real_joint_deg - joint_bias[axis]) * joint_direction[axis];
}

double get_pos(int axes)
{
    return joint_delta(axes, g_UR->get_actual_position(axes));
}

void printDeque(deque<double>& d) {
	for (deque<double>::const_iterator it = d.begin(); it != d.end(); it++) {
		//使用const_iterator，容器中的数据就不可以修改了。
		cout << *it << " " <<endl;
	}
	cout << endl;
}


void cycle_run()
{
	g_UR->cycle_run();
}


//调用程序
void line_move_UR()
{
	double pos_cur_ang[6];  ///当前位置角度值,角度制
	VectorXd pos_acs(6);

	for(int i=0;i<6;i++)
	{
		pos_cur_ang[i] = g_UR->get_actual_position(i);  ///获取当前位置角度值
	}


	for(int i=0;i<6;i++)
	{
		pos_acs(i) = pos_cur_ang[i];
	}

    std::cout<<"当前位置角度值："<<pos_acs<<endl;


	MatrixXd trans_matrix_line_test;
	g_UR->calc_forward_kin(pos_acs,trans_matrix_line_test);  ///计算当前位置的正解矩阵

	VectorXd origin_point_line_test = tr_2_MCS(trans_matrix_line_test);  ///计算当前位置的直角坐标
	VectorXd target_point_line_test = origin_point_line_test;
	//target_point_line_test[1] = target_point_line_test[1] + 100;  ///设置目标位置

    target_point_line_test[0] =673;
    target_point_line_test[1] =-141;
    target_point_line_test[2] =269;
    // target_point_line_test[3] =-3.14;
    // target_point_line_test[4] =0;
    // target_point_line_test[5] =-1.57;


    /*
    //2023.7.27自己输入目标位置
    string position;
    cout << "用户输入格式 x|y|z" << endl;
    cin >> position;
    position.append("|3.14|0.0001|0.0001|");
    VectorXd target_point_line_test(6); //目标位置x-y-z-r-p-y
    int split_pos = -1;
    int i = 0;
    while (string::npos != (const long unsigned int)(split_pos = position.find("|")) and strcmp("|", position.c_str()) != 0)
    {
        // cout << trajectory << endl;
        double joint_position = atof(position.substr(0, split_pos).c_str());
        target_point_line_test(i) = joint_position; // x-y-z-r-p-y
        // cout << joint_position << endl;
        position = position.substr(split_pos + 1);
        i++;
        if (5 == i)
        {
            break;
        }
    }  
    target_point(3) = -1.57;
    target_point(4) =     0;
    target_point(5) = -2.61;
    */



    std::cout<<"trans_matrix_line_test："<<trans_matrix_line_test<<endl;

    std::cout<<"target_point_line_test："<<target_point_line_test<<endl;


	VectorXd origin_acs_line_test = pos_acs;  ///起始位置的关节坐标

	double velc_line_test = 0;
	double accc_line_test = 0;
	double Ts_line_test = 0.001;
	double maxVel_line_test = 8;     //改变机械臂运行速度（之前是1）
	double maxAcc_line_test = 3;
	double maxDecel_line_test = -3;
	double maxJerk_line_test = 100;///设置直线插补的参数

	std::deque<double> trajectory_line_test;
	///直线插补
	g_UR->move_line_interp(target_point_line_test,
			origin_point_line_test,origin_acs_line_test, velc_line_test, accc_line_test,
			Ts_line_test, maxVel_line_test, maxAcc_line_test, maxDecel_line_test,
			maxJerk_line_test, trajectory_line_test);


	VectorXd pos_cur(6);
	double cur_angle_double[6];
	VectorXd mcs_cur;

	if(!g_UR->get_power_on_status())
		g_UR->power_on();
	sleep(2);

    //判断是否上电
    if(g_UR->get_power_on_status())
    {
        cout << "机器人已上电！"<<endl;
    }
    else
    {
        cout << "机器人未上电！"<<endl;
    }


    //std::cout<<"trajectory_line_test："<<trajectory_line_test<<endl;


	///插补轨迹写入运动队列
	g_UR->set_angle_deque(trajectory_line_test);


    //2023.7.27输出

    int k=0;
    
	while(g_UR->get_power_on_status())
    {

        // k++;
	
		for(int i=0;i<6;i++)
		{
			cur_angle_double[i] = g_UR->get_actual_position(i);  ///获取当前位置角度值
		}
		for(int i=0;i<6;i++)
		{
			pos_cur[i] = cur_angle_double[i];
		}
		g_UR->calc_forward_kin(pos_cur,trans_matrix_line_test);


        //获取的
        //std::cout<<"各轴角度："<<pos_cur<<std::endl;

		mcs_cur = tr_2_MCS(trans_matrix_line_test);
		printf("line_move_test mcs ");
		for(int i=0;i<3;i++)
		{
			printf("%lf ",trans_matrix_line_test(i,3));  ///运动过程中输出实时直角坐标
		}
		printf("\n");

        sleep(1);

		// if(k > 10)
		//  	g_UR->power_off();   

		// if(g_UR->get_angle_deque().empty() && g_UR->get_power_on_status())
		//  	g_UR->power_off();

        if(g_UR->get_angle_deque().empty())
             g_UR->power_off();

    }	
    

}


//自己写直线程序
void cartesian_space_linear_move()
{
	if (not robot_is_moving)
    {
        // 机器人同时只可执行一次动作
        robot_is_moving = true;
    }
    else
    {
        return;
    }

	double pos_cur_ang[6];  ///当前位置角度值,角度制
	VectorXd pos_acs(6);

	for(int i=0;i<6;i++)
	{
		pos_cur_ang[i] = g_UR->get_actual_position(i);  ///获取当前位置角度值
	}
	for(int i=0;i<6;i++)
	{
		pos_acs(i) = pos_cur_ang[i];
	}
	MatrixXd trans_matrix_line_test;
    VectorXd XYZ_position(3);
	g_UR->calc_forward_kin(pos_acs,trans_matrix_line_test);  ///计算当前位置的正解矩阵

	VectorXd origin_point_line_test = tr_2_MCS(trans_matrix_line_test);  ///计算当前位置的直角坐标
    origin_point_line_test(3)=1.57;
    origin_point_line_test(4)=0;
    origin_point_line_test(5)=3.14;
	cout<<"origin_point_line_test:"<<origin_point_line_test<<endl;

	VectorXd target_point_line_test = origin_point_line_test;

	//指定xyzRYZ   （400，0，500，3.14，0，0）
    target_point_line_test(0)=580;
    target_point_line_test(1)=0;
    target_point_line_test(2)=300;

	cout<<"target_point_line_test:"<<target_point_line_test<<endl;


	double max_step=0.01;    //最大插补步长
	VectorXd delta(6);        //相邻两点位置变化 
	VectorXd Inter_points(6);    //各位置插补点数
	int maxInter=0;            //最大插补点数
	VectorXd temp_xyzRPY=origin_point_line_test;     // 用于暂存插补点位置
    VectorXd point_pos(6);                           //存储逆解结果
    VectorXd pLast=pos_acs;
	deque<double> trajectory_line_test;              //总的各轴运动轨迹
    
	for(int i=0;i<6;i++)
	{
		delta(i)=target_point_line_test(i)-origin_point_line_test(i);
		Inter_points(i)=floor(delta(i)/max_step);
		
		if(abs(Inter_points(i))>maxInter)
		{
			maxInter=abs(Inter_points(i));           //最大插补点数
		}
	}

	cout<<"maxInter："<<maxInter<<endl;

	//生成链表
	for(int step=0;step<=maxInter;step++)
	{
		for(int i=0;i<6;i++)
		{
			temp_xyzRPY(i) =  origin_point_line_test(i)+step*delta(i)/maxInter;
		}
        g_UR->calc_inverse_kin(g_UR->rpy_2_tr(temp_xyzRPY),pLast,point_pos);
        pLast=point_pos;
		for (int ai=0; ai!=6; ++ai) /*1-n轴*/
		{
            trajectory_line_test.push_back(point_pos[ai]);
		}
	}   


	VectorXd pos_cur(6);
	double cur_angle_double[6];
	VectorXd mcs_cur;
	if(!g_UR->get_power_on_status())
		g_UR->power_on();
	sleep(2);
	///插补轨迹写入运动队列

	cout<<"开始运动"<<endl;

	g_UR->set_angle_deque(trajectory_line_test);


	while(g_UR->get_power_on_status())
	{
		for(int i=0;i<6;i++)
		{
			cur_angle_double[i] = g_UR->get_actual_position(i);  ///获取当前位置角度值
		}
		for(int i=0;i<6;i++)
		{
			pos_cur[i] = cur_angle_double[i];
		}
		g_UR->calc_forward_kin(pos_cur,trans_matrix_line_test);
		mcs_cur = tr_2_MCS(trans_matrix_line_test);
		printf("line_move_test mcs ");
		for(int i=0;i<3;i++)
		{
			printf("%lf ",trans_matrix_line_test(i,3));  ///运动过程中输出实时直角坐标
		}
		printf("\n");
		if(g_UR->get_angle_deque().empty() && g_UR->get_power_on_status())
			g_UR->power_off();
		sleep(2);
	}

    cout << "当前角度 deg:" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << g_UR->get_actual_position(i) << "  ";
    }
	cout << endl;

	robot_is_moving = false;

}


void cycle_move_UR()
{
    //定义一个圆弧
	VectorXd cycle_center(3);      //圆弧圆心
	// cycle_center[0] = 473;
	// cycle_center[1] = -141;
	// cycle_center[2] = 450;
    cycle_center[0] = 400;
	cycle_center[1] = 300;
	cycle_center[2] = 400;
	double radius = 200;           //圆弧半径
    double cycle_rad=2*M_PI;       //圆弧弧度
	
    
    double pos_cur_ang[6];   //当前位置角度值,角度制
	VectorXd pos_acs(6);     // 当前位置各关节角度

	for(int i=0;i<6;i++)
	{
		pos_cur_ang[i] = g_UR->get_actual_position(i);  //获取当前位置角度值
	}
	for(int i=0;i<6;i++)
	{
		pos_acs(i) = pos_cur_ang[i];                 //当前位置各关节角度
	}
	MatrixXd trans_matrix_line_test;
	g_UR->calc_forward_kin(pos_acs,trans_matrix_line_test);         ///计算当前位置的正解矩阵
    VectorXd origin_point_test = tr_2_MCS(trans_matrix_line_test);  ///计算当前位置的直角坐标

    VectorXd cycle_start_point = origin_point_test;          //圆弧开始位置直角坐标
    VectorXd cycle_start_pos=pos_acs;                        //圆弧开始位置关节角度

    cycle_start_point[0] = cycle_center[0]+radius*cos(0);
    cycle_start_point[1] = cycle_center[1]+radius*sin(0);
    cycle_start_point[2] = cycle_center[2];
    // cycle_start_point[3] = 3.14;
	// cycle_start_point[4] = 0;
	// cycle_start_point[5] = -1.57;

	VectorXd cycle_end_point = origin_point_test;          //圆弧结束位置直角坐标
    VectorXd cycle_end_pose = pos_acs;                     //圆弧结束位置关节角度
    cycle_end_point[0] = cycle_center[0]+radius*cos(cycle_rad);
    cycle_end_point[1] = cycle_center[1]+radius*sin(cycle_rad);
    cycle_end_point[2] = cycle_center[2];
    // cycle_end_point[3] = -3.14;
	// cycle_end_point[4] = 0;
	// cycle_end_point[5] = 1.57;

    cout<<"origin_point_test:"<<origin_point_test<<endl;
    cout<<"cycle_start_point："<<cycle_start_point<<endl;

    g_UR->calc_inverse_kin(g_UR->rpy_2_tr(cycle_start_point), pos_acs, cycle_start_pos); // 逆解得到圆弧起点位置各关节角度值

    cout<<"cycle_start_pos："<<cycle_start_pos<<endl;

    // 实体机器人移动
    joints_move(cycle_start_pos); // 使用机器人运动关节差补move_joint_interp方式移动机械臂到圆弧起点位置

    for(int i=0;i<6;i++)
	{
		pos_cur_ang[i] = g_UR->get_actual_position(i);  //获取当前位置角度值
	}
	for(int i=0;i<6;i++)
	{
		pos_acs(i) = pos_cur_ang[i];                 //当前位置各关节角度
	}

	g_UR->calc_forward_kin(pos_acs,trans_matrix_line_test);         ///计算当前位置的正解矩阵
    cycle_start_point = tr_2_MCS(trans_matrix_line_test);           ///计算当前位置的直角坐标
    cycle_start_pos=pos_acs;                                        //圆弧开始位置关节角度

    cout<<"圆弧开始位置关节角度:"<<cycle_start_pos<<endl;

    std::deque<double> rlst;
    std::deque<double> rlstMcs;
	double velc_cycle_test = 0;
	double accc_cycle_test = 0;
	double Ts_cycle_test = 0.001;
	double maxVel_cycle_test = 3;     //改变机械臂运行速度（之前是1）
	double maxAcc_cycle_test = 1;
	double maxDecel_cycle_test = -1;
	double maxJerk_cycle_test = 100;  //设置直线插补的参数
    double displacement=200;
    //double displacement=2;  //调式       用0-2PI插值

    //0-1用五次多项式插补得到轨迹插补比例
    g_UR->calc_Interp_5_1_5(0, 1, Ts_cycle_test, maxVel_cycle_test/displacement, maxAcc_cycle_test/displacement,
          maxDecel_cycle_test/displacement, maxJerk_cycle_test/displacement, rlstMcs, velc_cycle_test / displacement, accc_cycle_test / displacement);

    rlst = rlstMcs;

    //球面线性插补
    int n = 6;
	Matrix3d mOrigin = g_UR->rpy_2_r(cycle_start_point.segment(3,3));      //得到初始位置的姿态旋转矩阵
	Matrix3d mTarget = g_UR->rpy_2_r(cycle_end_point.segment(3,3));        //得到目标位置的姿态旋转矩阵
	Vector3d pOrigin = cycle_start_point.head(3);                          //提取前3个元素，得到圆弧开始位置xyz坐标
	Vector3d pTarget = cycle_end_point.head(3);                            //提取前3个元素，得到圆弧结束位置xyz坐标
	Vector3d pr;
	MatrixXd Tr(4, 4);
	VectorXd posr(6);
	VectorXd posACS(n);
	VectorXd pLast(n);
	Quaternion<double> qr;
    Quaternion<double> qOrigin(mOrigin);
	Quaternion<double> qTarget(mTarget);
    std::deque<double> trajectory_joint;       //圆弧插补各轴轨迹序列

	for (std::deque<double>::iterator rit = rlst.begin(); rit != rlst.end(); ++rit)
	{
		double r=*rit;
		//qr = qOrigin.slerp(r, qTarget);                         //旋转四元数的球面插值Rw1.slerp(t,Rw2),t在(0.0 - 1.0)范围,当成标量的话相当于Rw1 + t*(Rw2-Rw1)
        qr = qOrigin;

        pr[0] = cycle_center[0]+radius*cos(r*cycle_rad);          //得到圆弧插补轨迹位置坐标
		pr[1] = cycle_center[1]+radius*sin(r*cycle_rad);
		pr[2] = cycle_center[2];
    
		//末端齐次变换矩阵
		Tr << qr.toRotationMatrix(), pr, MatrixXd::Zero(1, 3), 1;    //.toRotationMatrix():旋转向量转旋转矩阵；
        posr.head(6) <<  g_UR->tr_2_MCS(Tr);
		if (rit==rlst.begin())
		{
			pLast=cycle_start_pos;
		}
		g_UR->calc_inverse_kin(g_UR->rpy_2_tr(posr),pLast,posACS);     //逆解得到插补轨迹点的各轴关节角度
        
        //cout<<"posACS："<<posACS(0)<<" | "<<posACS(1)<<" | "<<posACS(2)<<" | "<<posACS(3)<<" | "<<posACS(4)<<" | "<<posACS(5)<<endl;

	    for (int ai=0; ai!=n; ++ai) //1-n轴
	    {
	    	trajectory_joint.push_back(posACS[ai]);       //将轨迹上各轴关节角度序列写入轨迹序列
	    }
		pLast = posACS;	    
	}

	cout<<"开始圆弧"<<endl;
	if(!g_UR->get_power_on_status())
		g_UR->power_on();
	sleep(2);
    //判断是否上电
    if(g_UR->get_power_on_status())
    {
        cout << "机器人已上电！"<<endl;
    }
    else
    {
        cout << "机器人未上电！"<<endl;
    }
    //插补轨迹写入运动队列
	g_UR->set_angle_deque(trajectory_joint);


    cout<<"圆弧结束"<<endl;
}

/*    2023.10.31读相机识别数据    */
vector<double> read_vec;     //保存路径点

void read_txt()
{
    char filename[] = "camera_trajectory.txt";   //读取相机处理数据文件
    int fd = -1;
    int res = 0;                     //读取文件获得的路径点长度
    char read_buf[1000000] = {0};    //创建字符数组存放txt信息
    int vector_size;                 //保存路径点长度

    fd = open(filename, O_RDONLY, 0664);
    if(fd < 0)
    {
        cout<<"file open fail"<<endl;
        return;
    }

    res = read(fd, read_buf,sizeof(read_buf));                        //读取txt信息存入字符数组
    string camera_string(&read_buf[0],&read_buf[res-1]);              //字符数组转换为字符串
    replace(camera_string.begin(),camera_string.end(),'\n',' ');      //字符串中的换行符替换为空格
    memset(read_buf,0,sizeof(read_buf));
    strncpy(read_buf,camera_string.c_str(),camera_string.length());   //重新将字符串转换为字符数组
    if(res < 0)
    {
        cout<<"read fail"<<endl;
        return;
    }
    else
    {
       
        char* temp = strtok(read_buf," ");                  //注意空格和换行
        while(temp != NULL){
            read_vec.push_back(atof(temp));
            temp = strtok(NULL," ");
        }
    }
    vector_size = read_vec.size();
    cout<<"get leght:"<<res<<endl;
    cout<<"read success:"<<vector_size<<" content:"<<read_vec.back()<<endl;
}

/*
void read_txt()
{
    char filename[] = "test.txt";   //读取相机处理数据文件
    int fd = -1;
    int res = 0;        //读取文件获得的路径点长度
    char read_buf[1000000] = {0};

    fd = open(filename, O_RDONLY, 0664);
    if(fd < 0)
    {
        cout<<"file open fail"<<endl;
        return;
    }

    res = read(fd, read_buf,sizeof(read_buf));
    string test(&read_buf[0],&read_buf[res-1]);
    replace(test.begin(),test.end(),'\n',' ');
    memset(read_buf,0,sizeof(read_buf));
    strncpy(read_buf,test.c_str(),test.length());
    if(res < 0)
    {
        cout<<"read fail"<<endl;
        return;
    }
    else
    {
       
        char* temp = strtok(read_buf," ");                  //注意空格和换行
        while(temp != NULL){
            read_vec.push_back(atof(temp));
            temp = strtok(NULL," ");
        }
    }
    vector_size = read_vec.size();
    cout<<"get leght:"<<res<<endl;
    cout<<"read success:"<<vector_size<<" content:"<<read_vec.back()<<endl;
}
*/

/***********对相邻点之间进行插补 ***********/
void follow_point_move_UR()
{

    if (not robot_is_moving)
    {
        // 机器人同时只可执行一次动作
        robot_is_moving = true;
    }
    else
    {
        return;
    }

    double pos_cur_ang[6];   //当前位置角度值,角度制
	VectorXd pos_acs(6);     // 当前位置各关节角度

	for(int i=0;i<6;i++)
	{
		pos_cur_ang[i] = g_UR->get_actual_position(i);  //获取当前位置角度值
	}
	for(int i=0;i<6;i++)
	{
		pos_acs(i) = pos_cur_ang[i];                 //当前位置各关节角度
	}
	MatrixXd trans_matrix_line_test;
	g_UR->calc_forward_kin(pos_acs,trans_matrix_line_test);         ///计算当前位置的正解矩阵
    VectorXd origin_point_test = tr_2_MCS(trans_matrix_line_test);  ///计算当前位置的直角坐标

    double* read_array = read_vec.data();      //创建临时数组存放相机识别得到的数据
    //cout<<"read_array："<<endl;

    //将相机数据转换为 点矩阵数据
    Eigen::MatrixXd points_temp = Map<MatrixXd>(read_array,  6, read_vec.size()/6);     //只找到了列优先的  Map 数组重组模板函数，所以后面转置一下
    MatrixXd points = points_temp.transpose();

    cout<<"points："<<points<<endl;
    cout<<"points.row(read_vec.size()/6)："<<points.row(read_vec.size()/6-1)<<endl;
    read_vec.clear();    


    VectorXd pLast=pos_acs;    //上一时刻关节角度
    VectorXd point_posACS(6);          //位置点对应的各关节角度
    MatrixXd points_pos(points.rows()+1, points.cols());        //位置点对应的各关节角度矩阵,增加第一行机器人初始位置
    points_pos.row(0)=pos_acs;   //初始角度必须为当前实际角度，位置点角度矩阵第一行为初始实际关节角

    //计算各个位置点对应的各轴角度
    for(int j=0;j<=points.rows()-1;j++)
    {
        g_UR->calc_inverse_kin(g_UR->rpy_2_tr(points.row(j)),pLast,point_posACS);
        points_pos.row(j+1)=point_posACS;
        //cout<<"points_pos.row(j+1)"<<points_pos.row(j+1)<<endl;
        pLast=point_posACS;
    }
    cout<<"points_pos："<<points_pos<<endl;

    VectorXd delta_pos(6);    //相邻两点角度变化
    VectorXd axis_step(6);    //各轴插补点数
    int maxStep=0;            //最大插补点数
    float pos_step=0.01;      //角度增加步长--控制运动速度（0.001）
  

    std::deque<double> trajectory_joint;     //总的各轴运动轨迹

    std::deque<double> joint0_trajectory;    //关节0插补轨迹     
    std::deque<double> joint1_trajectory;    //关节1插补轨迹    
    std::deque<double> joint2_trajectory;    //关节2插补轨迹    
    std::deque<double> joint3_trajectory;    //关节3插补轨迹    
    std::deque<double> joint4_trajectory;    //关节4插补轨迹    
    std::deque<double> joint5_trajectory;    //关节5插补轨迹    

    //对相邻两点之间六轴轴轨迹进行插补
    for(int j=0;j<points_pos.rows()-1;j++)
    {
        for(int i=0;i<=5;i++)
        {
            delta_pos(i)=points_pos(j+1,i)-points_pos(j,i);
            axis_step(i)=floor(delta_pos(i)/pos_step);
            
            if(abs(axis_step(i))>maxStep)
            {
                maxStep=abs(axis_step(i));           //相邻两段的最大插补点数
            }

            //cout<<"最大插补点数计算完成"<<maxStep<<endl;
        }

        //插补各轴角度
        for(float t=1;t<maxStep;t++)
        {
            joint0_trajectory.push_back(points_pos(j,0)+t*delta_pos(0)/maxStep);
            joint1_trajectory.push_back(points_pos(j,1)+t*delta_pos(1)/maxStep);
            joint2_trajectory.push_back(points_pos(j,2)+t*delta_pos(2)/maxStep);
            joint3_trajectory.push_back(points_pos(j,3)+t*delta_pos(3)/maxStep);
            joint4_trajectory.push_back(points_pos(j,4)+t*delta_pos(4)/maxStep);
            joint5_trajectory.push_back(points_pos(j,5)+t*delta_pos(5)/maxStep);          
        }      
    }
    cout<<"插补各轴角度完成完成"<<endl;

    auto it0 = joint0_trajectory.begin();
    auto it1 = joint1_trajectory.begin();
    auto it2 = joint2_trajectory.begin();
    auto it3 = joint3_trajectory.begin();
    auto it4 = joint4_trajectory.begin();
    auto it5 = joint5_trajectory.begin();
    //将得到的各轴轨迹交叉插入轨迹序列
    while (it0 != joint0_trajectory.end() && it1 != joint1_trajectory.end() && it2 != joint2_trajectory.end() && it3 != joint3_trajectory.end() && it4 != joint4_trajectory.end() && it5 != joint5_trajectory.end())
    {
        
        trajectory_joint.push_back(*it0);
        trajectory_joint.push_back(*it1);
        trajectory_joint.push_back(*it2);
        trajectory_joint.push_back(*it3);
        trajectory_joint.push_back(*it4);
        trajectory_joint.push_back(*it5);

        ++it0;
        ++it1;
        ++it2;
        ++it3;
        ++it4;
        ++it5;
    }

    cout<<"插补轨迹完成"<<endl;
    if(!g_UR->get_power_on_status())
		g_UR->power_on();
    sleep(2);

    //插补轨迹写入运动队列
    g_UR->set_angle_deque(trajectory_joint); //设置运动轨迹

    double cur_angle_double[6];
    while (g_UR->get_power_on_status() && !g_UR->get_angle_deque().empty()) //循环检测使能状态
    {
		for(int i=0;i<6;i++)
		{
			cur_angle_double[i] = g_UR->get_actual_position(i);  ///获取当前位置角度值
		}
		printf("joint_move_test %lf %lf %lf %lf %lf %lf \n",cur_angle_double[0],cur_angle_double[1],cur_angle_double[2],cur_angle_double[3],cur_angle_double[4],cur_angle_double[5]);
        if (g_UR->get_angle_deque().empty() && g_UR->get_power_on_status())
        {
            g_UR->power_off(); //关闭使能
        }                              //判断运动状态
        sleep(1);
    }

    robot_is_moving = false;
}


//测试笛卡尔空间位置插补，是否能平缓通过多个点
/*   归一化角度（-pi,pi）  */
double normalizeAngle(double angle) {
	while (angle > PI) {
		angle -= 2 * PI;
	}
	while (angle < -PI) {
		angle += 2 * PI;
	}
	return angle;
}


void positionInter()
{
    if (not robot_is_moving)
    {
        // 机器人同时只可执行一次动作
        robot_is_moving = true;
    }
    else
    {
        return;
    }

    double pos_cur_ang[6];   //当前位置角度值,角度制
	VectorXd pos_acs(6);     // 当前位置各关节角度

	for(int i=0;i<6;i++)
	{
		pos_cur_ang[i] = g_UR->get_actual_position(i);  //获取当前位置角度值
	}
	for(int i=0;i<6;i++)
	{
		pos_acs(i) = pos_cur_ang[i];                 //当前位置各关节角度
	}
	MatrixXd trans_matrix_line_test;
	g_UR->calc_forward_kin(pos_acs,trans_matrix_line_test);         ///计算当前位置的正解矩阵
    VectorXd origin_point_test = tr_2_MCS(trans_matrix_line_test);  ///计算当前位置的直角坐标

    double* read_array = read_vec.data();      //创建临时数组存放相机识别得到的数据
    //cout<<"read_array："<<endl;

    //将相机数据转换为 点矩阵数据
    Eigen::MatrixXd points_temp = Map<MatrixXd>(read_array,  6, read_vec.size()/6);     //只找到了列优先的  Map 数组重组模板函数，所以后面转置一下
    MatrixXd points = points_temp.transpose();

    cout<<"points："<<points<<endl;
    cout<<"points.row(read_vec.size()/6)："<<points.row(read_vec.size()/6-1)<<endl;
    read_vec.clear();    

    VectorXd pLast=pos_acs;    //上一时刻关节角度
    VectorXd temp_posACS(6);          //位置点对应的各关节角度

    // 机器人先移动到打磨的第一个点
    g_UR->calc_inverse_kin(g_UR->rpy_2_tr(points.row(0)),pLast,temp_posACS);
    joints_move(temp_posACS); // 使用机器人运动关节差补move_joint_interp方式移动机械臂

    VectorXd delta_position(6);    //相邻两点的变化
    VectorXd tempInter(6);    //相邻两点的变化
    VectorXd axis_step(6);         //各轴插补点数
    int maxStep=0;                 //最大插补点数
    float pStep=0.05;              //位置增加步长--控制运动速度（0.001）
  

    std::deque<double> trajectory_joint;     //总的各轴运动轨迹

    std::deque<double> joint0_trajectory;    //关节0插补轨迹     
    std::deque<double> joint1_trajectory;    //关节1插补轨迹    
    std::deque<double> joint2_trajectory;    //关节2插补轨迹    
    std::deque<double> joint3_trajectory;    //关节3插补轨迹    
    std::deque<double> joint4_trajectory;    //关节4插补轨迹    
    std::deque<double> joint5_trajectory;    //关节5插补轨迹    

    //对相邻两点之间轨迹进行插补
    for(int j=0;j<points.rows()-1;j++)
    {
        //xyz计算
        for(int i=0;i<=5;i++)
        {
            if(i<=2)
            {
                delta_position(i)=points(j+1,i)-points(j,i);
            }
            else
            {
                double endAngle=points(j+1,i);
                while (endAngle - points(j,i) > PI) {
                    endAngle -= 2 * PI;
                }
                while (endAngle - points(j,i) < -PI) {
                    endAngle += 2 * PI;
                }
                delta_position(i)=endAngle-points(j,i);
            }
            
            axis_step(i)=floor(delta_position(i)/pStep);
            
            if(abs(axis_step(i))>maxStep)
            {
                maxStep=abs(axis_step(i));           //相邻两段的最大插补点数
            }
            //cout<<"最大插补点数计算完成"<<maxStep<<endl;
        }

        //插补各轴角度
        for(float t=0;t<maxStep;t++)
        {
            tempInter(0)=points(j,0)+t*delta_position(0)/maxStep;
            tempInter(1)=points(j,1)+t*delta_position(1)/maxStep;
            tempInter(2)=points(j,2)+t*delta_position(2)/maxStep;
            tempInter(3)=normalizeAngle(points(j,3)+t*delta_position(3)/maxStep);
            tempInter(4)=normalizeAngle(points(j,4)+t*delta_position(4)/maxStep);
            tempInter(5)=normalizeAngle(points(j,5)+t*delta_position(5)/maxStep);    

            g_UR->calc_inverse_kin(g_UR->rpy_2_tr(tempInter),pLast,temp_posACS);
            pLast = temp_posACS;  
            for (int ai=0; ai!=6; ++ai) //1-n轴
            {
                trajectory_joint.push_back(temp_posACS[ai]);
            }
        }      
    }
    
    cout<<"插补轨迹完成"<<endl;
    if(!g_UR->get_power_on_status())
		g_UR->power_on();
    sleep(2);

    //插补轨迹写入运动队列
    g_UR->set_angle_deque(trajectory_joint); //设置运动轨迹

    double cur_angle_double[6];
    while (g_UR->get_power_on_status() && !g_UR->get_angle_deque().empty()) //循环检测使能状态
    {
		for(int i=0;i<6;i++)
		{
			cur_angle_double[i] = g_UR->get_actual_position(i);  ///获取当前位置角度值
		}
		printf("joint_move_test %lf %lf %lf %lf %lf %lf \n",cur_angle_double[0],cur_angle_double[1],cur_angle_double[2],cur_angle_double[3],cur_angle_double[4],cur_angle_double[5]);
        if (g_UR->get_angle_deque().empty() && g_UR->get_power_on_status())
        {
            g_UR->power_off(); //关闭使能
        }                              //判断运动状态
        sleep(1);
    }

    robot_is_moving = false;

}

void positionInter1()
{
    if (not robot_is_moving)
    {
        // 机器人同时只可执行一次动作
        robot_is_moving = true;
    }
    else
    {
        return;
    }

    double pos_cur_ang[6];   //当前位置角度值,角度制
	VectorXd pos_acs(6);     // 当前位置各关节角度

	for(int i=0;i<6;i++)
	{
		pos_cur_ang[i] = g_UR->get_actual_position(i);  //获取当前位置角度值
	}
	for(int i=0;i<6;i++)
	{
		pos_acs(i) = pos_cur_ang[i];                 //当前位置各关节角度
	}
	MatrixXd trans_matrix_line_test;
	g_UR->calc_forward_kin(pos_acs,trans_matrix_line_test);         ///计算当前位置的正解矩阵
    VectorXd origin_point_test = tr_2_MCS(trans_matrix_line_test);  ///计算当前位置的直角坐标

    double* read_array = read_vec.data();      //创建临时数组存放相机识别得到的数据
    //cout<<"read_array："<<endl;

    //将相机数据转换为 点矩阵数据
    Eigen::MatrixXd points_temp = Map<MatrixXd>(read_array,  6, read_vec.size()/6);     //只找到了列优先的  Map 数组重组模板函数，所以后面转置一下
    MatrixXd points = points_temp.transpose();

    cout<<"points："<<points<<endl;
    cout<<"points.row(read_vec.size()/6)："<<points.row(read_vec.size()/6-1)<<endl;
    read_vec.clear();    

    VectorXd pLast=pos_acs;    //上一时刻关节角度
    VectorXd temp_posACS(6);          //位置点对应的各关节角度

    // 机器人先移动到打磨的第一个点
    g_UR->calc_inverse_kin(g_UR->rpy_2_tr(points.row(0)),pLast,temp_posACS);
    joints_move(temp_posACS); // 使用机器人运动关节差补move_joint_interp方式移动机械臂

    VectorXd delta_position(6);    //相邻两点的变化
    VectorXd tempInter(6);    //相邻两点的变化
    VectorXd axis_step(6);         //各轴插补点数
    int maxStep=0;                 //最大插补点数
    float pStep=0.01;              //位置增加步长--控制运动速度（0.001）
  

    std::deque<double> trajectory_joint;     //总的各轴运动轨迹

    std::deque<double> joint0_trajectory;    //关节0插补轨迹     
    std::deque<double> joint1_trajectory;    //关节1插补轨迹    
    std::deque<double> joint2_trajectory;    //关节2插补轨迹    
    std::deque<double> joint3_trajectory;    //关节3插补轨迹    
    std::deque<double> joint4_trajectory;    //关节4插补轨迹    
    std::deque<double> joint5_trajectory;    //关节5插补轨迹    


    //对相邻两点之间轨迹进行插补
    for(int j=0;j<points.rows()-1;j++)
    {
        for(int i=0;i<=5;i++)
        {
            delta_position(i)=points(j+1,i)-points(j,i);
            axis_step(i)=floor(delta_position(i)/pStep);
            
            if(abs(axis_step(i))>maxStep)
            {
                maxStep=abs(axis_step(i));           //相邻两段的最大插补点数
            }
            //cout<<"最大插补点数计算完成"<<maxStep<<endl;
        }

        //插补各轴角度
        for(float t=1;t<=maxStep;t++)
        {
            if(tempInter(0)<points(j+1,0))
            {
                tempInter(0)=points(j,0)+t*pStep;
            }
            else
            {
                tempInter(0)=points(j+1,0);
            }

            if(tempInter(1)<points(j+1,1))
            {
                tempInter(1)=points(j,1)+t*pStep;
            }
            else
            {
                tempInter(1)=points(j+1,1);
            }
            
            if(tempInter(2)<points(j+1,2))
            {
                tempInter(2)=points(j,2)+t*pStep;
            }
            else
            {
                tempInter(2)=points(j+1,2);
            }

            if(tempInter(3)<points(j+1,3))
            {
                tempInter(3)=points(j,3)+t*pStep;
            }
            else
            {
                tempInter(3)=points(j+1,3);
            }

            if(tempInter(4)<points(j+1,4))
            {
                tempInter(4)=points(j,4)+t*pStep;
            }
            else
            {
                tempInter(4)=points(j+1,4);
            }

            if(tempInter(5)<points(j+1,5))
            {
                tempInter(5)=points(j,5)+t*pStep;
            }
            else
            {
                tempInter(5)=points(j+1,5);
            }

            g_UR->calc_inverse_kin(g_UR->rpy_2_tr(tempInter),pLast,temp_posACS);
            pLast = temp_posACS;  
            for (int ai=0; ai!=6; ++ai) //1-n轴
            {
                trajectory_joint.push_back(temp_posACS[ai]);
            }
        }      
    }
    
    cout<<"插补轨迹完成"<<endl;
    if(!g_UR->get_power_on_status())
		g_UR->power_on();
    sleep(2);

    //插补轨迹写入运动队列
    g_UR->set_angle_deque(trajectory_joint); //设置运动轨迹

    double cur_angle_double[6];
    while (g_UR->get_power_on_status() && !g_UR->get_angle_deque().empty()) //循环检测使能状态
    {
		for(int i=0;i<6;i++)
		{
			cur_angle_double[i] = g_UR->get_actual_position(i);  ///获取当前位置角度值
		}
		printf("joint_move_test %lf %lf %lf %lf %lf %lf \n",cur_angle_double[0],cur_angle_double[1],cur_angle_double[2],cur_angle_double[3],cur_angle_double[4],cur_angle_double[5]);
        if (g_UR->get_angle_deque().empty() && g_UR->get_power_on_status())
        {
            g_UR->power_off(); //关闭使能
        }                              //判断运动状态
        sleep(1);
    }

    robot_is_moving = false;

}


/***********不插补**********/

void read_txt_no()
{
    char filename[] = "test_no.txt";   //读取相机处理数据文件
    int fd = -1;
    int res = 0;        //读取文件获得的路径点长度
    char read_buf[1000000] = {0};
    int vector_size;            //保存路径点长度

    fd = open(filename, O_RDONLY, 0664);
    if(fd < 0)
    {
        cout<<"file open fail"<<endl;
        return;
    }

    res = read(fd, read_buf,sizeof(read_buf));
    string test(&read_buf[0],&read_buf[res-1]);
    replace(test.begin(),test.end(),'\n',' ');
    memset(read_buf,0,sizeof(read_buf));
    strncpy(read_buf,test.c_str(),test.length());
    if(res < 0)
    {
        cout<<"read fail"<<endl;
        return;
    }
    else
    {
       
        char* temp = strtok(read_buf," ");                  //注意空格和换行
        while(temp != NULL){
            read_vec.push_back(atof(temp));
            temp = strtok(NULL," ");
        }
    }
    vector_size = read_vec.size();
    cout<<"get leght:"<<res<<endl;
    cout<<"read success:"<<vector_size<<" content:"<<read_vec.back()<<endl;
}


void follow_point_move_UR_no()
{

    if (not robot_is_moving)
    {
        // 机器人同时只可执行一次动作
        robot_is_moving = true;
    }
    else
    {
        return;
    }

    double pos_cur_ang[6];   //当前位置角度值,角度制
	VectorXd pos_acs(6);     // 当前位置各关节角度

	for(int i=0;i<6;i++)
	{
		pos_cur_ang[i] = g_UR->get_actual_position(i);  //获取当前位置角度值
	}
	for(int i=0;i<6;i++)
	{
		pos_acs(i) = pos_cur_ang[i];                 //当前位置各关节角度
	}
	MatrixXd trans_matrix_line_test;
	g_UR->calc_forward_kin(pos_acs,trans_matrix_line_test);         ///计算当前位置的正解矩阵
    VectorXd origin_point_test = tr_2_MCS(trans_matrix_line_test);  ///计算当前位置的直角坐标

    //定义一个路径点矩阵points(x,y,z,Z,Y,X)
	//MatrixXd points(5, 6);
    //MatrixXd points((vector_size+1)/6, 6);
    cout<<"start insert"<<endl;
    
    double* read_array = read_vec.data();      //创建临时数组存放相机识别得到的数据
    //cout<<"read_array："<<endl;

    //将相机数据转换为点矩阵数据
    Eigen::MatrixXd points_temp = Map<MatrixXd>(read_array,  6, read_vec.size()/6);     //只找到了列优先的  Map 数组重组模板函数，所以后面转置一下
    MatrixXd points = points_temp.transpose();

    cout<<"points："<<points<<endl;
    cout<<"points.row(read_vec.size()/6)："<<points.row(read_vec.size()/6-1)<<endl;
    read_vec.clear();    

    VectorXd pLast=pos_acs;    //上一时刻关节角度
    VectorXd point_posACS(6);          //位置点对应的各关节角度
    MatrixXd points_pos(points.rows()+1, points.cols());        //位置点对应的各关节角度矩阵,增加第一行机器人初始位置
    points_pos.row(0)=pos_acs;   //初始角度必须为当前实际角度，位置点角度矩阵第一行为初始实际关节角

    //计算各个位置点对应的各轴角度
    for(int j=0;j<=points.rows()-1;j++)
    {
        g_UR->calc_inverse_kin(g_UR->rpy_2_tr(points.row(j)),pLast,point_posACS);
        points_pos.row(j+1)=point_posACS;
        //cout<<"points_pos.row(j+1)"<<points_pos.row(j+1)<<endl;
        pLast=point_posACS;
    }
    cout<<"points_pos："<<points_pos<<endl;


   VectorXd delta_pos(6);    //相邻两点角度变化
    VectorXd axis_step(6);    //各轴插补点数
    int maxStep=0;            //最大插补点数
    float pos_step=0.002;    //角度增加步长--控制运动速度（0.0005）

    std::deque<double> trajectory_joint;     //总的各轴运动轨迹

    std::deque<double> joint0_trajectory;    //关节0插补轨迹     
    std::deque<double> joint1_trajectory;    //关节1插补轨迹    
    std::deque<double> joint2_trajectory;    //关节2插补轨迹    
    std::deque<double> joint3_trajectory;    //关节3插补轨迹    
    std::deque<double> joint4_trajectory;    //关节4插补轨迹    
    std::deque<double> joint5_trajectory;    //关节5插补轨迹    

    //对相邻两点之间六轴轴轨迹进行插补
    for(int j=0;j<1;j++)
    {
        for(int i=0;i<=5;i++)
        {
            delta_pos(i)=points_pos(j+1,i)-points_pos(j,i);
            axis_step(i)=floor(delta_pos(i)/pos_step);
            
            if(abs(axis_step(i))>maxStep)
            {
                maxStep=abs(axis_step(i));           //相邻两段的最大插补点数
            }

            //cout<<"最大插补点数计算完成"<<maxStep<<endl;
        }

        //插补各轴角度
        for(float t=0;t<maxStep;t++)
        {
            joint0_trajectory.push_back(points_pos(j,0)+t*delta_pos(0)/maxStep);
            joint1_trajectory.push_back(points_pos(j,1)+t*delta_pos(1)/maxStep);
            joint2_trajectory.push_back(points_pos(j,2)+t*delta_pos(2)/maxStep);
            joint3_trajectory.push_back(points_pos(j,3)+t*delta_pos(3)/maxStep);
            joint4_trajectory.push_back(points_pos(j,4)+t*delta_pos(4)/maxStep);
            joint5_trajectory.push_back(points_pos(j,5)+t*delta_pos(5)/maxStep);          
        } 
        
    }
    cout<<"插补各轴角度完成完成"<<endl;

    auto it0 = joint0_trajectory.begin();
    auto it1 = joint1_trajectory.begin();
    auto it2 = joint2_trajectory.begin();
    auto it3 = joint3_trajectory.begin();
    auto it4 = joint4_trajectory.begin();
    auto it5 = joint5_trajectory.begin();
    //将得到的各轴轨迹交叉插入轨迹序列
    while (it0 != joint0_trajectory.end() && it1 != joint1_trajectory.end() && it2 != joint2_trajectory.end() && it3 != joint3_trajectory.end() && it4 != joint4_trajectory.end() && it5 != joint5_trajectory.end())
    {
        
        trajectory_joint.push_back(*it0);
        trajectory_joint.push_back(*it1);
        trajectory_joint.push_back(*it2);
        trajectory_joint.push_back(*it3);
        trajectory_joint.push_back(*it4);
        trajectory_joint.push_back(*it5);

        ++it0;
        ++it1;
        ++it2;
        ++it3;
        ++it4;
        ++it5;
    }

    for(int j=1;j<points_pos.rows()-1;j++)
    {
    	 for (int ai=0; ai!=6; ++ai) //1-n轴
	    {
	    	trajectory_joint.push_back(points_pos(j,ai));       //将轨迹上各轴关节角度序列写入轨迹序列
	    }
    }


    cout<<"插补轨迹完成"<<endl;
    sleep(2);

    //插补轨迹写入运动队列
    g_UR->set_angle_deque(trajectory_joint); //设置运动轨迹

    double cur_angle_double[6];
    while (g_UR->get_power_on_status() && !g_UR->get_angle_deque().empty()) //循环检测使能状态
    {
		for(int i=0;i<6;i++)
		{
			cur_angle_double[i] = g_UR->get_actual_position(i);  ///获取当前位置角度值
		}
		printf("joint_move_test %lf %lf %lf %lf %lf %lf \n",cur_angle_double[0],cur_angle_double[1],cur_angle_double[2],cur_angle_double[3],cur_angle_double[4],cur_angle_double[5]);
        if (g_UR->get_angle_deque().empty() && g_UR->get_power_on_status())
        {
            g_UR->power_off(); //关闭使能
        }                              //判断运动状态
        sleep(1);
    }

    robot_is_moving = false;
}


void goto_positions()
{
    if (not robot_is_moving)
    {
        // 机器人同时只可执行一次动作
        robot_is_moving = true;
    }
    else
    {
        // return string("0");
    }

//---------------------------------------------------------------------------------------------------
    // 上电
    if (!g_UR->get_power_on_status())
    {                             //判断使能状态
        g_UR->power_on(); //开启使能
        sleep(2);
    }

 //---------------------------------------------------------------------------------------------------

    double pos_cur_ang[6];   //当前位置角度值,角度制
	VectorXd pos_acs(6);     // 当前位置各关节角度

	for(int i=0;i<6;i++)
	{
		pos_cur_ang[i] = g_UR->get_actual_position(i);  //获取当前位置角度值
	}
	for(int i=0;i<6;i++)
	{
		pos_acs(i) = pos_cur_ang[i];                 //当前位置各关节角度
	}
	MatrixXd trans_matrix_line_test;
	g_UR->calc_forward_kin(pos_acs,trans_matrix_line_test);         ///计算当前位置的正解矩阵
    VectorXd origin_point_test = tr_2_MCS(trans_matrix_line_test);  ///计算当前位置的直角坐标

    double* read_array = read_vec.data();      //创建临时数组存放相机识别得到的数据
    //cout<<"read_array："<<endl;

    //将相机数据转换为 点矩阵数据
    Eigen::MatrixXd points_temp = Map<MatrixXd>(read_array,  6, read_vec.size()/6);     //只找到了列优先的  Map 数组重组模板函数，所以后面转置一下
    MatrixXd points = points_temp.transpose();

    cout<<"points："<<points<<endl;
    cout<<"points.row(read_vec.size()/6)："<<points.row(read_vec.size()/6-1)<<endl;
    read_vec.clear();    


    VectorXd pLast=pos_acs;    //上一时刻关节角度
    VectorXd point_posACS(6);          //位置点对应的各关节角度
    MatrixXd points_pos(points.rows(), points.cols());        //位置点对应的各关节角度矩阵,增加第一行机器人初始位置


    //计算各个位置点对应的各轴角度
    for(int j=0;j<points.rows();j++)
    {
        g_UR->calc_inverse_kin(g_UR->rpy_2_tr(points.row(j)),pLast,point_posACS);
        points_pos.row(j)=point_posACS;
        //cout<<"points_pos.row(j+1)"<<points_pos.row(j+1)<<endl;
        pLast=point_posACS;
    }
    cout<<"points_pos："<<points_pos<<endl;

    // 重置机器人运动标志
    robot_is_moving = false;
    for(int j=0;j<points_pos.rows();j++)
    {

        // 实体机器人移动
        joints_move(points_pos.row(j)); // 使用机器人运动关节差补move_joint_interp方式移动机械臂

        while(1)
        {
            if(g_UR->get_angle_deque().empty())
            {
                cout<<"over"<<endl;
                break;
            }
        }
    }


}


void joint_move_UR()
{
    if (not robot_is_moving)
    {
        // 机器人同时只可执行一次动作
        robot_is_moving = true;
    }
    else
    {
        return;
    }

    char joint_info[255];
    cout << "逆时针旋转角度为正，顺时针旋转角度为负，输入角度格式为：轴;角度" << endl;
    cin >> joint_info;
    char joint_axis_str[2];
    char joint_deg_str[255];
    // 读取关节轴
    joint_axis_str[0] = joint_info[0];
    joint_axis_str[1] = '\0';
    // 读取关节角度
    for (int i = 0; i < 255; i++)
    {
        cout << joint_info << endl;
        if ('\0' == joint_info[i + 2])
        {
            joint_deg_str[i] = '\0';

            break;
        }
        joint_deg_str[i] = joint_info[i + 2];
    }
    int axis = atoi(joint_axis_str);
    double deg = atof(joint_deg_str);
    cout << "逆时针旋转角度为正，顺时针旋转角度为负，输入角度格式为：轴;角度" << endl;
    cout << "轴:" << axis << endl;
    cout << "转角:" << deg << endl;
    cout << "是否确定？yes|no" << endl;
    cin >> joint_info;
    if (0 != strcmp(joint_info, "yes"))
    {
        cout << "取消转动操作，程序退出";
        robot_is_moving = false;
        return;
    }

    VectorXd target_point_joint(6);      //目标位置,角度制
    VectorXd origin_point_joint_test(6); //初始位置,角度制
    VectorXd vel_current_joint_test(6);  //当前速度,角度制
    VectorXd acc_current_joint_test(6);  //当前加速度,角度制
    double pos_cur_ang[6];               //当前位置角度值,角度制

    for (int i = 0; i < 6; i++)
    {
        pos_cur_ang[i] = g_UR->get_actual_position(i); //获取当前位置角度值
        origin_point_joint_test(i) = pos_cur_ang[i];           //当前位置作为起始位置
        cout << "joint" << i << "--" << pos_cur_ang[i] << endl;
    }

    // 机器人标准初始姿态位置(0位置)
    for (int i = 0; i < 6; i++)
    {
        if (axis == i)
        {
            target_point_joint(i) = joint_add(i, pos_cur_ang[i], deg);
        }
        else
        {
            target_point_joint(i) = pos_cur_ang[i];
        }
    }


    cout<<"角度加和："<<target_point_joint<<endl;


    vel_current_joint_test << 0, 0, 0, 0, 0, 0; //设置当前速度
    acc_current_joint_test << 0, 0, 0, 0, 0, 0; //设置当前加速度
    double Ts_joint_test = 0.001;               //设置运动周期
    double velPerc_joint_test = 2;              //设置速度百分比
    double accPerc_joint_test = 2;              //设置加速度百分比
    double decPerc_joint_test = 2;              //设置减速度百分比
    double jerkPerc_joint_test = 2;             //设置雅可比速度百分比
    std::deque<double> trajectory_joint;

	cout << "origin_point_joint_test:" << origin_point_joint_test << endl;

	cout << "target_point_joint:" << target_point_joint <<endl;

    //计算关节插补
    g_UR->move_joint_interp(target_point_joint,
                                    origin_point_joint_test, vel_current_joint_test, acc_current_joint_test, Ts_joint_test, velPerc_joint_test,
                                    accPerc_joint_test, decPerc_joint_test, jerkPerc_joint_test, trajectory_joint);
    
    sleep(1);
    //插补轨迹写入运动队列
    g_UR->set_angle_deque(trajectory_joint); //设置运动轨迹



    while (g_UR->get_angle_deque().empty()) //循环检测使能状态
    {
        // print_velocity(axis);
        usleep(100);
    }

	sleep(1);

    for (int i = 0; i < 6; i++)
    {
        cout << "joint" << i << "--" << g_UR->get_actual_position(i) << endl;
    }
    robot_is_moving = false;
}

void goto_position_test(void)
{

    if (not robot_is_moving)
    {
        // 机器人同时只可执行一次动作
        robot_is_moving = true;
    }
    else
    {
        // return string("0");
    }
    // 在此处添加代码
    /* 在下方添加代码 */
    
    string position;
    cout << "用户输入格式 x|y|z" << endl;
    cin >> position;
    position.append("|3.14|0.0001|0.0001|");

//---------------------------------------------------------------------------------------------------
    // 上电
    if (!g_UR->get_power_on_status())
    {                             //判断使能状态
        g_UR->power_on(); //开启使能
        sleep(2);
    }

 //---------------------------------------------------------------------------------------------------

    VectorXd pos_acs_original(6); // 初始位置各关节角度

    const double direction[] = {1, 1, -1, -1, 1, 1};

    for (int i = 0; i < 6; i++)
    {
        //pos_acs_original(i) = direction[i] * (g_UR->get_actual_position(i) - joint_bias[i]); // 获取当前位置各关节角度值,为相对于零位置移动量
        pos_acs_original(i) = 0.01;
        //pos_acs_original(i) = pos_acs_original(i) * 3.14159 / 180;
    }
    cout << "当前目标位置关节角,移动量" << endl;
    cout << pos_acs_original << endl;

    VectorXd target_point(6); //目标位置x-y-z-r-p-y
    int split_pos = -1;
    int i = 0;
    while (string::npos != (const long unsigned int)(split_pos = position.find("|")) and strcmp("|", position.c_str()) != 0)
    {
        // cout << trajectory << endl;
        double joint_position = atof(position.substr(0, split_pos).c_str());
        target_point(i) = joint_position; // x-y-z-r-p-y
        // cout << joint_position << endl;
        position = position.substr(split_pos + 1);
        i++;
        if (5 == i)
        {
            break;
        }
    }

    target_point(3) = 3.14;
    target_point(4) = 0.001;
    target_point(5) = 0.001;

    cout << target_point(i) << endl;
    

    cout << "目标位置" << endl;
    cout << target_point << endl;                                                                       // 目标位置x-y-z-r-p-y

    //robot_is_moving = false;
    //return;

    VectorXd target_acs(6);                                                                             // 目标位置的各个关节角
    g_UR->calc_inverse_kin(g_UR->rpy_2_tr(target_point), pos_acs_original, target_acs); // 逆解得到各关节角度值，为相对于零位置移动量
    cout << "计算目标位置" << endl;
    cout << "实际得到的目标位置关节角,移动量" << endl;
    cout << target_acs << endl;
    for (int i = 0; i < 6; i++)
    {
        target_acs(i) = joint_bias[i] + target_acs(i) * direction[i]; // 得到各关节角度值，为机械臂实际各关节角度
    }
    cout << "实际得到的目标位置关节角，绝对量" << endl;
    cout << target_acs << endl;

    // 重置机器人运动标志
    robot_is_moving = false;

    // 实体机器人移动
    joints_move(target_acs); // 使用机器人运动关节差补move_joint_interp方式移动机械臂

    //return string("ok");

    
    g_UR->power_off(); //关闭使能


    /* 在上方添加代码 */
}

void goto_position_test_UR(void)
{

    if (not robot_is_moving)
    {
        // 机器人同时只可执行一次动作
        robot_is_moving = true;
    }
    else
    {
        // return string("0");
    }
    // 在此处添加代码
    /* 在下方添加代码 */
    
    // string position;
    // cout << "用户输入格式 x|y|z" << endl;
    // cin >> position;
    // position.append("|1.57|0.0001|0.0001|");

    /*    2023.7.24（加上变换末端姿态）      */
    string position;
    cout << "用户输入格式 x|y|z|r|p|y|" << endl;
    cin >> position;
    /*               结束                  */

//---------------------------------------------------------------------------------------------------
    // 上电
    if (!g_UR->get_power_on_status())
    {                             //判断使能状态
        g_UR->power_on(); //开启使能
        sleep(2);
    }

 //---------------------------------------------------------------------------------------------------

    VectorXd pos_acs_original(6); // 初始位置各关节角度

    const double direction[] = {1, 1, -1, -1, 1, 1};

    for (int i = 0; i < 6; i++)
    {
        //pos_acs_original(i) = direction[i] * (g_UR->get_actual_position(i) - joint_bias[i]); // 获取当前位置各关节角度值,为相对于零位置移动量
        pos_acs_original(i) = 0.01;
        //pos_acs_original(i) = pos_acs_original(i) * 3.14159 / 180;
    }
    cout << "当前目标位置关节角,移动量" << endl;
    cout << pos_acs_original << endl;

    VectorXd target_point(6); //目标位置x-y-z-r-p-y
    int split_pos = -1;
    int i = 0;
    while (string::npos != (const long unsigned int)(split_pos = position.find("|")) and strcmp("|", position.c_str()) != 0)
    {
        // cout << trajectory << endl;
        double joint_position = atof(position.substr(0, split_pos).c_str());
        target_point(i) = joint_position; // x-y-z-r-p-y
        // cout << joint_position << endl;
        position = position.substr(split_pos + 1);
        i++;
        if (6 == i)
        {
            break;
        }
    }

    // target_point(3) = 3.14;
    // target_point(4) = 0.001;
    // target_point(5) = 0.001;

    // cout << target_point(i) << endl;
    

    cout << "目标位置：" << endl;
    cout << target_point << endl;                                           // 目标位置x-y-z-r-p-y

    //robot_is_moving = false;
    //return;

    VectorXd target_acs(6); 
    
                                                                                // 目标位置的各个关节角
    g_UR->calc_inverse_kin(g_UR->rpy_2_tr(target_point), pos_acs_original, target_acs); // 逆解得到各关节角度值，为相对于零位置移动量


    //目标位置矩阵
    cout<<"目标位置矩阵"<<g_UR->rpy_2_tr(target_point)<<endl;


    cout << "计算目标位置" << endl;
    cout << "实际得到的目标位置关节角,移动量" << endl;
    cout << target_acs << endl;
    // for (int i = 0; i < 6; i++)
    // {
    //     target_acs(i) = joint_bias[i] + target_acs(i) * direction[i]; // 得到各关节角度值，为机械臂实际各关节角度
    // }

/*  2023.7.24   */
    for (int i = 0; i < 6; i++)
    {
        target_acs(i) = joint_bias[i] + target_acs(i) * joint_direction[i]; // 乘以joint_direction得到各关节角度值，为机械臂实际各关节角度
    }


    cout << "实际得到的目标位置关节角，绝对量" << endl;
    cout << target_acs << endl;

    char joint_info_test[255];
    cout << "是否确定？yes|no" << endl;
    cin >> joint_info_test;
    if (0 != strcmp(joint_info_test, "yes"))
    {
        cout << "取消转动操作，程序退出";
        robot_is_moving = false;
        return;
    }

    // 实体机器人移动
    joints_move(target_acs); // 使用机器人运动关节差补move_joint_interp方式移动机械臂



    double pos_cur_ang[6];  ///当前位置角度值,角度制
	VectorXd posnow_acs(6);
	for(int i=0;i<6;i++)
	{
		pos_cur_ang[i] = g_UR->get_actual_position(i);  ///获取当前位置角度值
	}
	for(int i=0;i<6;i++)
	{
		posnow_acs(i) = pos_cur_ang[i]* joint_direction[i];
	}
    std::cout<<"当前位置角度值："<<posnow_acs<<endl;
    robot_is_moving = false;

    //return string("ok");
    
    //g_UR->power_off(); //关闭使能
    /* 在上方添加代码 */
}


void joints_move(VectorXd target_point_joint)
{
    //cout << "joints_move" << endl;

    VectorXd origin_point_joint_test(6); //初始位置,角度制
    VectorXd vel_current_joint_test(6);  //当前速度,角度制
    VectorXd acc_current_joint_test(6);  //当前加速度,角度制

    for (int i = 0; i < 6; i++)
    {
        origin_point_joint_test(i) = g_UR->get_actual_position(i); //获取当前位置角度值
    }

    vel_current_joint_test << 0, 0, 0, 0, 0, 0; //设置当前速度
    acc_current_joint_test << 0, 0, 0, 0, 0, 0; //设置当前加速度
    double Ts_joint_test = 0.001;               //设置运动周期
    //double velPerc_joint_test = 5;             //设置速度百分比
    double velPerc_joint_test = 1;

    //double accPerc_joint_test = 5;             //设置加速度百分比
    //double decPerc_joint_test = 10;             //设置减速度百分比
    //double jerkPerc_joint_test = 10;            //设置雅可比速度百分比
    /*  测试速度百分比  */
    double accPerc_joint_test = 1;
    double decPerc_joint_test = 1;
    double jerkPerc_joint_test = 0.5; 

    std::deque<double> trajectory_joint;        // 轨迹

    g_UR->move_joint_interp(target_point_joint,
                                    origin_point_joint_test, vel_current_joint_test, acc_current_joint_test, Ts_joint_test, velPerc_joint_test,
                                    accPerc_joint_test, decPerc_joint_test, jerkPerc_joint_test, trajectory_joint);

    // if (!g_UR->get_power_on_status())
    // {                             //判断使能状态
    //     g_UR->power_on(); //开启使能
    // }
    // sleep(2);

    //插补轨迹写入运动队列
    g_UR->set_angle_deque(trajectory_joint); //设置运动轨迹

    while (g_UR->get_power_on_status()) //循环检测使能状态
    {
        if (g_UR->get_angle_deque().empty() && g_UR->get_power_on_status())
        {
            // g_UR->power_off(); //关闭使能
            //cout << "end :" << endl;
            break;
        } //判断运动状态
        usleep(100);
    }
    cout << "task end " << endl;

}



void app_exit(void)
{
    exit(0);
}

void end_UR(void)
{
    g_UR->power_off();
    cout << "机器人已下电！" << endl;
}

void start_UR(void)
{
    if (!g_UR->get_power_on_status())
    {
        //判断使能状态
        g_UR->power_on(); //开启使能
    }
	sleep(3);

    /*  2023.7.24  */
    //判断是否上电
    if(g_UR->get_power_on_status())
    {
        cout << "机器人已上电！"<<endl;
    }
    else
    {
        cout << "机器人未上电！"<<endl;
    }

}


void joint_zero_UR()
{
    if (not robot_is_moving)
    {
        // 机器人同时只可执行一次动作
        robot_is_moving = true;
    }
    else
    {
        return;
    }


    VectorXd target_point_joint(6);      //目标位置,角度制
    VectorXd origin_point_joint_test(6); //初始位置,角度制
    VectorXd vel_current_joint_test(6);  //当前速度,角度制
    VectorXd acc_current_joint_test(6);  //当前加速度,角度制
    double pos_cur_ang[6];               //当前位置角度值,角度制

    for (int i = 0; i < 6; i++)
    {
        pos_cur_ang[i] = g_UR->get_actual_position(i); //获取当前位置角度值
        origin_point_joint_test(i) = pos_cur_ang[i];           //当前位置作为起始位置
        cout << "joint" << i << "--" << pos_cur_ang[i] << endl;
    }

    // 机器人标准初始姿态位置(0位置)
    // for (int i = 0; i < 6; i++)
    // {
    //     target_point_joint(i) = joint_deg(i, 0);
    //     // cout << joint_deg(i, 0) << endl;
    // }


/*       2023.7.23          */
    //机器人回零目标角度应该是0
    for (int i = 0; i < 6; i++)
    {
        target_point_joint(i) = joint_deg(i, 0);
        cout << joint_deg(i, 0) << endl;
    }

    cout<<"回零目标角度:"<<endl;
    cout<<target_point_joint<<endl;

    vel_current_joint_test << 0, 0, 0, 0, 0, 0; //设置当前速度
    acc_current_joint_test << 0, 0, 0, 0, 0, 0; //设置当前加速度
    double Ts_joint_test = 0.001;               //设置运动周期
    double velPerc_joint_test = 2;              //设置速度百分比
    double accPerc_joint_test = 2;              //设置加速度百分比
    double decPerc_joint_test = 2;              //设置减速度百分比
    double jerkPerc_joint_test = 2;             //设置雅可比速度百分比
    std::deque<double> trajectory_joint;

    //计算关节插补
    g_UR->move_joint_interp(target_point_joint,
                                    origin_point_joint_test, vel_current_joint_test, acc_current_joint_test, Ts_joint_test, velPerc_joint_test,
                                    accPerc_joint_test, decPerc_joint_test, jerkPerc_joint_test, trajectory_joint);

    
    
    //插补轨迹写入运动队列
    g_UR->set_angle_deque(trajectory_joint); //设置运动轨迹

    sleep(2);

    cout << "joint zero end,current joint reg :" << endl;
    cout << "current joint reg :" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << "joint" << i << "--" << get_pos(i) << endl;
    }
    cout << "机器人已回零点！" << endl;
    robot_is_moving = false;
}


/* *  拖动示教  * */
void drag()
{
    cout<<"进入"<<endl;

	if (not robot_is_moving)
    {
        // 机器人同时只可执行一次动作
        robot_is_moving = true;
    }
    // else
    // {
    //     return;
    // }


    // 上电
    if (!g_UR->get_power_on_status())
    {                             //判断使能状态
        g_UR->power_on(); //开启使能
        sleep(2);
    }

	/* 拖动模型 Fe=K*x */
	double K = 100;
	VectorXd zero(6);
	zero << 0, 0, 0, 0, 0, 0;

	vector<VectorXd> F;    //传感器测出的接触力
	vector<VectorXd> dp;

	VectorXd delta = zero;
	VectorXd position(6);
	VectorXd position_0(6);
	position_0 << 473, -141, 469, 1.57, 0, 3.14;

	
	VectorXd posACS(6);
	VectorXd posLast(6);
	posLast << 0, 0, 0, 0, 0, 0;

	MatrixXd positionACS(4,4);

	VectorXd f(6);
	f << 0, 0, 10 , 0, 0, 0;


	for (int i = 0; i < 1000; i++)
	{
		if (i < 50)
		{
			F.push_back(f);
		}
		else
		{
			F.push_back(zero);
		}

		dp.push_back(zero);
	}


    double pos_cur_ang[6];   //当前位置角度值,角度制
	VectorXd pos_acs(6);     // 当前位置各关节角度

	for(int i=0;i<6;i++)
	{
		pos_cur_ang[i] = g_UR->get_actual_position(i);  //获取当前位置角度值
	}
	for(int i=0;i<6;i++)
	{
		pos_acs(i) = pos_cur_ang[i];                 //当前位置各关节角度
	}

    cout<<pos_acs<<endl;

	for (int i = 0; i < 100; i++)
	{
		dp[i] = 1/K *F[i];
		
		for (int j = 3; j <= 5; j++)
		{
			dp[i](j) = 0;

		}

		delta = delta + dp[i];

		position = position_0 + delta;

		cout << position.transpose() << endl;


		g_UR->calc_inverse_kin(g_UR->rpy_2_tr(position), posLast, posACS);
		cout << posACS.transpose() << endl;


        for(int i=0;i<6;i++)
		{
			g_UR->set_target_position(i, posACS(i));
		}

		posLast = posACS;
        g_UR->calc_forward_kin(posACS, positionACS);
		
		//cout << positionACS(0,3) <<"," << positionACS(1,3) << "," << positionACS(2,3)  << endl;
	}

    robot_is_moving = false;

}

void drag1()
{
    cout<<"进入"<<endl;

	if (not robot_is_moving)
    {
        // 机器人同时只可执行一次动作
        robot_is_moving = true;
    }
    // else
    // {
    //     return;
    // }


    // 上电
    if (!g_UR->get_power_on_status())
    {                             //判断使能状态
        g_UR->power_on(); //开启使能
        sleep(2);
    }

    double pos_cur_ang[6];   //当前位置角度值,角度制
	VectorXd pos_acs(6);     // 当前位置各关节角度

	for(int i=0;i<6;i++)
	{
		pos_cur_ang[i] = g_UR->get_actual_position(i);  //获取当前位置角度值
	}
	for(int i=0;i<6;i++)
	{
		pos_acs(i) = pos_cur_ang[i];                 //当前位置各关节角度
	}

    MatrixXd trans_matrix_line_test;
	g_UR->calc_forward_kin(pos_acs,trans_matrix_line_test);         ///计算当前位置的正解矩阵
    VectorXd origin_point_test = g_UR->tr_2_MCS(trans_matrix_line_test);  ///计算当前位置的直角坐标

    cout<<"pos_acs:"<<pos_acs.transpose()<<endl;
    cout<<"trans_matrix_line_test:"<<trans_matrix_line_test<<endl;
    cout<<"origin_point_test:"<<origin_point_test.transpose()<<endl;
    

	/* 拖动模型 F=m*a+b*v */
	int mass = 100;  //质量
	int damper = 50;   //阻尼
	double dt = 0.01;

	VectorXd accDesired = VectorXd::Zero(6);         //加速度
	VectorXd velocityDesired = VectorXd::Zero(6);    //速度  xyzRPY
	VectorXd F = VectorXd::Zero(6);                  //力


	VectorXd position(6);   //实际位置
	position=origin_point_test;
    cout<<position<<endl;
    VectorXd posACS(6);
	VectorXd posLast=pos_acs;

	for (int i = 0; i < 10000; i++)
	{
		if (i < 5000)
		{
			F << 0, 0, 50, 0, 0, 0;
		}
		else
		{
			F << 0, 0, 0, 0, 0, 0;
		}

		if (F[0] == 0 && F[1] == 0 && F[2] == 0 && F[3] == 0 && F[4] == 0 && F[5] == 0)
		{
			velocityDesired << 0, 0, 0, 0, 0, 0;
		}

		accDesired = (F - damper * velocityDesired) / mass;
		velocityDesired = velocityDesired + accDesired * dt;
		
		position = position + velocityDesired * dt;

		//cout << position.transpose() << endl;

		g_UR->calc_inverse_kin(g_UR->rpy_2_tr(position), posLast, posACS);
        posLast = posACS;

		cout << posACS.transpose() << endl;

        for(int i=0;i<6;i++)
		{
			g_UR->set_target_position(i, posACS(i));
		}
        
        sleep(0.5);

	}


    robot_is_moving = false;

}

// 正弦轨迹运动
void sin_move_UR()
{
    if (not robot_is_moving)
    {
        // 机器人同时只可执行一次动作
        robot_is_moving = true;
    }
    else
    {
        return;
    }

    VectorXd target_point_joint(6); //目标位置,角度制
    double pos_cur_ang[6];          //当前位置角度值,角度制

    cout << "task joint5d30"
         << " start ,current joint reg:" << endl;
    for (int i = 0; i < 6; i++)
    {
        pos_cur_ang[i] = g_UR->get_actual_position(i); //获取当前位置角度值
        cout << "joint" << i << "--" << pos_cur_ang[i] << endl;
    }

    //目标位置设置为当前位置增加偏移
    target_point_joint(0) = pos_cur_ang[0];
    target_point_joint(1) = pos_cur_ang[1];
    target_point_joint(2) = pos_cur_ang[2];
    target_point_joint(3) = pos_cur_ang[3];
    target_point_joint(4) = pos_cur_ang[4];
    target_point_joint(5) = pos_cur_ang[5]; //从当前位置开始运动固定角度

    std::deque<double> trajectory_joint;

    // 正弦运动
    double deg4;
    for (float i = 0; i < PI * 2; i += 0.0005)
    {
        deg4 = sin(i - PI / 2) + 1;
        deg4 = deg4 * 10;

        for (int j = 0; j < 6; j++)
        {
            trajectory_joint.push_back(target_point_joint[j] + deg4);
            // if (4 == j)
            // {
            //     trajectory_joint.push_back(target_point_joint[j] + deg4);
            // }
            // else
            // {
            //     trajectory_joint.push_back(target_point_joint[j]);
            // }
        }
    }
    // cout << "return" << endl;
    // return;

    double scale[] = {0, 0, 0, 0, 0, 0};
    double deg[] = {0, 0, 0, 0, 0, 0};

    double a = 0.2 * M_PI;

    /**/
    // //轨迹编号1
    // const double m[] = { 0.4949, - 0.4942 ,   0.0160  ,  0.0824 ,- 0.0991,0.7741 ,   0.0142, - 0.0853, - 0.1182 ,- 0.5848,0.0504 ,   0.0200   , 0.0236 ,- 0.0114 ,- 0.0826,0.0181   , 0.0072 ,- 0.0417 ,- 0.0072,    0.0236 , 0.1214, - 0.0465, - 0.1839 ,   0.0808,    0.0282, -0.7341  ,  0.0610 ,   0.1883   , 0.3910 ,   0.0939 };
    // const double n[] = { 0.0937, - 0.6357, - 0.0469, - 0.0187 ,   0.2786,  0.0103, - 0.0093 ,   0.0884, - 0.1469,    0.0662, 0.4862, - 0.4932, - 0.0410 ,   0.4847 ,- 0.2632 ,0.2477, - 0.0246, - 1.0168   , 0.0843 ,   0.5029 , -0.9010  ,  0.1548 ,- 0.2820 ,   0.1418 ,   0.1740, 0.2601 ,   0.0364 ,- 0.0489 ,   0.2919, - 0.2707 };
    // const double b[] = { -0.3003,    0.0184,    0.4687 ,   0.0289, - 1.3485,   0.4469 };

    const double m[] = { -0.0166 ,   0.1153, - 0.0644, - 0.2775 ,   0.2431 , -0.6497 ,- 0.0893 ,- 0.0523  ,  0.2046 ,   0.5866, -0.2228  ,  0.4621  ,  0.0020, - 0.3369 ,   0.0957, -0.0036  ,  0.0645 ,- 0.0488  ,  0.0256 ,- 0.0377,0.8782 ,   0.2507 ,- 0.1132 ,- 0.4517, - 0.5639 ,-0.7292 ,   0.9469 ,- 0.3604  ,  0.1063  ,  0.0364 };
    const double n[] = { 0.4034 ,- 0.4354  ,  0.0736 ,- 0.3448  ,  0.3252, 0.1145 ,- 0.1774 ,   0.0393  ,  0.1672, - 0.1093 , -0.2786  ,  0.1258 ,- 0.0702,    0.4102, - 0.2806,0.2662 ,   0.3232 ,- 0.5247 ,- 0.4022 ,   0.4540 ,-0.4424 ,- 0.2916 ,- 0.6969   , 0.7716  ,  0.0060 , -0.6889 ,- 0.5524 ,- 0.4265  ,  0.8962 ,- 0.1024 };
    const double b[] = { 0.3009  ,  0.0937 ,- 0.3066 ,   0.3869 ,- 0.9970 ,- 1.4383 };

    // for (float i = 0; i < 30; i += 0.002)

    char step[100];
    cout << "step max 0.0015" << endl;
    cin >> step;
    double d_step = atof(step);
    if (0.0015 <= d_step or -0.0015 >= d_step)
    {
        cout << "step too big" << endl;
        return;
    }

    for (float i = 0; i < 30; i += d_step)
    {
        for (int _ = 0; _ < 6; _++)
        {
            scale[_] = 0.0;
        }
        // 第一轴
        // scale[0] = 1.8 * sin(i) + 0.8 * sin(i);

        for (int k = 0; k < 5; k++)
        {
            scale[0] += m[k] * sin(a * i * (k + 1)) / a / (k + 1);
        }
        for (int k = 0; k < 5; k++)
        {
            scale[0] -= n[k] * cos(a * i * (k + 1)) / a / (k + 1);
        }
        scale[0] += b[0];
        // 第二轴
        // scale[1] = 0.5 * sin(i) + 0.4 * sin(1.5 * i);

        for (int k = 5; k < 10; k++)
        {
            scale[1] += m[k] * sin(a * i * (k - 4)) / a / (k - 4);
        }
        for (int k = 5; k < 10; k++)
        {
            scale[1] -= n[k] * cos(a * i * (k - 4)) / a / (k - 4);
        }
        scale[1] += b[1];
       

        // 第三轴
        // scale[2] = 0.4 * sin(1.5 * i) + 0.2 * sin(i);

        for (int k = 10; k < 15; k++)
        {
            scale[2] += m[k] * sin(a * i * (k - 9)) / a / (k - 9);
        }
        for (int k = 10; k < 15; k++)
        {
            scale[2] -= n[k] * cos(a * i * (k - 9)) / a / (k - 9);
        }
        scale[2] += b[2];

        // 第四轴
        // scale[3] = 0.6 * sin(i) + 0.8 * sin(0.5 * i);

        for (int k = 15; k < 20; k++)
        {
            scale[3] += m[k] * sin(a * i * (k - 14)) / a / (k - 14);
        }
        for (int k = 15; k < 20; k++)
        {
            scale[3] -= n[k] * cos(a * i * (k - 14)) / a / (k - 14);
        }
        scale[3] += b[3];
        // 第五轴
        // scale[4] = 1.9 * sin(0.7 * i);

        for (int k = 20; k < 25; k++)
        {
            scale[4] += m[k] * sin(a * i * (k - 19)) / a / (k - 19);
        }
        for (int k = 20; k < 25; k++)
        {
            scale[4] -= n[k] * cos(a * i * (k - 19)) / a / (k - 19);
        }
        scale[4] += b[4];
       
        // 第六轴

        for (int k = 25; k < 30; k++)
        {
            scale[5] += m[k] * sin(a * i * (k - 24)) / a / (k - 24);
        }
        for (int k = 25; k < 30; k++)
        {
            scale[5] -= n[k] * cos(a * i * (k - 24)) / a / (k - 24);
        }
        scale[5] += b[5];

        for (int j = 0; j < 6; j++)
        {
            // 弧度转角度
            deg[j] = scale[j] * 180 / M_PI;
        }
        // 传入各关节角度序列
        for (int j = 0; j < 6; j++)
        {
            // trajectory_joint.push_back(target_point_joint[j] + deg[j] * joint_direction[j]);
            // trajectory_joint.push_back(joint_add(j, target_point_joint[j], deg[j]));
            trajectory_joint.push_back(joint_deg(j, deg[j]));
        }
    }

    cout << "trajectory_joint push end" << endl;


    sleep(2);

    //插补轨迹写入运动队列
    g_UR->set_angle_deque(trajectory_joint); //设置运动轨迹

    
    cout << "task joint5d30 end ,current joint reg :" << endl;
    for (int i = 0; i < 6; i++)
    {
        cout << "joint" << i << "--" << g_UR->get_actual_position(i) << endl;
    }

    robot_is_moving = false;
}


string get_joints_deg_str(string info)
{
    // cout << info << endl;
    string joints_deg_str = "";

    // 当前时间
    struct timespec st;
    clock_gettime(CLOCK_REALTIME, &st);

    joints_deg_str = joints_deg_str + "|" + to_string(st.tv_sec);
    joints_deg_str = joints_deg_str + "|" + to_string(st.tv_nsec);

    for (int i = 0; i < 6; i++)
    {
        // 获取当前关节角度
        joints_deg_str = joints_deg_str + "|" + to_string(joint_delta(i, g_UR->get_actual_position(i)));
    }
    double joint_torque = 0;
    for (int i = 0; i < 6; i++)
    {
        // 获取当前关节力矩
        // 牛米
        joint_torque = joints_rated_torque[i] * joints_reduce_ratio[i] * g_UR->get_actual_torque(i) / 1000.0 / 1000.0;
        joints_deg_str = joints_deg_str + "|" + to_string(joint_torque);
    }
    // // 速度接口延迟过大,故不采用,使用算法自己作差分
    // int vel_inc = 0;
    // for (unsigned int i = 0; i < 6; i++)
    // {
    //     // 获取当前关节速度
    //     // deg/s
    //     g_general_6s->get_SDO(i, (unsigned int)0x606c, (unsigned int)0x00, (unsigned char *)&vel_inc, sizeof(unsigned int));
    //     joints_deg_str = joints_deg_str + "|" + to_string(2 * PI * vel_inc / joints_reduce_ratio[0] / encoder_resolution);
    // }
    joints_deg_str = joints_deg_str + "|\n";

    return joints_deg_str;
}

void sensor_read(){
    int32_t val = sensor.get_val_sensor();
    cout << "力传感器值为："<< val << endl;
}

void sensor_start(){
    if(sensor.start() == 0)
    {
        cout << "485初始化成功" << endl;
    }
}
void serial_start(){
    sp.Config.BaudRate = SerialPort::BR115200;
    sp.Config.DataBits=SerialPort::DataBits8;
    sp.Config.StopBits=SerialPort::StopBits1;
    sp.Config.Parity=SerialPort::ParityNone;
    sp.Config.DevicePath=(char*)&"/dev/ttyS1";
    if(sp.Open()==false)printf("OPEN error!\n");
	else printf("OPEN OK!\n");
    if(sp.LoadConfig()==false)printf("Set error!\n");
	else printf("Set OK!\n");
}

void serial_read(){
    cout << "数据为:" << sp.Force_Sensor.first << endl;
}

void eRob_status_read() {
    for(int i=0; i < Number; i++) {
        uint16_t status = g_UR->get_status_word(i);
        printf("%d轴状态:%d \n", &i, &status);
    }
}


void joint_cmd_action()
{

    //cout << "开启线程,执行任务tcp 任务" << endl;
    //thread thread_obj(tcp_server);
    char control_str[100];

    map<string, FnPtr> func_map = {
        // 测试
        // {"hello", hello},
        // 程序退出
        {"exit", app_exit},
		{"end", end_UR},
		{"start", start_UR},
		
        // 机械臂归零，使用五次多项式插值
        {"zero", joint_zero_UR},
        // 单关节移动
        {"joint_move", joint_move_UR},
		{"sin_move",sin_move_UR},
 

        {"goto", goto_position_test_UR},
        {"line",line_move_UR},
        {"cycle",cycle_move_UR},
        {"positions",goto_positions},

        {"myLine",cartesian_space_linear_move},
        //测试不插补
        {"read_no",read_txt_no},
        {"points_no",follow_point_move_UR_no},
        //相邻点插补
        {"points",follow_point_move_UR},
        {"read",read_txt},

        //测试位置插补
        {"positionInter",positionInter},
        {"positionInter1",positionInter1},


        //测试拖动示教
        {"drag",drag},
        {"drag1",drag1},

        {"sensor_read",sensor_read},
        {"sensor_start",sensor_start},

        {"read_status",eRob_status_read},

        {"serial_read",serial_read},
        {"serial_start",serial_start}
    };

    while (true)
    {
        cout << "请输入控制字：" << endl;
        cin >> control_str;
        if (func_map.count(control_str) == 0)
        {
            cout << "目标控制字不存在，请输入有效的控制字：" << endl;
            continue;
        }
        cout << "执行任务：" << control_str << endl;
        func_map[control_str]();
    }
}

string joint_tcp_action(string control_str)
{

    int split_pos = control_str.find('&');
    string control_command = control_str.substr(0, split_pos);
    string control_info = control_str.substr(split_pos + 1);
    map<string, StrFnStrPtr> func_map = {
        // 程序退出
        // {"exit", app_exit},
        // // 机械臂归零，使用五次多项式插值
        // {"zero", joint_zero},
        // // 正弦移动测试，以当前位置为基准，慎用
        // {"sin_move", sin_move},
        // 输出当前关节角度信息
        {"current", get_joints_deg_str},
        // {"goto_position", goto_position},
        // {"get_robot_state", get_robot_state},
        // {"close_grasp", close_grasp},
        // {"open_grasp", open_grasp},
        // {"power_on", power_on},
        // {"power_off", power_off},
        // {"joint_move", joint_move},
        // {"zero", joint_zero},
        // {"zero_eye", joint_zero_eye_in_hand},
        // {"e_stop", e_stop},
    };

    if (func_map.count(control_command) == 0)
    {
        cout << "目标控制字不存在" << control_str << endl;
        return string("目标控制字不存在");
    }
    // cout << "执行任务：" << control_str << endl;
    return func_map[control_command](control_info);
}


void test_UR_func()
{
	printf("UR function\n");
	///设置DH参数,若机器人轴数小于6，则不需要设置多余轴号的参数
    /*             2023.7.26  修改DH参数                    */
	DH_param dh_example;
	dh_example.a[0] = 0;
	dh_example.a[1] = 427;
	dh_example.a[2] = 357;
	dh_example.a[3] = 0;
	dh_example.a[4] = 0;
	dh_example.a[5] = 0;
	dh_example.alpha[0] = M_PI*90/180;
	dh_example.alpha[1] = M_PI*0/180;
	dh_example.alpha[2] = M_PI*0/180;
	dh_example.alpha[3] = M_PI*90/180;
	dh_example.alpha[4] = M_PI*(-90)/180;
	dh_example.alpha[5] = M_PI*0/180;
	dh_example.d[0] = 147;
	dh_example.d[1] = 141;
	dh_example.d[2] = 0;
	dh_example.d[3] = 0;
	dh_example.d[4] = 116;
	dh_example.d[5] = 105;
	dh_example.theta[0] = M_PI*0/180;
	dh_example.theta[1] = M_PI*90/180;
	dh_example.theta[2] = M_PI*(-90)/180;
	dh_example.theta[3] = M_PI*90/180;
	dh_example.theta[4] = M_PI*90/180;
	dh_example.theta[5] = M_PI*0/180;
	g_UR->set_DH_param(dh_example);

	///设置笛卡尔参数,若机器人轴数小于6，则不需要设置多余轴号的参数
	Decare_Para decare;
	decare.maxacc = 3;
	decare.maxdec = -3;
	decare.maxjerk = 10000;
	decare.maxvel = 1000;
	g_UR->set_decare_param(decare);




	///设置电机参数,若机器人轴数小于6，则不需要设置多余轴号的参数
	Motor_Param motor_pa;
	motor_pa.encoder.reducRatio[0] = 101.0;
	motor_pa.encoder.reducRatio[1] = 101.0;
	motor_pa.encoder.reducRatio[2] = 101.0;
	motor_pa.encoder.reducRatio[3] = 101.0;
	motor_pa.encoder.reducRatio[4] = 101.0;
	motor_pa.encoder.reducRatio[5] = 101.0;
	motor_pa.encoder.singleTurnEncoder[0] = 50.836486816406;
	motor_pa.encoder.singleTurnEncoder[1] = 102.538833618164;
	motor_pa.encoder.singleTurnEncoder[2] = -9119.446792602539;
	motor_pa.encoder.singleTurnEncoder[3] = 51.523818969727;
	motor_pa.encoder.singleTurnEncoder[4] = -9081.813812255859;
	motor_pa.encoder.singleTurnEncoder[5] = -8.123703002930;
	// motor_pa.encoder.direction[0] = 1;
	// motor_pa.encoder.direction[1] = 1;
	// motor_pa.encoder.direction[2] = -1;
	// motor_pa.encoder.direction[3] = 1;
	// motor_pa.encoder.direction[4] = 1;
	// motor_pa.encoder.direction[5] = 1;


    //2023.7.27测试编码器转角
	motor_pa.encoder.direction[0] = -1;
	motor_pa.encoder.direction[1] = -1;
	motor_pa.encoder.direction[2] = 1;
	motor_pa.encoder.direction[3] = -1;
	motor_pa.encoder.direction[4] = -1;
	motor_pa.encoder.direction[5] = -1;


	for(int i=0;i<6;i++)
	{
		motor_pa.encoder.deviation[i] = 0;
		motor_pa.encoder.encoderResolution[i] = 19;
		motor_pa.RatedVel_rpm[i] = 3000;
		motor_pa.maxAcc[i] = 1;
		motor_pa.maxDecel[i] = -1;
		motor_pa.maxRotSpeed[i] = 2;
		motor_pa.RatedVel[i] = motor_pa.RatedVel_rpm[i] * 6 / motor_pa.encoder.reducRatio[i];
		motor_pa.DeRatedVel[i] = -motor_pa.RatedVel[i];
	}
	g_UR->set_motor_param(motor_pa);

	/*    主要功能块    */
    joint_cmd_action();

}


int create_controller()
{
	///初始化robot指针
	// g_General_6s = new General_6S(); ///通用六轴模型

	g_UR = new UR(); ///UR模型

	// g_UR = new UR(); ///UR模型
	printf("g_UR_ptr %p\n",g_UR);

	g_UR->slave_num.resize(g_UR->axis_sum);
	for(int i = 0;i < g_UR->axis_sum;i++)
		g_UR->slave_num[i] = i+1;  ///设置机器人各轴对应的从站序号, 从站序号从0开始

	return 0;
}

int CALLBACK()
{
return 0;
}


void myprintf(unsigned char c1, const char *s1, const char *s2, const char *s3, const long n, const char *format, ...)
{

    char dest[1024 * 16 * 16];
    va_list argptr;
    va_start(argptr, format);
    vsprintf(dest, format, argptr);
    va_end(argptr);
    printf(dest);
}

int create_pthread() {
    struct sched_param param;
    pthread_attr_t attr;
    int ret;

    /* Initialize pthread attributes (default values) */
    ret = pthread_attr_init(&attr);
    if (ret) {
         printf("init pthread attributes failed\n");
        goto out;
    }

    /* Set a specific stack size  */
     ret = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
    if (ret) {
        printf("pthread setstacksize failed\n");
        goto out;
    }

    /* Set scheduler policy and priority of pthread */
    ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (ret) {
        printf("pthread setschedpolicy failed\n");
        goto out;
    }
    param.sched_priority = 80;
    ret = pthread_attr_setschedparam(&attr, &param);
    if (ret) {
        printf("pthread setschedparam failed\n");
        goto out;
    }
    /* Use scheduling parameters of attr */
    ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (ret) {
        printf("pthread setinheritsched failed\n");
        goto out;
    }

    /* Create a pthread with specified attributes */
    ret = pthread_create(&thread, &attr, ec_thread_func, (void *)cycle_run);
    if (ret) {
        printf("create pthread failed\n");
        goto out;
    }
    return 0;
out:
    return ret;
}

int main()
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
        return -1;
    }
	if(config_ec() == -1) {
        return -1;
    }

	//把pdo地址存入变量中
    for(int i = 0; i < Number; i++) {
        Slave slave_temp(domain1_pd, erob_offset[i]);
        slave_vector.push_back(slave_temp);
    }

    //初始化控制器程序
    create_controller();
    if(create_pthread()) {
        printf("thread create failed! \n");
    }
    sleep(1);
    //功能测试函数
    //pdo测试
	for(int i = 0; i < g_UR->axis_sum;i++)
	{
		//(*slave_vector[g_UR->slave_num[i]].mode_of_operation) = 8;
        EC_WRITE_S8(slave_vector[g_UR->slave_num[i]].mode_of_operation, 8);
	}
	printf("mode test written \n");

	uint8_t mode_of_operation[g_UR->axis_sum];
	for(int i = 0; i < g_UR->axis_sum;i++)
	{
		mode_of_operation[i] = EC_READ_S8(slave_vector[g_UR->slave_num[i]].mode_of_operation_display);
		printf(" operation: %i ", mode_of_operation[i]);
	}
	printf("\n");
	//test_general_6s_func();   //通用六轴模型示例程序
	test_UR_func();             //UR模型示例程序

}



