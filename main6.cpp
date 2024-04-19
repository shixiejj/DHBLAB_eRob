#include<unistd.h>
#include<sys/socket.h>
#include <stdarg.h>
#include "stdio.h"
#include "server.cpp"



#include <cmath>
#include <iostream>
#include <map>
#include <time.h>
// #include "server.cpp"
#include <thread>

#include "slave/slave.h"
#include "robot_type.h"
#include "tcpsocket.h"
using namespace Eigen;
using namespace std;

UR* g_UR = nullptr;
General_6S* g_General_6s = nullptr;
std::vector<Slave> slave_vector;

typedef void (*FnPtr)(void);
typedef string (*StrFnPtr)(void);
typedef string (*StrFnStrPtr)(string);

string joint_tcp_action(string control_str);

#define PI 3.1415926



extern tcpsocket* g_tcp;
extern void start_ecm(int argc, char* argv[]);
extern void loop_display();


// 机械臂是否在动作
bool robot_is_moving = false;

// 机械臂各个关节额定力矩 
const double joints_rated_torque[] = {1270, 1270, 640, 318, 159, 159};

// 机械臂各个关节减速比
const double joints_reduce_ratio[] = {101, 101, 101, 101, 101, 101};

// 机械臂各个关节初始偏置
const double joint_bias[] = {
    0,
    0,
    -90,
    0,
    90,
    0,
};

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
const int joint_direction[] = {-1, -1, -1, -1, -1, -1};

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




int cycle_run()
{
	g_UR->cycle_run();
	return 0;
}

void joint_move_test()
{
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
	}

	///目标位置设置为当前位置增加偏移
	target_point_joint_test(0) = pos_cur_ang[0] + 10;
	target_point_joint_test(1) = pos_cur_ang[1] + 10;
	target_point_joint_test(2) = pos_cur_ang[2] + 10;
	target_point_joint_test(3) = pos_cur_ang[3] + 10;
	target_point_joint_test(4) = pos_cur_ang[4] - 10;
	target_point_joint_test(5) = pos_cur_ang[5] - 10;  ///从当前位置开始运动固定角度
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
void move_to_zero_pos()
{
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
		target_point_joint_test(i) = 0;  ///目标位置是0度
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

	if(!g_General_6s->get_power_on_status())
		g_General_6s->power_on();
	sleep(2);
	///插补轨迹写入运动队列
	g_General_6s->set_angle_deque(trajectory_joint_test);
	double cur_angle_double[6];
	while(g_General_6s->get_power_on_status())
	{
		for(int i=0;i<6;i++)
		{
			cur_angle_double[i] = g_General_6s->get_actual_position(i);  ///获取当前位置角度值
		}
		printf("move_to_zero_pos %lf %lf %lf %lf %lf %lf \n",cur_angle_double[0],cur_angle_double[1],cur_angle_double[2],cur_angle_double[3],cur_angle_double[4],cur_angle_double[5]);
		if(g_General_6s->get_angle_deque().empty() && g_General_6s->get_power_on_status())
			g_General_6s->power_off();
		sleep(2);
	}
}
void line_move_test()
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
	VectorXd target_point_line_test = origin_point_line_test;
	target_point_line_test[0] = target_point_line_test[0] + 100;  ///设置目标位置

	VectorXd origin_acs_line_test = pos_acs;  ///起始位置的关节坐标

	double velc_line_test = 0;
	double accc_line_test = 0;
	double Ts_line_test = 0.001;
	double maxVel_line_test = 1;
	double maxAcc_line_test = 3;
	double maxDecel_line_test = -3;
	double maxJerk_line_test = 100;///设置直线插补的参数

	std::deque<double> trajectory_line_test;
	///直线插补
	g_General_6s->move_line_interp(target_point_line_test,
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




// void sin_move()
// {
//     if (not robot_is_moving)
//     {
//         // 机器人同时只可执行一次动作
//         robot_is_moving = true;
//     }
//     else
//     {
//         return;
//     }

//     VectorXd target_point_joint(6); //目标位置,角度制
//     double pos_cur_ang[6];          //当前位置角度值,角度制

//     cout << "task joint5d30"
//          << " start ,current joint reg:" << endl;
//     for (int i = 0; i < 6; i++)
//     {
//         pos_cur_ang[i] = g_General_6s->get_actual_position(i); //获取当前位置角度值
//         cout << "joint" << i << "--" << pos_cur_ang[i] << endl;
//     }

//     //目标位置设置为当前位置增加偏移
//     target_point_joint(0) = pos_cur_ang[0];
//     target_point_joint(1) = pos_cur_ang[1];
//     target_point_joint(2) = pos_cur_ang[2];
//     target_point_joint(3) = pos_cur_ang[3];
//     target_point_joint(4) = pos_cur_ang[4];
//     target_point_joint(5) = pos_cur_ang[5]; //从当前位置开始运动固定角度

//     std::deque<double> trajectory_joint;

//     // 正弦运动
//     double deg4;
//     for (float i = 0; i < PI * 2; i += 0.001)
//     {
//         deg4 = sin(i - PI / 2) + 1;
//         deg4 = deg4 * 10;

//         for (int j = 0; j < 6; j++)
//         {
//             // trajectory_joint.push_back(target_point_joint[0] + deg4);
//             if (0 == j)
//             {
//                 trajectory_joint.push_back(target_point_joint[j] + deg4);
//             }
//             else
//             {
//                 trajectory_joint.push_back(target_point_joint[j]);
//             }
//         }
//     }


//     printDeque(trajectory_joint);

//     cout << "trajectory_joint push end" << endl;

//     if (!g_General_6s->get_power_on_status())
//     {                             //判断使能状态
//         g_General_6s->power_on(); //开启使能
//     }

//     sleep(10);

//     //插补轨迹写入运动队列
//     g_General_6s->set_angle_deque(trajectory_joint); //设置运动轨迹

//     while (g_General_6s->get_power_on_status()) //循环检测使能状态
//     {
//         if (g_General_6s->get_angle_deque().empty() && g_General_6s->get_power_on_status())
//         {
//             g_General_6s->power_off(); //关闭使能
//             cout << "joint5d30 end,current joint reg :" << endl;
//             for (int i = 0; i < 6; i++)
//             {
//                 cout << "joint" << i << "--" << g_General_6s->get_actual_position(i) << endl;
//             }
//         } //判断运动状态
//         usleep(100);
//     }
//     cout << "task joint5d30 end ,current joint reg :" << endl;
//     for (int i = 0; i < 6; i++)
//     {
//         cout << "joint" << i << "--" << g_General_6s->get_actual_position(i) << endl;
//     }

//     robot_is_moving = false;
// }


// void sliding_trajectory()
// {
// 	if (not robot_is_moving)
//     {
//         // 机器人同时只可执行一次动作
//         robot_is_moving = true;
//     }
//     else
//     {
//         return;
//     }
// }




// void joint_move()
// {
//     if (not robot_is_moving)
//     {
//         // 机器人同时只可执行一次动作
//         robot_is_moving = true;
//     }
//     else
//     {
//         return;
//     }

//     char joint_info[255];
//     cout << "逆时针旋转角度为正，顺时针旋转角度为负，输入角度格式为：轴;角度" << endl;
//     cin >> joint_info;
//     char joint_axis_str[2];
//     char joint_deg_str[255];
//     // 读取关节轴
//     joint_axis_str[0] = joint_info[0];
//     joint_axis_str[1] = '\0';
//     // 读取关节角度
//     for (int i = 0; i < 255; i++)
//     {
//         cout << joint_info << endl;
//         if ('\0' == joint_info[i + 2])
//         {
//             joint_deg_str[i] = '\0';

//             break;
//         }
//         joint_deg_str[i] = joint_info[i + 2];
//     }
//     int axis = atoi(joint_axis_str);
//     double deg = atof(joint_deg_str);
//     cout << "逆时针旋转角度为正，顺时针旋转角度为负，输入角度格式为：轴;角度" << endl;
//     cout << "轴:" << axis << endl;
//     cout << "转角:" << deg << endl;
//     cout << "是否确定？yes|no" << endl;
//     cin >> joint_info;
//     if (0 != strcmp(joint_info, "yes"))
//     {
//         cout << "取消转动操作，程序退出";
//         robot_is_moving = false;
//         return;
//     }

//     VectorXd target_point_joint(6);      //目标位置,角度制
//     VectorXd origin_point_joint_test(6); //初始位置,角度制
//     VectorXd vel_current_joint_test(6);  //当前速度,角度制
//     VectorXd acc_current_joint_test(6);  //当前加速度,角度制
//     double pos_cur_ang[6];               //当前位置角度值,角度制

//     for (int i = 0; i < 6; i++)
//     {
//         pos_cur_ang[i] = g_General_6s->get_actual_position(i); //获取当前位置角度值
//         origin_point_joint_test(i) = pos_cur_ang[i];           //当前位置作为起始位置
//         cout << "joint" << i << "--" << pos_cur_ang[i] << endl;
//     }

//     // 机器人标准初始姿态位置(0位置)
//     for (int i = 0; i < 6; i++)
//     {
//         if (axis == i)
//         {
//             target_point_joint(i) = joint_add(i, pos_cur_ang[i], deg);
//         }
//         else
//         {
//             target_point_joint(i) = pos_cur_ang[i];
//         }
//     }

//     vel_current_joint_test << 0, 0, 0, 0, 0, 0; //设置当前速度
//     acc_current_joint_test << 0, 0, 0, 0, 0, 0; //设置当前加速度
//     double Ts_joint_test = 0.001;               //设置运动周期
//     double velPerc_joint_test = 2;              //设置速度百分比
//     double accPerc_joint_test = 2;              //设置加速度百分比
//     double decPerc_joint_test = 2;              //设置减速度百分比
//     double jerkPerc_joint_test = 2;             //设置雅可比速度百分比
//     std::deque<double> trajectory_joint;

// 	cout << "origin_point_joint_test:" << origin_point_joint_test << endl;

// 	cout << "target_point_joint:" << target_point_joint <<endl;

//     //计算关节插补
//     g_General_6s->move_joint_interp(target_point_joint,
//                                     origin_point_joint_test, vel_current_joint_test, acc_current_joint_test, Ts_joint_test, velPerc_joint_test,
//                                     accPerc_joint_test, decPerc_joint_test, jerkPerc_joint_test, trajectory_joint);
//     cout << "0000"<<endl;
//     if (!g_General_6s->get_power_on_status())
//     {
//         //判断使能状态
//         g_General_6s->power_on(); //开启使能
//     }
// 	cout << "1111"<<endl;
//     sleep(10);
// 	cout << "2222"<<endl;
//     //插补轨迹写入运动队列
//     g_General_6s->set_angle_deque(trajectory_joint); //设置运动轨迹

// 	printDeque(trajectory_joint);

//     cout << "3333"<<endl;
//     while (g_General_6s->get_power_on_status()) //循环检测使能状态
//     {
//         if (g_General_6s->get_angle_deque().empty() && g_General_6s->get_power_on_status())
//         {
//             g_General_6s->power_off(); //关闭使能
//         }                              //判断运动状态
//         usleep(100);
// 		// cout << "joint" << 0 << "--" << g_UR->get_actual_position(0) << endl;
//     }
//     cout << "4444"<<endl;
//     for (int i = 0; i < 6; i++)
//     {
//         cout << "joint" << i << "--" << g_General_6s->get_actual_position(i) << endl;
//     }
//     robot_is_moving = false;
// }

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
	cout << "机器人已上电！"<<endl;
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
    for (int i = 0; i < 6; i++)
    {
        target_point_joint(i) = joint_deg(i, 0);
        // cout << joint_deg(i, 0) << endl;
    }

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
    //轨迹编号1
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

    for (float i = 0; i < 50; i += d_step)
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







void joint_cmd_action()
{

    cout << "开启线程,执行任务tcp 任务" << endl;
    thread thread_obj(tcp_server);

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

        {"sin_move",sin_move_UR},
        
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
	dh_example.alpha[4] = M_PI*90/180;
	dh_example.alpha[5] = M_PI*0/180;
	dh_example.d[0] = 147;
	dh_example.d[1] = 141;
	dh_example.d[2] = 0;
	dh_example.d[3] = 6;
	dh_example.d[4] = 116;
	dh_example.d[5] = 105;
	dh_example.theta[0] = M_PI*0/180;
	dh_example.theta[1] = M_PI*90/180;
	dh_example.theta[2] = M_PI*0/180;
	dh_example.theta[3] = M_PI*90/180;
	dh_example.theta[4] = M_PI*180/180;
	dh_example.theta[5] = M_PI*180/180;
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
	motor_pa.encoder.direction[0] = 1;
	motor_pa.encoder.direction[1] = 1;
	motor_pa.encoder.direction[2] = -1;
	motor_pa.encoder.direction[3] = 1;
	motor_pa.encoder.direction[4] = 1;
	motor_pa.encoder.direction[5] = 1;
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

// void test_general_6s_func()
// {
// 	printf("general_6s function\n");
// 	///设置DH参数,若机器人轴数小于6，则不需要设置多余轴号的参数
// 	DH_param dh_example;
// 	dh_example.a[0] = 0;
// 	dh_example.a[1] = 295;
// 	dh_example.a[2] = 37;
// 	dh_example.a[3] = 0;
// 	dh_example.a[4] = 0;
// 	dh_example.a[5] = 0;
// 	dh_example.alpha[0] = M_PI*90/180;
// 	dh_example.alpha[1] = M_PI*0/180;
// 	dh_example.alpha[2] = M_PI*90/180;
// 	dh_example.alpha[3] = M_PI*90/180;
// 	dh_example.alpha[4] = M_PI*-90/180;
// 	dh_example.alpha[5] = M_PI*0/180;
// 	dh_example.d[0] = 367.5;
// 	dh_example.d[1] = 0;
// 	dh_example.d[2] = 0;
// 	dh_example.d[3] = 295.5;
// 	dh_example.d[4] = 0;
// 	dh_example.d[5] = 78.5;
// 	dh_example.theta[0] = M_PI*0/180;
// 	dh_example.theta[1] = M_PI*90/180;
// 	dh_example.theta[2] = M_PI*0/180;
// 	dh_example.theta[3] = M_PI*0/180;
// 	dh_example.theta[4] = M_PI*90/180;
// 	dh_example.theta[5] = M_PI*0/180;
// 	g_General_6s->set_DH_param(dh_example);

// 	///设置笛卡尔参数,若机器人轴数小于6，则不需要设置多余轴号的参数
// 	Decare_Para decare;
// 	decare.maxacc = 3;
// 	decare.maxdec = -3;
// 	decare.maxjerk = 10000;
// 	decare.maxvel = 1000;
// 	g_General_6s->set_decare_param(decare);

// 	///设置电机参数,若机器人轴数小于6，则不需要设置多余轴号的参数
// 	Motor_Param motor_pa;
// 	motor_pa.encoder.reducRatio[0] = 81;
// 	motor_pa.encoder.reducRatio[1] = 101;
// 	motor_pa.encoder.reducRatio[2] = 63.462;
// 	motor_pa.encoder.reducRatio[3] = 68.966;
// 	motor_pa.encoder.reducRatio[4] = 81.25;
// 	motor_pa.encoder.reducRatio[5] = 40.625;
// 	motor_pa.encoder.singleTurnEncoder[0] = 1598.604125976562;
// 	motor_pa.encoder.singleTurnEncoder[1] = -1749.262390136719;
// 	motor_pa.encoder.singleTurnEncoder[2] = -5150.456542968750;
// 	motor_pa.encoder.singleTurnEncoder[3] = -114.224853515625;
// 	motor_pa.encoder.singleTurnEncoder[4] = 7665.018310546875;
// 	motor_pa.encoder.singleTurnEncoder[5] = 5702.393188476562;
// 	motor_pa.encoder.direction[0] = 1;
// 	motor_pa.encoder.direction[1] = 1;
// 	motor_pa.encoder.direction[2] = -1;
// 	motor_pa.encoder.direction[3] = -1;
// 	motor_pa.encoder.direction[4] = 1;
// 	motor_pa.encoder.direction[5] = 1;
// 	for(int i=0;i<6;i++)
// 	{
// 		motor_pa.encoder.deviation[i] = 0;
// 		motor_pa.encoder.encoderResolution[i] = 17;
// 		motor_pa.RatedVel_rpm[i] = 3000;
// 		motor_pa.maxAcc[i] = 3;
// 		motor_pa.maxDecel[i] = -3;
// 		motor_pa.maxRotSpeed[i] = 2;
// 		motor_pa.RatedVel[i] = motor_pa.RatedVel_rpm[i] * 6 / motor_pa.encoder.reducRatio[i];
// 		motor_pa.DeRatedVel[i] = -motor_pa.RatedVel[i];
// 	}
// 	g_General_6s->set_motor_param(motor_pa);

// 	/*    主要功能块    */
//     joint_cmd_action();


// 	// ///正解测试
// 	// VectorXd pos_acs(6);

// 	// pos_acs<<0.5,0,0,0,0,0;  ///输入的关节角度,弧度制

// 	// MatrixXd trans_matrix;  ///存储正解矩阵

// 	// g_General_6s->calc_forward_kin(pos_acs,trans_matrix);  ///正解函数
// 	// printf("calcForwardKin transMatrix:\n");  ///正解结果打印在屏幕上
// 	// for(int i=0;i<4;i++)
// 	// {
// 	// 	for(int j=0;j<4;j++)
// 	// 	{
// 	// 		printf("%lf ",trans_matrix(i,j));
// 	// 	}
// 	// 	printf("\n");
// 	// }
// 	// printf("\n");
// 	// ///逆解测试
// 	// VectorXd pos_result(6);   ///存储逆解结果
// 	// g_General_6s->calc_inverse_kin(trans_matrix,pos_acs,pos_result);  ///逆解函数
// 	// printf("calc_inverse_kin axis pos:\n");  ///逆解结果打印在屏幕上
// 	// for(int i=0;i<6;i++)
// 	// 	printf("%lf ",pos_result[i]);
// 	// printf("\n");

//     // for(int i = 0;i < g_General_6s->axis_sum;i++)
//     // {
// 	// 	*(slave_vector[g_General_6s->slave_num[i]].mode_of_operation) = 8;
//     // }
// 	// sleep(1);
  
// 	// if(!g_General_6s->get_power_on_status())  ///判断使能状态
// 	// 	g_General_6s->power_on();  ///开启使能

// 	// printf("power on out\n");

// //	///关节运动测试
// //	joint_move_test();
// //
// //	///回零点
// //	move_to_zero_pos();
// //
// //	///测试直线插补
// //	line_move_test();
// }

// void* readdata(void* args)
// {
// 	g_tcp = new tcpsocket();
// 	printf("readdata\n");
// 	g_tcp->initSocket();
// 	return 0;
// }

// void createPthread()
// {
// 	printf("createPthread\n");
//     // 定义线程的 id 变量，多个变量使用数组
// 	pthread_t tid_read;

// 	//参数依次是：创建的线程id，线程参数，调用的函数，传入的函数参数
// 	int ret = pthread_create(&tid_read, NULL, readdata, NULL);
// 	if (ret != 0)
// 	{
// 	}
//     //等各个线程退出后，进程才结束，否则进程强制结束了，线程可能还没反应过来；
//    // pthread_exit(NULL);
// }

// int start_controller()
// {
// 	///初始化robot指针
// 	g_General_6s = new General_6S(); ///通用六轴模型

// 	// g_UR = new UR(); ///UR模型

// 	// g_UR = new UR(); ///UR模型
// 	printf("g_UR_ptr %p\n",g_General_6s);

// 	g_General_6s->slave_num.resize(g_General_6s->axis_sum);
// 	for(int i = 0;i < g_General_6s->axis_sum;i++)
// 		g_General_6s->slave_num[i] = i+1;  ///设置机器人各轴对应的从站序号, 从站序号从0开始

// 	registerCustomeAppWorkpd(cycle_run);  ///注册循环执行的函数，每个通讯周期调用一次

// 	//pdo测试
// 	for(int i = 0; i < g_General_6s->axis_sum;i++)
// 	{
// 		(*slave_vector[g_General_6s->slave_num[i]].mode_of_operation) = 8;
// 	}
// 		printf("555 \n");

// 	EC_T_WORD mode_of_operation[g_General_6s->axis_sum];
// 	for(int i = 0; i < g_General_6s->axis_sum;i++)
// 	{
// 		mode_of_operation[i] = *(slave_vector[g_General_6s->slave_num[i]].mode_of_operation);
// 		printf("%i ", mode_of_operation[i]);
// 	}
// 	printf("\n");
// 	///功能测试函数
// 	test_general_6s_func(); ///通用六轴模型示例程序
// 	// test_UR_func(); //////UR模型示例程序

// 	printf("test_general_6s_func out \n");

// 	return 0;
// }

int start_controller()
{
	///初始化robot指针
	// g_General_6s = new General_6S(); ///通用六轴模型

	g_UR = new UR(); ///UR模型

	// g_UR = new UR(); ///UR模型
	printf("g_UR_ptr %p\n",g_UR);

	g_UR->slave_num.resize(g_UR->axis_sum);
	for(int i = 0;i < g_UR->axis_sum;i++)
		g_UR->slave_num[i] = i+1;  ///设置机器人各轴对应的从站序号, 从站序号从0开始

	registerCustomeAppWorkpd(cycle_run);  ///注册循环执行的函数，每个通讯周期调用一次

	//pdo测试
	for(int i = 0; i < g_UR->axis_sum;i++)
	{
		(*slave_vector[g_UR->slave_num[i]].mode_of_operation) = 8;
	}
		printf("555 \n");

	EC_T_WORD mode_of_operation[g_UR->axis_sum];
	for(int i = 0; i < g_UR->axis_sum;i++)
	{
		mode_of_operation[i] = *(slave_vector[g_UR->slave_num[i]].mode_of_operation);
		printf("%i ", mode_of_operation[i]);
	}
	printf("\n");
	///功能测试函数
	// test_general_6s_func(); ///通用六轴模型示例程序
	test_UR_func(); //////UR模型示例程序

	printf("test_general_6s_func out \n");

	return 0;
}

int CALLBACK()
{
return 0;
}


void myprintf(unsigned char c1, const char *s1, const char *s2, const char *s3, const long n, const char *format, ...)
{

 char dest[1024 * 16*16];
 va_list argptr;
 va_start(argptr, format);
 vsprintf(dest, format, argptr);
 va_end(argptr);
 printf(dest);
}

int main()
{
	int nArgc = 1;
    char *argv[8];
    int CycleTime = 1000;

    argv[0]=(char *)(&CycleTime);

    enableRealtimeEnvironment();

    EC_PF_EC_START_CustomeLog_CALLBACK p2 = myprintf;
    // registerCustomeAppLog(p2);

    printf("startEcMaster\n");
    startEcMaster(nArgc,argv);

 ///获取从站ID和product code
 std::vector<std::pair<unsigned int, unsigned int> > slave_ID_vector;
 getSlaveIDVec(slave_ID_vector);

 ///获取输入和输出pdo地址
 PDOAddrVec out_PDO_address_vector, in_PDO_address_vector;
 getPDOAddrVec(out_PDO_address_vector, in_PDO_address_vector);


	//把pdo地址存入变量中
 for(int i = 0;i < out_PDO_address_vector.size();i++)
 {
    Slave slave_temp(out_PDO_address_vector[i], in_PDO_address_vector[i]);
    slave_vector.push_back(slave_temp);
 }

 EC_PF_EC_START_AppWorkpd_CALLBACK p = CALLBACK;
 registerCustomeAppWorkpd(p);
//  createPthread();
 ///启动控制器程序
 start_controller();
 ///循环显示
 loop_display();
}

