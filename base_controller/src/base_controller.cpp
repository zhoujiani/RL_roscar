/******************************************************************
基于串口通信的ROS小车基础控制器，功能如下：
1.实现ros控制数据通过固定的格式和串口通信，从而达到控制小车的移动
2.订阅了/cmd_vel主题，只要向该主题发布消息，就能实现对控制小车的移动
3.发布里程计主题/odm
串口通信说明：
1.写入串口:线速度和角速度左右，单位m/s和rad/s
2.读取串口:小车x,y坐标，方向角，线速度，角速度
*******************************************************************/
#include "ros/ros.h" //ros需要的头文件
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
// 以下为串口通讯需要的头文件
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include "serial/serial.h"
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>

#include <iomanip>
#include <bitset>
/****************************************************************************/
using std::cerr;
using std::cout;
using std::endl;
using std::exception;
using std::string;
using std::vector;
/***************************************************************************/

serial::Serial ROS_UART; // 创建串口对象

typedef struct
{
    float cmd_vx;
    float cmd_vy;
    float cmd_womiga;
    float cmd_m1;
    float cmd_m2;
    float cmd_m3;
    float cmd_m4;
    float fix_px;
    float fix_py;
    float fix_ang;
    uint32_t rgb_buz;
    float servo1;
    float servo2;
    float servo3;
    float servo4;
    uint32_t laser_shoot_ms;
    uint32_t state;
} ROS_STM_TYPEDEF;

typedef struct
{
    float odom_px;
    float odom_py;
    float odom_ang;
    float odom_vx;
    float odom_vy;
    float odom_womoga;
    float uwb_px;
    float uwb_py;
    float goal_px;
    float goal_py;
    float goal_ang;
    float act_servo1;
    float act_servo2;
    float act_servo3;
    float act_servo4;
    uint32_t laser_rev_ms;
    uint32_t state;
} STM_ROS_TYPEDEF;

ROS_STM_TYPEDEF ROS_STM_DATA;
STM_ROS_TYPEDEF STM_ROS_DATA;
/****************************************************/
/*
ROS serial(USART) inti
*/
int User_SerialInit(void)
{
    try
    {
        ROS_UART.setPort("/dev/ttyTHS1");
        ROS_UART.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(200);
        ROS_UART.setTimeout(to);
        ROS_UART.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open ROS_UART port ttyTHS1!");
        return -1;
    }

    if (ROS_UART.isOpen())
    {
        ROS_INFO_STREAM("ROS_UART ttyTHS1 serial port opened!");
        return 1;
    }
    else
    {
        return -1;
    }
}
/*
Cmd_vel topic callback
*/
void User_CmdVelCallback(const geometry_msgs::Twist &cmd_input) // 订阅/cmd_vel主题回调函数
{
    ROS_STM_DATA.cmd_vx = cmd_input.linear.x;
    ROS_STM_DATA.cmd_vy = cmd_input.linear.y;
    ROS_STM_DATA.cmd_womiga = cmd_input.angular.z;
}
/*
Joystick message receive
*/
void User_JoystickCallback(const sensor_msgs::Joy::ConstPtr &joy) // 订阅
{
    // 查看joy消息
    /*for (int i = 0; i < joy->axes.size(); ++i)
    {
        ROS_INFO("Axes[%d]: %f", i, joy->axes[i]);
    }
    for (int i = 0; i < joy->buttons.size(); ++i)
    {
        ROS_INFO("Button[%d]: %s", i, joy->buttons[i] ? "pressed" : "released");
    }*/
    ROS_STM_DATA.cmd_vx = joy->axes[3] * 100;
    ROS_STM_DATA.cmd_vy = joy->axes[2] * -100;
    ROS_STM_DATA.cmd_womiga = joy->axes[6] * 100;
    ROS_STM_DATA.servo1 = joy->axes[0] * 100;
    ROS_STM_DATA.servo2 = joy->axes[1] * 100;
    if ((joy->buttons[6] == 1) || (joy->buttons[7] == 1))
        ROS_STM_DATA.laser_shoot_ms = 1;
    else if ((joy->buttons[6] == 0) && (joy->buttons[7] == 0))
        ROS_STM_DATA.laser_shoot_ms = 0;

    ROS_INFO("Joystick vx:%.1f vy:%.1f wo:%.1f servo:%.1f shoot:%d",
             ROS_STM_DATA.cmd_vx, ROS_STM_DATA.cmd_vy, ROS_STM_DATA.cmd_womiga, ROS_STM_DATA.servo1, ROS_STM_DATA.laser_shoot_ms);
}
/*
ROS send data to stm32
*/
void User_RosToStmSend(void)
{
    uint8_t data_buf[78] = {0};
    data_buf[0] = 'R';
    data_buf[1] = 'O';
    data_buf[2] = 'S';
    data_buf[3] = ':';
    data_buf[72] = '>';
    data_buf[73] = 'S';
    data_buf[74] = 'T';
    data_buf[75] = 'M';
    data_buf[76] = '\r';
    data_buf[77] = '\n';

    memcpy(&data_buf[4], &ROS_STM_DATA, 68);
    ROS_UART.write(data_buf, 78);
}
/*
ROS receive data from stm32
*/
uint8_t User_StmToRosParas(void)
{
    string rec_buffer; // 串口数据接收变量

    ROS_UART.flushInput();                           // 先清除串口缓存
    rec_buffer = ROS_UART.readline(100, ">ROS\r\n"); // 阻塞等待读取到100个字节或等于">ROS\r\n"的字符
    // 查看接收数据
    /*for (char c : rec_buffer)
    {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)c;
    }
    std::cout << std::endl;*/

    const char *receive_data = rec_buffer.data(); // 保存串口发送来的数据
    int cnt = rec_buffer.size();
    if (cnt >= 78)
    {
        if ((receive_data[cnt - 78] == 'S') &&
            (receive_data[cnt - 77] == 'T') &&
            (receive_data[cnt - 76] == 'M') &&
            (receive_data[cnt - 75] == ':')) // 串口接收的数据长度正确就处理并发布里程计数据消息
        {
            memcpy(&STM_ROS_DATA, &receive_data[cnt - 74], 68);
            // ROS_INFO("odom_px:%.3f;odom_py:%.3f;odom_ang:%.3f;uwb_px:%.3f;uwb_py:%.3f;laser_ms:%d;state:%d;\r\n",
            //     STM_ROS_DATA.odom_px,STM_ROS_DATA.odom_py,STM_ROS_DATA.odom_ang,STM_ROS_DATA.uwb_px,STM_ROS_DATA.uwb_py,STM_ROS_DATA.laser_rev_ms,STM_ROS_DATA.state);
            return 1;
        }
        else
        {
            ROS_INFO("STM communicate error...");
            for (char c : rec_buffer)
            {
                std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)c;
            }
            std::cout << std::endl;
            return 0;
        }
    }
    else
    {
        // ROS_INFO_STREAM(rec_buffer);
        ROS_INFO("STM communicate lost...");
        for (char c : rec_buffer)
        {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)c;
        }
        std::cout << std::endl;
        return 0;
    }
    ··
}

void User_OdomTopicPublish(const ros::Publisher *publisher)
{
    nav_msgs::Odometry odom;                          // 定义里程计对象
    geometry_msgs::TransformStamped odom_trans;       // 创建一个tf发布需要使用的TransformStamped类型消息
    static tf::TransformBroadcaster odom_broadcaster; // 定义tf对象
    // 定义covariance矩阵，作用为解决文职和速度的不同测量的不确定性
    float covariance[36] = {0.01, 0, 0, 0, 0, 0,  // covariance on gps_x
                            0, 0.01, 0, 0, 0, 0,  // covarianceon gps_y
                            0, 0, 99999, 0, 0, 0, // covarianceon gps_z
                            0, 0, 0, 99999, 0, 0, // large covariance on rot x
                            0, 0, 0, 0, 99999, 0, // large covariance on rot y
                            0, 0, 0, 0, 0, 0.01}; // large covariance on rot z
    for (int i = 0; i < 36; i++)                  // 载入covariance矩阵
    {
        odom.pose.covariance[i] = covariance[i];
    }

    // 载入坐标（tf）变换时间戳
    odom_trans.header.stamp = ros::Time::now();
    // 发布坐标变换的父子坐标系
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    // tf位置数据：x,y,z,方向
    odom_trans.transform.translation.x = STM_ROS_DATA.odom_px;
    odom_trans.transform.translation.y = STM_ROS_DATA.odom_py;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(STM_ROS_DATA.odom_ang); // 将偏航角转换成四元数
    // 发布tf坐标变化
    odom_broadcaster.sendTransform(odom_trans);

    // 载入里程计时间戳
    odom.header.stamp = ros::Time::now();
    // 里程计的父子坐标系
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    // 里程计位置数据：x,y,z,方向
    odom.pose.pose.position.x = STM_ROS_DATA.odom_px;
    odom.pose.pose.position.y = STM_ROS_DATA.odom_py;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(STM_ROS_DATA.odom_ang); // 将偏航角转换成四元数
    // 载入线速度和角速度
    odom.twist.twist.linear.x = STM_ROS_DATA.odom_vx;
    odom.twist.twist.linear.y = STM_ROS_DATA.odom_vy;
    odom.twist.twist.angular.z = STM_ROS_DATA.odom_womoga;
    // 发布里程计
    publisher->publish(odom);
}
/*
main
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "base_controller"); // 初始化串口节点

    ros::NodeHandle n;                                                             // 定义节点进程句柄
    ros::Subscriber sub_cmd_vel = n.subscribe("cmd_vel", 20, User_CmdVelCallback); // 订阅/cmd_vel主题
    ros::Subscriber sub_joystick = n.subscribe("joy", 20, User_JoystickCallback);  // 订阅/joy主题
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 20);         // 定义要发布/odom主题
    User_SerialInit();
    ros::Rate loop_rate(50); // 设置周期休眠时间
    while (ros::ok())
    {
        User_RosToStmSend();

        if (User_StmToRosParas())
        {
            User_OdomTopicPublish(&odom_pub);
                }

        ros::spinOnce(); // callback函数必须处理所有问题时，才可以用到
        loop_rate.sleep();
    }
    ROS_UART.close();
    return 0;
}
