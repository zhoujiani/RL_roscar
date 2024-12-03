#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// 里程计回调函数
void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    // 在这里处理里程计信息，例如打印出来
    ROS_INFO("Received odom: seq=%d, x=%f, y=%f", odom_msg->header.seq, odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "base_server");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 创建一个publisher，发布到/cmd_vel主题，消息类型为geometry_msgs::Twist，队列大小为10
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 创建一个subscriber，订阅/odom主题，消息类型为nav_msgs::Odometry，队列大小为10，回调函数为odomCallback
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);

    // 设置循环频率为10Hz
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        // 创建一个Twist消息
        geometry_msgs::Twist cmd_vel_msg;

        // 填充Twist消息，这里是示例值，你需要根据实际情况来设置
        cmd_vel_msg.linear.x = 0.1;  // 向前移动的线速度
        cmd_vel_msg.angular.z = 0.0; // 不旋转

        // 发布消息
        cmd_vel_pub.publish(cmd_vel_msg);

        // 处理回调函数
        ros::spinOnce();

        // 按照循环频率延时
        loop_rate.sleep();
    }

    return 0;
}
