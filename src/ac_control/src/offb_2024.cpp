#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "ac_control/qr_scanner.h"

// 全局变量

// 订阅飞控的状态，用于判断是否解锁以及当前模式，存储到current_state全局变量中
mavros_msgs::State current_state;
void stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

// 订阅本地位姿回调函数，将接收到的位姿存储在全局变量local_pose中
geometry_msgs::PoseStamped local_pose;
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    local_pose = *msg;
}

std_msgs::Bool vision_state;
void VisinoStateCallBack(const std_msgs::Bool::ConstPtr &msg)
{
    vision_state = *msg;
}

ac_control::qr_scanner vision_date;
void VisinoCallBack(const ac_control::qr_scanner::ConstPtr &msg)
{
    vision_date = *msg;
}

void parseTask(const std::string &data, std::string &bigClass, int &subTask)
{
    bigClass.clear();
    std::string num;
    for (char c : data)
    {
        if (std::isalpha(c))
            bigClass += c; // 只保留字母
        else if (std::isdigit(c))
            num += c; // 只保留数字
    }
    subTask = num.empty() ? -1 : std::stoi(num); // 如果没有数字，设置为-1
}

int main(int argc, char **argv)
{

    // 设置中文
    setlocale(LC_ALL, "");

    // 初始化ROS节点
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // 订阅飞控状态和本地位姿
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, poseCallback);
    ros::Subscriber visino_state_sub = nh.subscribe<std_msgs::Bool>("/vision_node/qr_scan/state", 10, VisinoStateCallBack);
    ros::Subscriber vision_sub = nh.subscribe<ac_control::qr_scanner>("/vision_node/qr_scan", 10, VisinoCallBack);

    // 发布到目标位置的话题
    ros::Publisher local_pose_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ros::Publisher fcu_state_pub = nh.advertise<std_msgs::Bool>("/fcu_node/scan_state", 10);
    ros::Publisher fcu_location_pub = nh.advertise<std_msgs::String>("/fcu_node/location", 10);

    // 服务器客户端（设定无人机的模式和状态）
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // 设置循环频率
    ros::Rate rate(20.0);

    // 发布空的目标位置，确保无人机接收到初始位置
    mavros_msgs::PositionTarget pose;
    pose.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.5; // 设置初始高度为2米

    // 等待飞控连接
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("飞控已连接，开始发布目标位置...");

    // 进入 Offboard 模式前，需持续发送设定点
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pose_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // 切换到 Offboard 模式
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // 无人机解锁
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true; // 解锁

    // 记录当前时间
    ros::Time last_request = ros::Time::now();

    // 记录飞机是否到达目标
    std_msgs::Bool fcu_state;
    fcu_state.data = false; // 初始状态为未到达
    fcu_state_pub.publish(fcu_state);

    // 记录飞机位置
    std_msgs::String fcu_location;
    fcu_location.data = "takeoff"; // 初始位置为00
    fcu_location_pub.publish(fcu_location);

    // 要求2
    // 解析任务数据
    std::string bigClass;
    int subTask;
    parseTask(vision_date.message, bigClass, subTask);

    // 用于走圈的变量
    int mode1_step = 0;
    int mode2_step = 0;
    int sametimes = 0;
    int mode = 1;

    while (ros::ok())
    {

        mode = vision_date.mode; // 获取当前模式

        // 飞机状态设定与判定
        // 进入while循环后，先循环5s，然后再向客户端发送无人机状态设置的消息
        // set_mode_client.call   arming_client.call
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
            // ROS_INFO("当前模式: %s, 解锁状态: %s", current_state.mode.c_str(), current_state.armed ? "已解锁" : "未解锁");
            else
            {
                if (mode == 1)
                {
                    switch (mode1_step)
                    {
                    case 0: // 起飞
                        pose.position.x = 0.0;
                        pose.position.y = 0.0;
                        pose.position.z = 0.5; // 起飞高度0.5m

                        // 发布位置
                        fcu_location.data = "takeoff";
                        fcu_location_pub.publish(fcu_location);

                        if (local_pose.pose.position.z > 0.45 && local_pose.pose.position.z < 0.55)
                        {

                            if (sametimes > 10)
                            {
                                mode1_step = 1;
                                sametimes = 0;
                                ROS_INFO("起飞完成，开始下一步");
                            }
                            else
                            {
                                sametimes++;
                            }
                        }
                        else
                        {
                            sametimes = 0; // 如果高度不对，重置计数器
                        }
                        break;
                    case 1: // x轴前进

                        if (local_pose.pose.position.z > 0.45 && local_pose.pose.position.z < 0.55)
                        {
                            // 飞机到达目标点
                            // 扫描、激光
                            // 这里可以添加激光扫描的代码
                            if (sametimes > 10)
                            {
                                mode1_step = 2;
                                sametimes = 0;
                                ROS_INFO("已完成起飞，下一步到达(0,0,1)");
                                pose.position.x = 0;
                                pose.position.y = 0;
                                pose.position.z = 1.0;
                            }
                            else
                            {
                                sametimes++;
                            }
                        }
                        else
                        {
                            sametimes = 0; // 如果高度不对，重置计数器
                        }

                        break;

                    case 2: // y前进
                        if (local_pose.pose.position.z > 0.97 && local_pose.pose.position.z < 1.03)
                        {
                            // 扫描、激光
                            // 这里可以添加激光扫描的代码
                            if (sametimes > 10)
                            {
                                mode1_step = 3;
                                sametimes = 0;
                                ROS_INFO("已到达(0,0,1)，下一步扫描A6到达(0.75,-0.5,1)");
                                pose.position.x = 0.75;
                                pose.position.y = -0.5;
                                pose.position.z = 1.0;
                            }
                            else
                            {
                                sametimes++;
                            }
                        }
                        else
                        {
                            sametimes = 0; // 如果位置不对，重置计数器
                        }
                        break;

                    case 3:
                        if (local_pose.pose.position.y > -0.53 && local_pose.pose.position.y < -0.47 && local_pose.pose.position.x > 0.72 && local_pose.pose.position.x < 0.78)
                        {
                            // 激光

                            // 这里可以添加激光扫描的代码
                            if (sametimes > 10)
                            {
                                // 飞机到达目标点，等待识别
                                fcu_state.data = true;
                                fcu_state_pub.publish(fcu_state);
                                // 发布位置
                                fcu_location.data = "A6";
                                fcu_location_pub.publish(fcu_location);

                                // 等待识别 扫描
                                if (vision_state.data == true)
                                {
                                    ROS_INFO("已到达(0.75,-0.5,1)并扫描A6，下一步扫描A5到达(1.25,-0.5,1)");
                                    mode1_step = 4;
                                    sametimes = 0;

                                    // 飞机开始前往下一个地点
                                    fcu_state.data = false;
                                    fcu_state_pub.publish(fcu_state);

                                    pose.position.x = 1.25;
                                    pose.position.y = -0.5;
                                    pose.position.z = 1.0;
                                }
                                else
                                {
                                    ROS_INFO("未扫描到A6，等待识别");
                                }
                            }
                            else
                            {
                                sametimes++;
                            }
                        }
                        else
                        {
                            sametimes = 0; // 如果位置不对，重置计数器
                        }
                        break;

                    case 4:
                        if (local_pose.pose.position.x < 1.28 && local_pose.pose.position.x > 1.22)
                        {
                            // 扫描、激光
                            // 这里可以添加激光扫描的代码
                            if (sametimes > 10)
                            {
                                // 飞机到达目标点，等待识别
                                fcu_state.data = true;
                                fcu_state_pub.publish(fcu_state);
                                // 发布位置
                                fcu_location.data = "A5";
                                fcu_location_pub.publish(fcu_location);

                                // 等待识别 扫描
                                if (vision_state.data == true)
                                {
                                    mode1_step = 5;
                                    sametimes = 0;

                                    // 飞机开始前往下一个地点
                                    fcu_state.data = false;
                                    fcu_state_pub.publish(fcu_state);

                                    ROS_INFO("已到达(1.25,-0.5,1)并扫描A5，下一步扫描A4到达(1.75,-0.5,1)");
                                    pose.position.x = 1.75;
                                    pose.position.y = -0.5;
                                    pose.position.z = 1.0;
                                }
                                else
                                {
                                    ROS_INFO("未扫描到A5，等待识别");
                                }
                            }
                            else
                            {
                                sametimes++;
                            }
                        }
                        else
                        {
                            sametimes = 0; // 如果位置不对，重置计数器
                        }
                        break;

                    case 5:
                        if (local_pose.pose.position.x < 1.78 && local_pose.pose.position.x > 1.72)
                        {

                            // 扫描、激光
                            // 这里可以添加激光扫描的代码
                            if (sametimes > 10)
                            {
                                // 飞机到达目标点，等待识别
                                fcu_state.data = true;
                                fcu_state_pub.publish(fcu_state);
                                // 发布位置
                                fcu_location.data = "A4";
                                fcu_location_pub.publish(fcu_location);

                                if (vision_state.data == true)
                                {
                                    mode1_step = 6;
                                    sametimes = 0;

                                    // 飞机开始前往下一个地点
                                    fcu_state.data = false;
                                    fcu_state_pub.publish(fcu_state);

                                    ROS_INFO("已到达(1.75,0.5,1)并扫描A4，下一步扫描A1到达(1.75,-0.5,1.4)");
                                    pose.position.x = 1.75;
                                    pose.position.y = -0.5;
                                    pose.position.z = 1.4;
                                }
                                else
                                {
                                    ROS_INFO("未扫描到A4，等待识别");
                                }
                            }
                            else
                            {
                                sametimes++;
                            }
                        }
                        else
                        {
                            sametimes = 0; // 如果位置不对，重置计数器
                        }
                        break;

                    case 6:
                        if (local_pose.pose.position.z < 1.43 && local_pose.pose.position.z > 1.37)
                        {

                            // 扫描、激光
                            // 这里可以添加激光扫描的代码
                            if (sametimes > 10)
                            {
                                // 飞机到达目标点，等待识别
                                fcu_state.data = true;
                                fcu_state_pub.publish(fcu_state);
                                // 发布位置
                                fcu_location.data = "A1";
                                fcu_location_pub.publish(fcu_location);

                                if (vision_state.data == true)
                                {
                                    mode1_step = 7;
                                    sametimes = 0;

                                    // 飞机开始前往下一个地点
                                    fcu_state.data = false;
                                    fcu_state_pub.publish(fcu_state);

                                    ROS_INFO("已到达(1.75,-0.5,1.4)并扫描A1，下一步扫描A2到达(1.25,-0.5,1.4)");
                                    pose.position.x = 1.25;
                                    pose.position.y = -0.5;
                                    pose.position.z = 1.4;
                                }
                                else
                                {
                                    ROS_INFO("未扫描到A1，等待识别");
                                }
                            }
                            else
                            {
                                sametimes++;
                            }
                        }
                        else
                        {
                            sametimes = 0; // 如果位置不对，重置计数器
                        }
                        break;

                    case 7:
                        if (local_pose.pose.position.x < 1.28 && local_pose.pose.position.x > 1.17)
                        {
                            // 扫描、激光
                            // 这里可以添加激光扫描的代码
                            if (sametimes > 10)
                            {
                                // 飞机到达目标点，等待识别
                                fcu_state.data = true;
                                fcu_state_pub.publish(fcu_state);
                                // 发布位置
                                fcu_location.data = "A2";
                                fcu_location_pub.publish(fcu_location);

                                if (vision_state.data == true)
                                {
                                    mode1_step = 8;
                                    sametimes = 0;
                                    // 飞机开始前往下一个地点
                                    fcu_state.data = false;
                                    fcu_state_pub.publish(fcu_state);

                                    ROS_INFO("已到达(1.25,-0.5,1.4)并扫描A2，下一步扫描A3到达(0.75,-0.5,1.4)");
                                    pose.position.x = 0.75;
                                    pose.position.y = -0.5;
                                    pose.position.z = 1.4;
                                }
                                else
                                {
                                    ROS_INFO("未扫描到A2，等待识别");
                                }
                            }
                            else
                            {
                                sametimes++;
                            }
                        }
                        else
                        {
                            sametimes = 0; // 如果位置不对，重置计数器
                        }
                        break;

                    case 8:
                        if (local_pose.pose.position.x < 0.78 && local_pose.pose.position.x > 0.72)
                        {

                            // 扫描、激光
                            // 这里可以添加激光扫描的代码
                            if (sametimes > 10)
                            {
                                // 飞机到达目标点，等待识别
                                fcu_state.data = true;
                                fcu_state_pub.publish(fcu_state);
                                // 发布位置
                                fcu_location.data = "A3";
                                fcu_location_pub.publish(fcu_location);

                                if (vision_state.data == true)
                                {
                                    mode1_step = 9;
                                    sametimes = 0;
                                    // 飞机开始前往下一个地点
                                    fcu_state.data = false;
                                    fcu_state_pub.publish(fcu_state);

                                    ROS_INFO("已到达(0.75,-0.5,1.4)并扫描A3，下一步转移到B面，到达(0,-0.5,1.4)");
                                    pose.position.x = 0.0;
                                    pose.position.y = -0.5;
                                    pose.position.z = 1.4;
                                }
                                else
                                {
                                    ROS_INFO("未扫描到A3，等待识别");
                                }
                            }
                            else
                            {
                                sametimes++;
                            }
                        }
                        else
                        {
                            sametimes = 0; // 如果位置不对，重置计数器
                        }
                        break;

                    case 9:
                        if (local_pose.pose.position.x < 0.03 && local_pose.pose.position.x > -0.03)
                        {
                            // 扫描、激光
                            // 这里可以添加激光扫描的代码
                            if (sametimes > 10)
                            {
                                mode1_step = 10;
                                sametimes = 0;

                                // 飞机开始前往下一个地点
                                fcu_state.data = false;
                                fcu_state_pub.publish(fcu_state);

                                ROS_INFO("已到达(0,-0.5,1.4)，下一步转移到B面,到达(0,-1.0,1.4)");
                                pose.position.x = 0.0;
                                pose.position.y = -1.0;
                                pose.position.z = 1.4;
                            }
                            else
                            {
                                sametimes++;
                            }
                        }
                        else
                        {
                            sametimes = 0; // 如果位置不对，重置计数器
                        }
                        break;

                    case 10:
                        if (local_pose.pose.position.y < -0.97 && local_pose.pose.position.y > -1.03)
                        {
                            // 扫描、激光
                            // 这里可以添加激光扫描的代码
                            if (sametimes > 10)
                            {
                                mode1_step = 11;
                                sametimes = 0;
                                ROS_INFO("已到达(0,-1.0,1.4)，下一步扫描B1到达(0.75,-1.0,1.4)");
                                pose.position.x = 0.75;
                                pose.position.y = -1.0;
                                pose.position.z = 1.4;
                            }
                            else
                            {
                                sametimes++;
                            }
                        }
                        else
                        {
                            sametimes = 0; // 如果位置不对，重置计数器
                        }
                        break;

                    case 11:
                        if (local_pose.pose.position.x < 0.78 && local_pose.pose.position.x > 0.72)
                        {

                            // 扫描、激光
                            // 这里可以添加激光扫描的代码
                            if (sametimes > 10)
                            {
                                // 飞机到达目标点，等待识别
                                fcu_state.data = true;
                                fcu_state_pub.publish(fcu_state);
                                // 发布位置
                                fcu_location.data = "B1";
                                fcu_location_pub.publish(fcu_location);

                                if (vision_state.data == true)
                                {
                                    mode1_step = 12;
                                    sametimes = 0;

                                    // 飞机开始前往下一个地点
                                    fcu_state.data = false;
                                    fcu_state_pub.publish(fcu_state);

                                    ROS_INFO("已到达(0.75,-1.0,1.4)并扫描B1，下一步扫描B2到达(1.25,-1.0,1.4)");
                                    pose.position.x = 1.25;
                                    pose.position.y = -1.0;
                                    pose.position.z = 1.4;
                                }
                                else
                                {
                                    ROS_INFO("未扫描到B1，等待识别");
                                }
                            }
                            else
                            {
                                sametimes++;
                            }
                        }
                        else
                        {
                            sametimes = 0; // 如果位置不对，重置计数器
                        }
                        break;

                    case 12:
                        if (local_pose.pose.position.x < 1.28 && local_pose.pose.position.x > 1.17)
                        {

                            // 扫描、激光
                            // 这里可以添加激光扫描的代码
                            if (sametimes > 10)
                            {
                                // 飞机到达目标点，等待识别
                                fcu_state.data = true;
                                fcu_state_pub.publish(fcu_state);
                                // 发布位置
                                fcu_location.data = "B2";
                                fcu_location_pub.publish(fcu_location);

                                if (vision_state.data == true)
                                {
                                    mode1_step = 13;
                                    sametimes = 0;

                                    // 飞机开始前往下一个地点
                                    fcu_state.data = false;
                                    fcu_state_pub.publish(fcu_state);

                                    ROS_INFO("已到达(1.25,-1.0,1.4)并扫描B2，下一步扫描B3到达(1.75,-1.0,1.4)");
                                    pose.position.x = 1.75;
                                    pose.position.y = -1.0;
                                    pose.position.z = 1.4;
                                }
                                else
                                {
                                    ROS_INFO("未扫描到B2，等待识别");
                                }
                            }
                            else
                            {
                                sametimes++;
                            }
                        }
                        else
                        {
                            sametimes = 0; // 如果位置不对，重置计数器
                        }
                        break;

                    case 13:
                        if (local_pose.pose.position.x < 1.78 && local_pose.pose.position.x > 1.72)
                        {

                            // 扫描、激光
                            // 这里可以添加激光扫描的代码
                            if (sametimes > 10)
                            {
                                // 飞机到达目标点，等待识别
                                fcu_state.data = true;
                                fcu_state_pub.publish(fcu_state);
                                // 发布位置
                                fcu_location.data = "B3";
                                fcu_location_pub.publish(fcu_location);

                                if (vision_state.data == true)
                                {
                                    mode1_step = 14;
                                    sametimes = 0;

                                    // 飞机开始前往下一个地点
                                    fcu_state.data = false;
                                    fcu_state_pub.publish(fcu_state);

                                    ROS_INFO("已到达(1.75,-1.0,1.4)并扫描B3，下一步扫描B6到达(1.75,-1.0,1)");
                                    pose.position.x = 1.75;
                                    pose.position.y = -1.0;
                                    pose.position.z = 1.0;
                                }
                                else
                                {
                                    ROS_INFO("未扫描到B3，等待识别");
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果位置不对，重置计数器
                            }
                            break;

                        case 14:
                            if (local_pose.pose.position.z < 1.03 && local_pose.pose.position.z > 0.97)
                            {

                                // 扫描、激光
                                // 这里可以添加激光扫描的代码
                                if (sametimes > 10)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "B6";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        // 飞机开始前往下一个地点
                                        mode1_step = 15;
                                        sametimes = 0;

                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(1.75,-1.0,1)并扫描B6，下一步扫描B5到达(1.25,-1.0,1)");
                                        pose.position.x = 1.25;
                                        pose.position.y = -1.0;
                                        pose.position.z = 1.0;
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到B6，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果位置不对，重置计数器
                            }
                            break;

                        case 15:
                            if (local_pose.pose.position.x < 1.28 && local_pose.pose.position.x > 1.22)
                            {
                                // 扫描、激光
                                // 这里可以添加激光扫描的代码
                                if (sametimes > 10)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "B5";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        mode1_step = 16;
                                        sametimes = 0;

                                        // 飞机开始前往下一个地点
                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(1.25,-1.0,1)并扫描B5，下一步扫描B4到达(0.75,-1.0,1)");
                                        pose.position.x = 0.75;
                                        pose.position.y = -1.0;
                                        pose.position.z = 1.0;
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到B5，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果位置不对，重置计数器
                            }
                            break;

                        case 16:
                            if (local_pose.pose.position.x < 0.78 && local_pose.pose.position.x > 0.72)
                            {

                                // 扫描、激光
                                // 这里可以添加激光扫描的代码
                                if (sametimes > 10)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "B4";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        mode1_step = 17;
                                        sametimes = 0;

                                        // 飞机开始前往下一个地点
                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(0.75,-1.0,1)并扫描B4，下一步扫描C6到达(0.75,-2.5,1)");
                                        pose.position.x = 0.25;
                                        pose.position.y = -2.5;
                                        pose.position.z = 1.0;
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到B4，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果位置不对，重置计数器
                            }
                            break;

                        case 17:
                            if (local_pose.pose.position.y < -2.47 && local_pose.pose.position.y > -2.53)
                            {

                                // 扫描、激光
                                // 这里可以添加激光扫描的代码
                                if (sametimes > 10)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "C6";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        // 飞机开始前往下一个地点
                                        mode1_step = 18;
                                        sametimes = 0;

                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(0.75,-2.5,1)并扫描C6，下一步扫描C5到达(1.25,-2.5,1)");
                                        pose.position.x = 1.25;
                                        pose.position.y = -2.5;
                                        pose.position.z = 1.0;
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到C6，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果位置不对，重置计数器
                            }
                            break;

                        case 18:
                            if (local_pose.pose.position.x < 1.28 && local_pose.pose.position.x > 1.22)
                            {

                                // 扫描、激光
                                // 这里可以添加激光扫描的代码
                                if (sametimes > 10)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "C5";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        // 飞机开始前往下一个地点
                                        mode1_step = 19;
                                        sametimes = 0;

                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(1.25,-2.5,1)并扫描C5，下一步扫描C4到达(1.75,-2.5,1)");
                                        pose.position.x = 1.75;
                                        pose.position.y = -2.5;
                                        pose.position.z = 1.0;
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到C5，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果位置不对，重置计数器
                            }
                            break;

                        case 19:
                            if (local_pose.pose.position.x < 1.78 && local_pose.pose.position.x > 1.72)
                            {

                                // 扫描、激光
                                // 这里可以添加激光扫描的代码
                                if (sametimes > 10)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "C4";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        mode1_step = 20;
                                        sametimes = 0;

                                        // 飞机开始前往下一个地点
                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(1.75,-2.5,1)并扫描C4，下一步扫描C1到达(1.75,-2.5,1.4)");
                                        pose.position.x = 1.75;
                                        pose.position.y = -2.5;
                                        pose.position.z = 1.4;
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到C4，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果位置不对，重置计数器
                            }
                            break;

                        case 20:
                            if (local_pose.pose.position.z < 1.43 && local_pose.pose.position.z > 1.37)
                            {

                                // 扫描、激光
                                // 这里可以添加激光扫描的代码
                                if (sametimes > 10)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "C1";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        mode1_step = 21;
                                        sametimes = 0;

                                        // 飞机开始前往下一个地点
                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(1.75,-2.5,1.4)并扫描C1，下一步扫描C2到达(1.25,-2.5,1.4)");
                                        pose.position.x = 1.25;
                                        pose.position.y = -2.5;
                                        pose.position.z = 1.4;
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到C1，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果位置不对，重置计数器
                            }
                            break;

                        case 21:
                            if (local_pose.pose.position.x < 1.28 && local_pose.pose.position.x > 1.17)
                            {

                                // 扫描、激光
                                // 这里可以添加激光扫描的代码
                                if (sametimes > 10)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "C2";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        // 飞机开始前往下一个地点
                                        mode1_step = 22;
                                        sametimes = 0;

                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(1.25,-2.5,1.4)并扫描C2，下一步扫描C3到达(0.75,-2.5,1.4)");
                                        pose.position.x = 0.75;
                                        pose.position.y = -2.5;
                                        pose.position.z = 1.4;
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到C2，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果位置不对，重置计数器
                            }
                            break;

                        case 22:
                            if (local_pose.pose.position.x < 0.78 && local_pose.pose.position.x > 0.72)
                            {

                                // 扫描、激光
                                // 这里可以添加激光扫描的代码
                                if (sametimes > 10)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "C3";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        mode1_step = 23;
                                        sametimes = 0;

                                        // 飞机开始前往下一个地点
                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(0.75,-2.5,1.4)并扫描C3，下一步转移到D面，到达(0,-2.5,1.4)");
                                        pose.position.x = 0.0;
                                        pose.position.y = -2.5;
                                        pose.position.z = 1.4;
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到C3，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果位置不对，重置计数器
                            }
                            break;

                        case 23:
                            if (local_pose.pose.position.x < 0.03 && local_pose.pose.position.x > -0.03)
                            {
                                // 扫描、激光
                                // 这里可以添加激光扫描的代码
                                if (sametimes > 10)
                                {
                                    mode1_step = 24;
                                    sametimes = 0;
                                    ROS_INFO("已到达(0,-2.5,1.4)，下一步转移到D面,到达(0,-3.0,1.4)");
                                    pose.position.x = 0.0;
                                    pose.position.y = -3.0;
                                    pose.position.z = 1.4;
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果位置不对，重置计数器
                            }
                            break;

                        case 24:
                            if (local_pose.pose.position.y < -2.97 && local_pose.pose.position.y > -3.03)
                            {
                                // 扫描、激光
                                // 这里可以添加激光扫描的代码
                                if (sametimes > 10)
                                {
                                    mode1_step = 25;
                                    sametimes = 0;
                                    ROS_INFO("已到达(0,-3.0,1.4)，下一步扫描D1到达(0.75,-3.0,1.4)");
                                    pose.position.x = 0.75;
                                    pose.position.y = -3.0;
                                    pose.position.z = 1.4;
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果位置不对，重置计数器
                            }
                            break;

                        case 25:
                            if (local_pose.pose.position.x < 0.78 && local_pose.pose.position.x > 0.72)
                            {

                                // 扫描、激光
                                // 这里可以添加激光扫描的代码
                                if (sametimes > 10)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "D1";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        mode1_step = 26;
                                        sametimes = 0;

                                        // 飞机开始前往下一个地点
                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(0.75,-3.0,1.4)并扫描D1，下一步扫描D2到达(1.25,-3.0,1.4)");
                                        pose.position.x = 1.25;
                                        pose.position.y = -3.0;
                                        pose.position.z = 1.4;
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到D1，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果位置不对，重置计数器
                            }
                            break;
                        case 26:
                            if (local_pose.pose.position.x < 1.28 && local_pose.pose.position.x > 1.17)
                            {

                                // 扫描、激光
                                // 这里可以添加激光扫描的代码
                                if (sametimes > 10)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "D2";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        // 飞机开始前往下一个地点
                                        mode1_step = 27;
                                        sametimes = 0;

                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(1.25,-3.0,1.4)并扫描D2，下一步扫描D3到达(1.75,-3.0,1.4)");
                                        pose.position.x = 1.75;
                                        pose.position.y = -3.0;
                                        pose.position.z = 1.4;
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到D2，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果位置不对，重置计数器
                            }
                            break;

                        case 27:
                            if (local_pose.pose.position.x < 1.78 && local_pose.pose.position.x > 1.72)
                            {

                                // 扫描、激光
                                // 这里可以添加激光扫描的代码
                                if (sametimes > 10)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "D3";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        mode1_step = 28;
                                        sametimes = 0;

                                        // 飞机开始前往下一个地点
                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(1.75,-3.0,1.4)并扫描D3，下一步扫描D6到达(1.75,-3.0,1)");
                                        pose.position.x = 1.75;
                                        pose.position.y = -3.0;
                                        pose.position.z = 1.0;
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到D3，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果位置不对，重置计数器
                            }
                            break;

                        case 28:
                            if (local_pose.pose.position.z < 1.03 && local_pose.pose.position.z > 0.97)
                            {

                                // 扫描、激光
                                // 这里可以添加激光扫描的代码
                                if (sametimes > 10)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "D6";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        // 飞机开始前往下一个地点
                                        mode1_step = 29;
                                        sametimes = 0;

                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(1.75,-3.0,1)并扫描D6，下一步扫描D5到达(1.25,-3.0,1)");
                                        pose.position.x = 1.25;
                                        pose.position.y = -3.0;
                                        pose.position.z = 1.0;
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到D6，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果位置不对，重置计数器
                            }
                            break;

                        case 29:
                            if (local_pose.pose.position.x < 1.28 && local_pose.pose.position.x > 1.22)
                            {

                                // 扫描、激光
                                // 这里可以添加激光扫描的代码
                                if (sametimes > 10)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "D5";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        // 飞机开始前往下一个地点
                                        mode1_step = 30;
                                        sametimes = 0;

                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(1.25,-3.0,1)并扫描D5，下一步扫描D4到达(0.75,-3.0,1)");
                                        pose.position.x = 0.75;
                                        pose.position.y = -3.0;
                                        pose.position.z = 1.0;
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到D5，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果位置不对，重置计数器
                            }
                            break;

                        case 30:
                            if (local_pose.pose.position.x < 0.78 && local_pose.pose.position.x > 0.72)
                            {

                                // 扫描、激光
                                // 这里可以添加激光扫描的代码
                                if (sametimes > 10)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "D4";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        mode1_step = 31;
                                        sametimes = 0;

                                        // 飞机开始前往下一个地点
                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(0.75,-3.0,1)并扫描D4，下一步前往降落点(2.5,-3.5,1)");
                                        pose.position.x = 2.5;
                                        pose.position.y = -3.5;
                                        pose.position.z = 1;
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到D4，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果位置不对，重置计数器
                            }
                            break;

                        case 31:
                            if (local_pose.pose.position.x < 2.53 && local_pose.pose.position.x > 2.47 && local_pose.pose.position.y < -3.2 && local_pose.pose.position.y > -3.8)
                            {
                                if (sametimes > 10)
                                {
                                    // 发布位置
                                    fcu_location.data = "landing";
                                    fcu_location_pub.publish(fcu_location);

                                    sametimes = 0;
                                    ROS_INFO("已到达降落点(2.5,-3.5,1)，下一步降落");
                                    offb_set_mode.request.custom_mode = "AUTO.LAND"; // 切换到自动降落模式
                                    if (current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
                                    {
                                        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                                        {
                                            ROS_INFO("成功切换到 AUTO.LAND 模式");
                                        }
                                        last_request = ros::Time::now();
                                    }
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果位置不对，重置计数器
                            }
                            break;
                            // case 5: // 降落上锁
                            //     offb_set_mode.request.custom_mode = "AUTO.LAND"; // 切换到自动降落模式
                            //     if(current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
                            //     {
                            //         if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                            //         {
                            //             ROS_INFO("成功切换到 AUTO.LAND 模式");
                            //         }
                            //         last_request = ros::Time::now();
                            //     }
                            //     break;

                        default:
                            ROS_WARN("未知状态，无法执行操作");
                            break;
                        }
                        break;
                    }
                }
                else if (mode == 2)
                {
                    if (bigClass == "A")
                    {
                        switch (mode2_step)
                        {
                        case 0: // 起飞
                            pose.position.x = 0.0;
                            pose.position.y = 0.0;
                            pose.position.z = 0.5; // 起飞高度0.5m

                            // 发布位置
                            fcu_location.data = "takeoff";
                            fcu_location_pub.publish(fcu_location);

                            if (local_pose.pose.position.z > 0.45 && local_pose.pose.position.z < 0.55)
                            {

                                if (sametimes > 10)
                                {
                                    mode1_step = 1;
                                    sametimes = 0;
                                    ROS_INFO("起飞完成，开始下一步");
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果高度不对，重置计数器
                            }
                            break;
                        case 1:
                            switch (subTask)
                            {
                            case 1: // A1
                                pose.position.x = 1.75;
                                pose.position.y = -0.5;
                                pose.position.z = 1.4; // 飞机飞到(1.75,-0.5,1.4)处
                                local_pose_pub.publish(pose);

                                if (local_pose.pose.position.x < 1.78 && local_pose.pose.position.x > 1.72 && local_pose.pose.position.y < -0.47 && local_pose.pose.position.y > -0.53 && local_pose.pose.position.z < 1.43 && local_pose.pose.position.z > 1.37)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "A1";
                                    fcu_location_pub.publish(fcu_location);
                                    if (vision_state.data == true)
                                    {
                                        mode2_step = 2;
                                        sametimes = 0;

                                        // 飞机开始前往下一个地点
                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(1.75,-0.5,1.4)并扫描A1");
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到A1，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes = 0; // 如果位置不对，重置计数器
                                }
                                break;

                            case 2: // A2
                                pose.position.x = 1.25;
                                pose.position.y = -0.5;
                                pose.position.z = 1.4;
                                local_pose_pub.publish(pose);

                                if (local_pose.pose.position.x < 1.28 && local_pose.pose.position.x > 1.22 && local_pose.pose.position.y < -0.47 && local_pose.pose.position.y > -0.53 && local_pose.pose.position.z < 1.43 && local_pose.pose.position.z > 1.37)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "A2";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        mode2_step = 3;
                                        sametimes = 0;

                                        // 飞机开始前往下一个地点
                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(1.25,-0.5,1.4)并扫描A2");
                                        pose.position.x = 0.75;
                                        pose.position.y = -0.5;
                                        pose.position.z = 1.4;
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到A2，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes = 0; // 如果位置不对，重置计数器
                                }
                                break;

                            case 3: // A3
                                pose.position.x = 0.75;
                                pose.position.y = -0.5;
                                pose.position.z = 1.4;
                                local_pose_pub.publish(pose);

                                if (local_pose.pose.position.x < 0.78 && local_pose.pose.position.x > 0.72 && local_pose.pose.position.y < -0.47 && local_pose.pose.position.y > -0.53 && local_pose.pose.position.z < 1.43 && local_pose.pose.position.z > 1.37)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "A3";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        mode2_step = 4;
                                        sametimes = 0;

                                        // 飞机开始前往下一个地点
                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(0.75,-0.5,1.4)并扫描A3");
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到A3，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes = 0; // 如果位置不对，重置计数器
                                }

                                break;

                            case 4:

                                break;
                            }
                            break;

                        case 2:
                            pose.position.x = 2.5;
                            pose.position.y = -0.5;
                            pose.position.z = 1.4;
                            local_pose_pub.publish(pose);

                            if (local_pose.pose.position.x < 2.53 && local_pose.pose.position.x > 2.47)
                            {
                                if (sametimes > 10)
                                {
                                    mode2_step = 3;
                                    sametimes = 0;
                                    ROS_INFO("已到达(1.25,-0.5,1.4)并扫描A5，下一步扫描A4到达(0.75,-0.5,1.4)");
                                    pose.position.x = 2.5;
                                    pose.position.y = -3.5;
                                    pose.position.z = 1.4;
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            break;

                        case 3:
                            if (local_pose.pose.position.y < -3.47 && local_pose.pose.position.y > -3.58)
                            {
                                if (sametimes > 10)
                                {
                                    // 发布位置
                                    fcu_location.data = "landing";
                                    fcu_location_pub.publish(fcu_location);

                                    sametimes = 0;
                                    offb_set_mode.request.custom_mode = "AUTO.LAND"; // 切换到自动降落模式
                                    if (current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
                                    {
                                        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                                        {
                                            ROS_INFO("成功切换到 AUTO.LAND 模式");
                                        }
                                        last_request = ros::Time::now();
                                    }
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0;
                            }
                            break;
                        }
                    }
                    else if (bigClass == "B")
                    {
                        switch (mode2_step)
                        {
                        case 0: // 起飞
                            pose.position.x = 0.0;
                            pose.position.y = 0.0;
                            pose.position.z = 0.5; // 起飞高度0.5m

                            // 发布位置
                            fcu_location.data = "takeoff";
                            fcu_location_pub.publish(fcu_location);

                            if (local_pose.pose.position.z > 0.45 && local_pose.pose.position.z < 0.55)
                            {

                                if (sametimes > 10)
                                {
                                    mode1_step = 1;
                                    sametimes = 0;
                                    ROS_INFO("起飞完成，开始下一步");
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果高度不对，重置计数器
                            }
                            break;

                        case 1:
                            pose.position.x = 0.0;
                            pose.position.y = -1.0;
                            pose.position.z = 0.5; // 起飞高度0.5m

                            // 发布位置
                            fcu_location.data = "takeoff";
                            fcu_location_pub.publish(fcu_location);

                            if (local_pose.pose.position.y < -0.97 && local_pose.pose.position.y > -1.03)

                            {

                                if (sametimes > 10)
                                {
                                    mode1_step = 2;
                                    sametimes = 0;
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果高度不对，重置计数器
                            }
                            break;

                        case 2:
                            switch (subTask)
                            {
                            case 1: // B1
                                pose.position.x = 0.75;
                                pose.position.y = -1.0;
                                pose.position.z = 1.4; // 飞机飞到(1.75,-0.5,1.4)处
                                local_pose_pub.publish(pose);

                                if (local_pose.pose.position.x < 0.78 && local_pose.pose.position.x > 0.72 && local_pose.pose.position.y < -0.97 && local_pose.pose.position.y > -1.03 && local_pose.pose.position.z < 1.43 && local_pose.pose.position.z > 1.37)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "B1";
                                    fcu_location_pub.publish(fcu_location);
                                    if (vision_state.data == true)
                                    {
                                        mode2_step = 3;
                                        sametimes = 0;

                                        // 飞机开始前往下一个地点
                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(0.75,-1.0,1.4)并扫描B1");
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到B1，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes = 0; // 如果位置不对，重置计数器
                                }
                                break;

                            case 2: // B2
                                pose.position.x = 1.25;
                                pose.position.y = -1.0;
                                pose.position.z = 1.4;
                                local_pose_pub.publish(pose);

                                if (local_pose.pose.position.x < 1.28 && local_pose.pose.position.x > 1.22 && local_pose.pose.position.y < -0.97 && local_pose.pose.position.y > -1.03 && local_pose.pose.position.z < 1.43 && local_pose.pose.position.z > 1.37)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "B2";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        mode2_step = 3;
                                        sametimes = 0;

                                        // 飞机开始前往下一个地点
                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(1.25,-1.0,1.4)并扫描B2");
                                        pose.position.x = 0.75;
                                        pose.position.y = -0.5;
                                        pose.position.z = 1.4;
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到B2，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes = 0; // 如果位置不对，重置计数器
                                }
                                break;

                            case 3: // A3
                                pose.position.x = 0.75;
                                pose.position.y = -1.0;
                                pose.position.z = 1.4;
                                local_pose_pub.publish(pose);

                                if (local_pose.pose.position.x < 0.78 && local_pose.pose.position.x > 0.72 && local_pose.pose.position.y < -0.97 && local_pose.pose.position.y > -1.03 && local_pose.pose.position.z < 1.43 && local_pose.pose.position.z > 1.37)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "B3";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        mode2_step = 3;
                                        sametimes = 0;

                                        // 飞机开始前往下一个地点
                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(0.75,-1.0,1.4)并扫描B3");
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到B3，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes = 0; // 如果位置不对，重置计数器
                                }

                                break;
                            }
                            break;

                        case 3:
                            pose.position.x = 2.5;
                            pose.position.y = -1.0;
                            pose.position.z = 1.4;
                            local_pose_pub.publish(pose);

                            if (local_pose.pose.position.x < 2.53 && local_pose.pose.position.x > 2.47)
                            {
                                if (sametimes > 10)
                                {
                                    mode2_step = 3;
                                    sametimes = 0;
                                    ROS_INFO("已到达(2.5,-1.0,1.4),下一步到降落点");
                                    pose.position.x = 2.5;
                                    pose.position.y = -3.5;
                                    pose.position.z = 1.4;
                                    local_pose_pub.publish(pose);
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            break;

                        case 4:
                            if (local_pose.pose.position.y < -3.47 && local_pose.pose.position.y > -3.58)
                            {
                                if (sametimes > 10)
                                {
                                    // 发布位置
                                    fcu_location.data = "landing";
                                    fcu_location_pub.publish(fcu_location);

                                    sametimes = 0;
                                    offb_set_mode.request.custom_mode = "AUTO.LAND"; // 切换到自动降落模式
                                    if (current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
                                    {
                                        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                                        {
                                            ROS_INFO("成功切换到 AUTO.LAND 模式");
                                        }
                                        last_request = ros::Time::now();
                                    }
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0;
                            }
                            break;
                        }
                    }
                    else if (bigClass == "C")
                    {
                        switch (mode2_step)
                        {
                        case 0: // 起飞
                            pose.position.x = 0.0;
                            pose.position.y = 0.0;
                            pose.position.z = 0.5; // 起飞高度0.5m

                            // 发布位置
                            fcu_location.data = "takeoff";
                            fcu_location_pub.publish(fcu_location);

                            if (local_pose.pose.position.z > 0.45 && local_pose.pose.position.z < 0.55)
                            {

                                if (sametimes > 10)
                                {
                                    mode1_step = 1;
                                    sametimes = 0;
                                    ROS_INFO("起飞完成，开始下一步");
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果高度不对，重置计数器
                            }
                            break;

                        case 1:
                            pose.position.x = 0.0;
                            pose.position.y = -2.5;
                            pose.position.z = 0.5; // 起飞高度0.5m

                            // 发布位置
                            fcu_location.data = "takeoff";
                            fcu_location_pub.publish(fcu_location);

                            if (local_pose.pose.position.y > -2.45 && local_pose.pose.position.y < -2.55)
                            {

                                if (sametimes > 10)
                                {
                                    mode1_step = 2;
                                    sametimes = 0;
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果高度不对，重置计数器
                            }
                            break;

                        case 2:
                            switch (subTask)
                            {
                            case 1: // c1
                                pose.position.x = 1.75;
                                pose.position.y = -2.5;
                                pose.position.z = 1.4; // 飞机飞到(1.75,-0.5,1.4)处
                                local_pose_pub.publish(pose);

                                if (local_pose.pose.position.x < 1.78 && local_pose.pose.position.x > 1.72 && local_pose.pose.position.y < -2.47 && local_pose.pose.position.y > -2.53 && local_pose.pose.position.z < 1.43 && local_pose.pose.position.z > 1.37)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "C1";
                                    fcu_location_pub.publish(fcu_location);
                                    if (vision_state.data == true)
                                    {
                                        mode2_step = 3;
                                        sametimes = 0;

                                        // 飞机开始前往下一个地点
                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(1.75,-2.5,1.4)并扫描C1");
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到C1，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes = 0; // 如果位置不对，重置计数器
                                }
                                break;

                            case 2: // C2
                                pose.position.x = 1.25;
                                pose.position.y = -2.5;
                                pose.position.z = 1.4;
                                local_pose_pub.publish(pose);

                                if (local_pose.pose.position.x < 1.28 && local_pose.pose.position.x > 1.22 && local_pose.pose.position.y < -2.47 && local_pose.pose.position.y > -2.53 && local_pose.pose.position.z < 1.43 && local_pose.pose.position.z > 1.37)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "C2";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        mode2_step = 3;
                                        sametimes = 0;

                                        // 飞机开始前往下一个地点
                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(1.25,-2.5,1.4)并扫描C2");
                                        pose.position.x = 0.75;
                                        pose.position.y = -0.5;
                                        pose.position.z = 1.4;
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到C2，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes = 0; // 如果位置不对，重置计数器
                                }
                                break;

                            case 3: // C3
                                pose.position.x = 1.75;
                                pose.position.y = -2.5;
                                pose.position.z = 1.4;
                                local_pose_pub.publish(pose);

                                if (local_pose.pose.position.x < 0.78 && local_pose.pose.position.x > 0.72 && local_pose.pose.position.y < -2.47 && local_pose.pose.position.y > -2.53 && local_pose.pose.position.z < 1.43 && local_pose.pose.position.z > 1.37)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "C3";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        mode2_step = 3;
                                        sametimes = 0;

                                        // 飞机开始前往下一个地点
                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(0.75,-2.5,1.4)并扫描C3");
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到C3，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes = 0; // 如果位置不对，重置计数器
                                }

                                break;
                            }
                            break;

                        case 3:
                            pose.position.x = 2.5;
                            pose.position.y = -2.5;
                            pose.position.z = 1.4;
                            local_pose_pub.publish(pose);

                            if (local_pose.pose.position.x < 2.53 && local_pose.pose.position.x > 2.47)
                            {
                                if (sametimes > 10)
                                {
                                    mode2_step = 3;
                                    sametimes = 0;
                                    ROS_INFO("已到达(2.5,-2.5,1.4),下一步到降落点");
                                    pose.position.x = 2.5;
                                    pose.position.y = -3.5;
                                    pose.position.z = 1.4;
                                    local_pose_pub.publish(pose);
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            break;

                        case 4:
                            if (local_pose.pose.position.y < -3.47 && local_pose.pose.position.y > -3.58)
                            {
                                if (sametimes > 10)
                                {
                                    // 发布位置
                                    fcu_location.data = "landing";
                                    fcu_location_pub.publish(fcu_location);

                                    sametimes = 0;
                                    offb_set_mode.request.custom_mode = "AUTO.LAND"; // 切换到自动降落模式
                                    if (current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
                                    {
                                        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                                        {
                                            ROS_INFO("成功切换到 AUTO.LAND 模式");
                                        }
                                        last_request = ros::Time::now();
                                    }
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0;
                            }
                            break;
                        }
                    }
                    else if (bigClass == "D")
                    {
                        switch (mode2_step)
                        {
                        case 0: // 起飞
                            pose.position.x = 0.0;
                            pose.position.y = 0.0;
                            pose.position.z = 0.5; // 起飞高度0.5m

                            // 发布位置
                            fcu_location.data = "takeoff";
                            fcu_location_pub.publish(fcu_location);

                            if (local_pose.pose.position.z > 0.45 && local_pose.pose.position.z < 0.55)
                            {

                                if (sametimes > 10)
                                {
                                    mode1_step = 1;
                                    sametimes = 0;
                                    ROS_INFO("起飞完成，开始下一步");
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果高度不对，重置计数器
                            }
                            break;

                        case 1:
                            pose.position.x = 0.0;
                            pose.position.y = -3.0;
                            pose.position.z = 0.5; // 起飞高度0.5m

                            // 发布位置
                            fcu_location.data = "takeoff";
                            fcu_location_pub.publish(fcu_location);

                            if (local_pose.pose.position.y < -2.97 && local_pose.pose.position.y > -3.03)
                            {

                                if (sametimes > 10)
                                {
                                    mode1_step = 2;
                                    sametimes = 0;
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0; // 如果高度不对，重置计数器
                            }
                            break;

                        case 2:
                            switch (subTask)
                            {
                            case 1: // D1
                                pose.position.x = 0.75;
                                pose.position.y = -3.0;
                                pose.position.z = 1.4; // 飞机飞到(1.75,-0.5,1.4)处
                                local_pose_pub.publish(pose);

                                if (local_pose.pose.position.x < 0.78 && local_pose.pose.position.x > 0.72 && local_pose.pose.position.y < -2.97 && local_pose.pose.position.y > -3.03 && local_pose.pose.position.z < 1.43 && local_pose.pose.position.z > 1.37)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "D1";
                                    fcu_location_pub.publish(fcu_location);
                                    if (vision_state.data == true)
                                    {
                                        mode2_step = 3;
                                        sametimes = 0;

                                        // 飞机开始前往下一个地点
                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(0.75,-3.0,1.4)并扫描D1");
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到D1，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes = 0; // 如果位置不对，重置计数器
                                }
                                break;

                            case 2: // D2
                                pose.position.x = 1.25;
                                pose.position.y = -3.0;
                                pose.position.z = 1.4;
                                local_pose_pub.publish(pose);

                                if (local_pose.pose.position.x < 1.28 && local_pose.pose.position.x > 1.22 && local_pose.pose.position.y < -2.97 && local_pose.pose.position.y > -3.03 && local_pose.pose.position.z < 1.43 && local_pose.pose.position.z > 1.37)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "D2";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        mode2_step = 3;
                                        sametimes = 0;

                                        // 飞机开始前往下一个地点
                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(1.25,-3.0,1.4)并扫描D2");
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到D2，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes = 0; // 如果位置不对，重置计数器
                                }
                                break;

                            case 3: // A3
                                pose.position.x = 1.75;
                                pose.position.y = -3.0;
                                pose.position.z = 1.4;
                                local_pose_pub.publish(pose);

                                if (local_pose.pose.position.x < 0.78 && local_pose.pose.position.x > 0.72 && local_pose.pose.position.y < -2.97 && local_pose.pose.position.y > -3.03 && local_pose.pose.position.z < 1.43 && local_pose.pose.position.z > 1.37)
                                {
                                    // 飞机到达目标点，等待识别
                                    fcu_state.data = true;
                                    fcu_state_pub.publish(fcu_state);
                                    // 发布位置
                                    fcu_location.data = "D3";
                                    fcu_location_pub.publish(fcu_location);

                                    if (vision_state.data == true)
                                    {
                                        mode2_step = 3;
                                        sametimes = 0;

                                        // 飞机开始前往下一个地点
                                        fcu_state.data = false;
                                        fcu_state_pub.publish(fcu_state);

                                        ROS_INFO("已到达(0.75,-3.0,1.4)并扫描D3");
                                    }
                                    else
                                    {
                                        ROS_INFO("未扫描到D3，等待识别");
                                    }
                                }
                                else
                                {
                                    sametimes = 0; // 如果位置不对，重置计数器
                                }

                                break;
                            }
                            break;

                        case 3:
                            pose.position.x = 2.5;
                            pose.position.y = -3.0;
                            pose.position.z = 1.4;
                            local_pose_pub.publish(pose);

                            if (local_pose.pose.position.x < 2.53 && local_pose.pose.position.x > 2.47)
                            {
                                if (sametimes > 10)
                                {
                                    mode2_step = 3;
                                    sametimes = 0;
                                    ROS_INFO("已到达(2.5,-1.0,1.4),下一步到降落点");
                                    pose.position.x = 2.5;
                                    pose.position.y = -3.5;
                                    pose.position.z = 1.4;
                                    local_pose_pub.publish(pose);
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            break;

                        case 4:
                            if (local_pose.pose.position.y < -3.47 && local_pose.pose.position.y > -3.58)
                            {
                                if (sametimes > 10)
                                {
                                    // 发布位置
                                    fcu_location.data = "landing";
                                    fcu_location_pub.publish(fcu_location);

                                    sametimes = 0;
                                    offb_set_mode.request.custom_mode = "AUTO.LAND"; // 切换到自动降落模式
                                    if (current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
                                    {
                                        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                                        {
                                            ROS_INFO("成功切换到 AUTO.LAND 模式");
                                        }
                                        last_request = ros::Time::now();
                                    }
                                }
                                else
                                {
                                    sametimes++;
                                }
                            }
                            else
                            {
                                sametimes = 0;
                            }
                            break;
                        }
                    }
                }
            }
        }
        // 发布位置控制
        local_pose_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }
}
