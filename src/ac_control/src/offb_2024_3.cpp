#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include "ac_control/qr_scanner.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <deque>

// ================== 通用结构体与工具 ==================
struct Waypoint
{
    double x, y, z;
    std::string location;
    std::string scan_name;
    bool need_scan;
};

using TaskMap = std::map<int, Waypoint>;
using AreaMap = std::map<std::string, TaskMap>;

bool isReached(const geometry_msgs::PoseStamped &pose, const Waypoint &wp, double tol = 0.05)
{
    return std::fabs(pose.pose.position.x - wp.x) < tol &&
           std::fabs(pose.pose.position.y - wp.y) < tol &&
           std::fabs(pose.pose.position.z - wp.z) < tol;
}

void publishStatus(ros::Publisher &state_pub, ros::Publisher &loc_pub, std_msgs::Bool &state,
                   std_msgs::String &loc, const std::string &location, const bool &arrived)
{
    state.data = arrived;
    state_pub.publish(state);
    loc.data = location;
    loc_pub.publish(loc);
}
// ================== callbak函数 ==================
// 订阅飞控的状态，用于判断是否解锁以及当前模式，存储到current_state全局变量中
mavros_msgs::State current_state;
void stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

// 订阅本地位姿回调函数，将接收到的位姿存储在全局变量local_pose中
geometry_msgs::PoseStamped local_pose;
nav_msgs::Path path;
std::deque<geometry_msgs::PoseStamped> pose_history_;
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pose_history_.push_back(*msg);
    if (pose_history_.size() > 10000)
    {
        pose_history_.pop_front(); // 保持最多100个历史位姿
    }
    local_pose = *msg;
    // 发布路径点到路径话题

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "camera_link";
    path.poses.assign(pose_history_.begin(), pose_history_.end());
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
// ================== 任务1：全自动航点巡航 ==================
void runMode1(const std::vector<Waypoint> &waypoints, int &step, int &sametimes, mavros_msgs::PositionTarget &pose,
              ros::Publisher &local_pose_pub, ros::Publisher &fcu_state_pub, ros::Publisher &fcu_location_pub,
              std_msgs::Bool &fcu_state, std_msgs::String &fcu_location, const geometry_msgs::PoseStamped &local_pose,
              ros::ServiceClient &set_mode_client, ros::Time &last_request, bool &mission_finished,
              ros::ServiceClient &arming_client, int &mode)
{
    if (step < waypoints.size() - 1)
    {
        const Waypoint &wp = waypoints[step];
        pose.position.x = wp.x;
        pose.position.y = wp.y;
        pose.position.z = wp.z;
        local_pose_pub.publish(pose);
        fcu_location.data = wp.location;
        fcu_location_pub.publish(fcu_location);

        if (isReached(local_pose, wp))
        {
            if (sametimes > 10)
            {
                step++;
                sametimes = 0;
                ROS_INFO("已到达航点%s，前往下一个", wp.location.c_str());
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
    }
    else if (step == waypoints.size() - 1)
    {
        // 最后一个点，降落
        const auto &wp = waypoints[step];
        pose.position.x = wp.x;
        pose.position.y = wp.y;
        pose.position.z = wp.z;
        local_pose_pub.publish(pose);

        if (isReached(local_pose, wp))
        {
            if (sametimes > 10)
            {
                ROS_INFO("已到达%s，任务完成，准备降落", wp.location.c_str());
                // 切换到降落模式
                step += 1;
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
    }
    else if (step == waypoints.size())
    {

        pose.position.z = 0;
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "AUTO.LAND";
        if (current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("成功切换到 AUTO.LAND 模式");
            }
            last_request = ros::Time::now();
        }
        if (local_pose.pose.position.z < 0.15)
        {
            if (sametimes > 10)
            {
                mission_finished = true;
                mode = 3;
                ROS_INFO("降落完成，任务1结束，进入空闲模式");
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
        return;
    }
    else
    {
        ROS_WARN("step超出范围");
        return; // 如果step超出范围，直接返回
    }
}

// ================== 任务2：指定区间路径点 ==================
void runMode2(const std::string &bigClass, int targetIdx, int &step, int &sametimes, mavros_msgs::PositionTarget &pose,
              ros::Publisher &local_pose_pub, ros::Publisher &fcu_state_pub, ros::Publisher &fcu_location_pub,
              std_msgs::Bool &fcu_state, std_msgs::String &fcu_location, const geometry_msgs::PoseStamped &local_pose,
              const AreaMap &allAreas, ros::ServiceClient &set_mode_client,
              ros::Time &last_request, bool &mission_finished,
              ros::ServiceClient &arming_client, int &mode)
{
    // 查找目标区
    auto areaIt = allAreas.find(bigClass);
    if (areaIt == allAreas.end())
    {
        ROS_WARN("未知区域: %s", bigClass.c_str());
        return;
    }
    const TaskMap &taskMap = areaIt->second;

    // 路径点序列：起始点 -> 目标点 -> 结束点
    std::vector<int> pathIdx = {0, targetIdx, (int)taskMap.size() - 1};
    Waypoint landing_wp = {2.5, -3.5, 1.0, "landing", "降落点", false};

    // 步骤说明
    // step=0: 起飞到起始点
    // step=1: 到目标货位点
    // step=2: 到结束点
    // step=3: 到降落点并降落
    // step=4: 任务完成并上锁

    // 1. 起飞到起始点
    if (step == 0)
    {
        const Waypoint &wp = taskMap.at(pathIdx[0]);
        pose.position.x = wp.x;
        pose.position.y = wp.y;
        pose.position.z = wp.z;
        local_pose_pub.publish(pose);
        fcu_location.data = wp.location;
        fcu_location_pub.publish(fcu_location);

        if (isReached(local_pose, wp))
        {
            if (sametimes > 10)
            {
                step++;
                sametimes = 0;
                ROS_INFO("已到达%s，准备前往目标货位", wp.location.c_str());
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
        return;
    }
    // 2. 到目标货位点
    else if (step == 1)
    {
        const Waypoint &wp = taskMap.at(pathIdx[1]);
        pose.position.x = wp.x;
        pose.position.y = wp.y;
        pose.position.z = wp.z;
        local_pose_pub.publish(pose);

        fcu_location.data = wp.location;
        fcu_location_pub.publish(fcu_location);

        if (isReached(local_pose, wp))
        {
            if (sametimes > 10)
            {
                step++;
                sametimes = 0;
                ROS_INFO("已到达%s，准备前往结束点", wp.location.c_str());
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
        return;
    }
    // 3. 到结束点
    else if (step == 2)
    {
        const Waypoint &wp = taskMap.at(pathIdx[2]);
        pose.position.x = wp.x;
        pose.position.y = wp.y;
        pose.position.z = wp.z;
        local_pose_pub.publish(pose);
        fcu_location.data = wp.location;
        fcu_location_pub.publish(fcu_location);

        if (isReached(local_pose, wp))
        {
            if (sametimes > 10)
            {
                step++;
                sametimes = 0;
                ROS_INFO("已到达结束点%s，准备降落", wp.location.c_str());
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
        return;
    }
    // step=3: 到降落点并降落上锁
    else if (step == 3)
    {
        pose.position.x = landing_wp.x;
        pose.position.y = landing_wp.y;
        pose.position.z = landing_wp.z;
        local_pose_pub.publish(pose);
        fcu_location.data = landing_wp.location;
        fcu_location_pub.publish(fcu_location);

        if (isReached(local_pose, landing_wp))
        {
            if (sametimes > 10)
            {
                step++;
                sametimes = 0;
                ROS_INFO("已到达降落点，准备降落");

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
        return;
    }
    else if(step == 4)
    {
        pose.position.z = 0;
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "AUTO.LAND";
        if (current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("成功切换到 AUTO.LAND 模式");
            }
            last_request = ros::Time::now();
        }
        if (local_pose.pose.position.z < 0.15)
        {
            if (sametimes > 10)
            {
                mission_finished = true;
                mode = 3;
                step ++;
                ROS_INFO("降落完成，任务2结束，进入空闲模式");
            }
            else
            {
                sametimes++;
            }
        }
    }
    else 
    {
        ROS_INFO("区域%s任务全部完成", bigClass.c_str());
        return;
    }
}

// ================== 主函数 ==================
int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // 订阅与发布
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, poseCallback);
    ros::Subscriber vision_sub = nh.subscribe<ac_control::qr_scanner>("/vision_node/qr_scan", 10, VisinoCallBack);

    ros::Publisher local_pose_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ros::Publisher fcu_state_pub = nh.advertise<std_msgs::Bool>("/fcu_node/scan_state", 10);
    ros::Publisher fcu_location_pub = nh.advertise<std_msgs::String>("/fcu_node/location", 10);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);

    // 发布空的目标位置，确保无人机接收到初始位置
    mavros_msgs::PositionTarget pose;
    pose.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.5;

    // 等待飞控连接
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("飞控已连接，开始发布目标位置...");

    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pose_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // 设定无人机工作模式 offboard
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // 无人机解锁
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // 任务1航点
    std::vector<Waypoint> mode1_waypoints = {
        {0.0, 0.0, 0.5, "takeoff", "起飞", false},
        {0.75, -0.5, 1.0, "A6", "A6", false},
        {1.25, -0.5, 1.0, "A5", "A5", false},
        {1.75, -0.5, 1.0, "A4", "A4", false},
        {1.75, -0.5, 1.4, "A1", "A1", false},
        {1.25, -0.5, 1.4, "A2", "A2", false},
        {0.75, -0.5, 1.4, "A3", "A3", false},
        {0.0, -0.5, 1.4, "A-B_1", "A-B_1", false},
        {0.0, -1.0, 1.4, "A-B_2", "A-B_2", false},
        {0.75, -1.0, 1.4, "B1", "B1", false},
        {1.25, -1.0, 1.4, "B2", "B2", false},
        {1.75, -1.0, 1.4, "B3", "B3", false},
        {1.75, -1.0, 1.0, "B6", "B6", false},
        {1.25, -1.0, 1.0, "B5", "B5", false},
        {0.75, -1.0, 1.0, "B4", "B4", false},
        {0.75, -2.5, 1.0, "C6", "C6", false},
        {1.25, -2.5, 1.0, "C5", "C5", false},
        {1.75, -2.5, 1.0, "C4", "C4", false},
        {1.75, -2.5, 1.4, "C1", "C1", false},
        {1.25, -2.5, 1.4, "C2", "C2", false},
        {0.75, -2.5, 1.4, "C3", "C3", false},
        {0.0, -2.5, 1.4, "C-D_1", "C-D_1", false},
        {0.0, -3.0, 1.4, "C-D_2", "C-D_2", false},
        {0.75, -3.0, 1.4, "D1", "D1", false},
        {1.25, -3.0, 1.4, "D2", "D2", false},
        {1.75, -3.0, 1.4, "D3", "D3", false},
        {1.75, -3.0, 1.0, "D6", "D6", false},
        {1.25, -3.0, 1.0, "D5", "D5", false},
        {0.75, -3.0, 1.0, "D4", "D4", false},
        {2.50, -3.5, 1.0, "landing", "landing", false}};

    // 任务2区域任务点
    TaskMap areaA = {
        {0, {0.0, 0.0, 1.4, "A区起始", "A区起始", false}},
        {1, {1.75, -0.5, 1.4, "A1", "A1", false}},
        {2, {1.25, -0.5, 1.4, "A2", "A2", false}},
        {3, {0.75, -0.5, 1.4, "A3", "A3", false}},
        {4, {1.75, -0.5, 1.0, "A4", "A4", false}},
        {5, {1.24, -0.5, 1.0, "A5", "A5", false}},
        {6, {0.75, -0.5, 1.0, "A6", "A6", false}},
        {7, {2.50, -0.5, 1.0, "A区结束", "A区结束", false}}};
    TaskMap areaB = {
        {0, {0.0, -1.0, 1.4, "B区起始", "B区起始", false}},
        {1, {0.75, -1.0, 1.4, "B1", "B1", false}},
        {2, {1.25, -1.0, 1.4, "B2", "B2", false}},
        {3, {0.75, -1.0, 1.4, "B3", "B3", false}},
        {4, {0.75, -1.0, 1.1, "B4", "B4", false}},
        {5, {1.25, -1.0, 1.0, "B5", "B5", false}},
        {6, {1.75, -1.0, 1.0, "B6", "B6", false}},
        {7, {2.50, -1.0, 1.4, "B区结束", "B区结束", false}}};
    TaskMap areaC = {
        {0, {0.0, -2.5, 1.4, "C区起始", "C区起始", false}},
        {1, {1.75, -2.5, 1.4, "C1", "C1", false}},
        {2, {1.25, -2.5, 1.4, "C2", "C2", false}},
        {3, {0.75, -2.5, 1.4, "C3", "C3", false}},
        {4, {1.75, -2.5, 1.0, "C4", "C4", false}},
        {5, {1.25, -2.5, 1.0, "C5", "C5", false}},
        {6, {0.75, -2.5, 1.0, "C6", "C6", false}},
        {7, {2.50, -2.5, 1.0, "C区结束", "C区结束", false}}};
    TaskMap areaD = {
        {0, {0.0, -3.0, 1.4, "D区起始", "D区起始", false}},
        {1, {0.75, -3.0, 1.4, "D1", "D1", false}},
        {2, {1.25, -3.0, 1.4, "D2", "D2", false}},
        {3, {1.75, -3.0, 1.4, "D3", "D3", false}},
        {4, {0.75, -3.0, 1.1, "D4", "D4", false}},
        {5, {1.25, -3.0, 1.0, "D5", "D5", false}},
        {6, {1.75, -3.0, 1.0, "D6", "D6", false}},
        {7, {2.50, -3.0, 1.0, "D区结束", "D区结束", false}}};
    AreaMap allAreas = {
        {"A", areaA},
        {"B", areaB},
        {"C", areaC},
        {"D", areaD}};

    // 任务参数
    int mode = 2;               // 1为任务1，2为任务2
    int last_mode = 0;        // 上一个任务模式
    std::string bigClass = "A"; // 任务2区域
    int subTask = 1;            // 任务2目标点编号
    int mode1_step = 0;
    int mode2_step = 0;
    int sametimes = 0;
    bool mission_finished = false;
    std_msgs::Bool fcu_state;

    std_msgs::String fcu_location;

    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {
        // 检查是否有新的任务模式
        if ((vision_date.mode == 1 || vision_date.mode == 2) && vision_date.mode != last_mode)
        {
            mode = vision_date.mode;
            last_mode = mode;
            mission_finished = false;
            mode1_step = 0;
            mode2_step = 0;
            sametimes = 0;
            
            ROS_INFO("收到新任务模式: %d，开始新任务", mode);
        }
        

        if (mission_finished)
        {
            ros::spinOnce();
            rate.sleep();
        }
        else
        {
            if (current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                    ROS_INFO("mode: %d, mission_finished: %d", mode, mission_finished);
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
                else
                {
                    if (mode == 1)
                    {
                        runMode1(mode1_waypoints, mode1_step, sametimes, pose, local_pose_pub,
                                 fcu_state_pub, fcu_location_pub, fcu_state, fcu_location, local_pose,
                                 set_mode_client, last_request, mission_finished, arming_client, mode);
                    }
                    else if (mode == 2)
                    {
                        runMode2(bigClass, subTask, mode2_step, sametimes, pose, local_pose_pub,
                                 fcu_state_pub, fcu_location_pub, fcu_state, fcu_location, local_pose,
                                 allAreas, set_mode_client, last_request,
                                 mission_finished, arming_client, mode);
                    }
                }
            }
        }
        // 发布当前位姿和路径
        pose.header.stamp = ros::Time::now();
        path_pub.publish(path);
        local_pose_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}