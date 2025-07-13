#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "ac_control/qr_scanner.h"
#include <vector>
#include <string>
#include <map>
#include <cmath>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <deque>
// ===================== 全局变量 =====================


// ================== 通用结构体与工具 ==================
struct Waypoint {
    double x, y, z;
    std::string location;
    std::string scan_name;
    bool need_scan;
};

bool isReached(const geometry_msgs::PoseStamped& pose, const Waypoint& wp, double tol = 0.05) {
    return std::fabs(pose.pose.position.x - wp.x) < tol &&
           std::fabs(pose.pose.position.y - wp.y) < tol &&
           std::fabs(pose.pose.position.z - wp.z) < tol;
}

void publishStatus(ros::Publisher& state_pub, ros::Publisher& loc_pub, std_msgs::Bool& state,
                     std_msgs::String& loc, const std::string& location, const bool& arrived) {
    state.data = arrived;
    state_pub.publish(state);
    loc.data = location;
    loc_pub.publish(loc);
}
// ================== 自定义功能函数 ==================
// 解析视觉任务数据，提取大类和子任务编号
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
    if (pose_history_.size() > 10000) {
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

// ================== mode1主流程 ==================
void runMode1(const std::vector<Waypoint>& waypoints, int& step, int& sametimes, mavros_msgs::PositionTarget& pose,
              ros::Publisher& local_pose_pub, ros::Publisher& fcu_state_pub, ros::Publisher& fcu_location_pub,
              std_msgs::Bool& fcu_state, std_msgs::String& fcu_location, const geometry_msgs::PoseStamped& local_pose,
              const std_msgs::Bool& vision_state,ros::ServiceClient& set_mode_client, ros::Time& last_request) {
    if (step < waypoints.size() - 1)
    { 
        const auto& wp = waypoints[step];
        pose.position.x = wp.x;
        pose.position.y = wp.y;
        pose.position.z = wp.z;
        local_pose_pub.publish(pose);

        if (isReached(local_pose, wp)) {
            if (sametimes > 10) {
                if(wp.need_scan)   publishStatus(fcu_state_pub, fcu_location_pub, fcu_state, fcu_location, wp.location, wp.need_scan);
                if (vision_state.data) {
                    // 如果需要扫描且视觉状态为真，表示已扫描到
                    publishStatus(fcu_state_pub, fcu_location_pub, fcu_state, fcu_location, wp.scan_name, false);
                    step++;
                    sametimes = 0;
                    ROS_INFO("已到达%s并扫描%s，前往下一个点", wp.location.c_str(), wp.scan_name.c_str());
                } else {
                    ROS_INFO("未扫描到%s，等待识别", wp.scan_name.c_str());
                }
            } else {
                sametimes++;
            }
        } else {
            sametimes = 0;
        }
    }
    else if(step == waypoints.size() - 1) {
        // 最后一个点，降落
        const auto& wp = waypoints[step];
        pose.position.x = wp.x;
        pose.position.y = wp.y;
        pose.position.z = wp.z;
        local_pose_pub.publish(pose);

        if(isReached(local_pose, wp)) {
            if (sametimes > 10) {
                ROS_INFO("已到达%s，任务完成，准备降落", wp.location.c_str());
                // 切换到降落模式
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
            } else {
                sametimes++;
            }
        } else {
            sametimes = 0;
        }
    }
    else 
    {
        ROS_WARN("step超出范围");
        return; // 如果step超出范围，直接返回
    }
        
}

// ================== mode2主流程 ==================
using TaskMap = std::map<int, Waypoint>;
using AreaMap = std::map<std::string, TaskMap>;

void runMode2(const std::string& bigClass, int targetIdx, int& step, int& sametimes, mavros_msgs::PositionTarget& pose,
              ros::Publisher& local_pose_pub, ros::Publisher& fcu_state_pub, ros::Publisher& fcu_location_pub,
              std_msgs::Bool& fcu_state, std_msgs::String& fcu_location, const geometry_msgs::PoseStamped& local_pose,
              const std_msgs::Bool& vision_state, const AreaMap& allAreas,
              mavros_msgs::SetMode& offb_set_mode, ros::ServiceClient& set_mode_client,
              mavros_msgs::State& current_state, ros::Time& last_request)
{
    // 查找目标区
    auto areaIt = allAreas.find(bigClass);
    if (areaIt == allAreas.end()) {
        ROS_WARN("未知区域: %s", bigClass.c_str());
        return;
    }
    const TaskMap& taskMap = areaIt->second;

    // 路径点序列：起始点 -> 目标点 -> 结束点
    std::vector<int> pathIdx = {0, targetIdx, (int)taskMap.size() - 1};
    Waypoint landing_wp = {2.5, -3.5, 1.0, "landing", "降落点", false};

    // 步骤说明
    // step=0: 起飞到起始点
    // step=1: 到目标货位点
    // step=2: 到结束点
    // step=3: 到降落点并降落
    // step>3: 任务完成

    // 1. 起飞到起始点
    if (step == 0) {
        const Waypoint& wp = taskMap.at(pathIdx[0]);
        pose.position.x = wp.x;
        pose.position.y = wp.y;
        pose.position.z = wp.z;
        local_pose_pub.publish(pose);
        fcu_location.data = wp.location;
        fcu_location_pub.publish(fcu_location);

        if (isReached(local_pose, wp)) {
            if (sametimes > 10) {
                step++;
                sametimes = 0;
                ROS_INFO("已到达%s，准备前往目标货位", wp.location.c_str());
            } else {
                sametimes++;
            }
        } else {
            sametimes = 0;
        }
        return;
    }

    // 2. 到目标货位点
    if (step == 1) {
        const Waypoint& wp = taskMap.at(pathIdx[1]);
        pose.position.x = wp.x;
        pose.position.y = wp.y;
        pose.position.z = wp.z;
        local_pose_pub.publish(pose);

        fcu_location.data = wp.location;
        fcu_location_pub.publish(fcu_location);

        if (isReached(local_pose, wp)) {
            if (sametimes > 10) {
                publishStatus(fcu_state_pub, fcu_location_pub, fcu_state, fcu_location, wp.location, true);
                if (vision_state.data) {
                    step++;
                    sametimes = 0;
                    publishStatus(fcu_state_pub, fcu_location_pub, fcu_state, fcu_location, wp.location, false);
                    ROS_INFO("已到达%s并扫描，准备前往结束点", wp.location.c_str());
                } else {
                    ROS_INFO("未扫描到%s，等待识别", wp.location.c_str());
                }
            } else {
                sametimes++;   
            }
        } else {
            sametimes = 0;
        }
        return;
    }

    // 3. 到结束点
    if (step == 2) {
        const Waypoint& wp = taskMap.at(pathIdx[2]);
        pose.position.x = wp.x;
        pose.position.y = wp.y;
        pose.position.z = wp.z;
        local_pose_pub.publish(pose);
        fcu_location.data = wp.location;
        fcu_location_pub.publish(fcu_location);

        if (isReached(local_pose, wp)) {
            if (sametimes > 10) {
                step++;
                sametimes = 0;
                ROS_INFO("已到达结束点%s，准备降落", wp.location.c_str());
            } else {
                sametimes++;
            }
        } else {
            sametimes = 0;
        }
        return;
    }

    // step=3: 到降落点
    if (step == 3) {
        pose.position.x = landing_wp.x;
        pose.position.y = landing_wp.y;
        pose.position.z = landing_wp.z;
        local_pose_pub.publish(pose);
        fcu_location.data = landing_wp.location;
        fcu_location_pub.publish(fcu_location);

        if (isReached(local_pose, landing_wp)) {
            if (sametimes > 10) {
                step++;
                sametimes = 0;
                ROS_INFO("已到达降落点，准备降落");
            } else {
                sametimes++;
            }
        } else {
            sametimes = 0;
        }
        return;
    }

    // step=4: 降落
    if (step == 4) {
        pose.position.x = landing_wp.x;
        pose.position.y = landing_wp.y;
        pose.position.z = 0.5; // 降落高度
        local_pose_pub.publish(pose);
        fcu_location.data = "landing";
        fcu_location_pub.publish(fcu_location);

        if (std::fabs(local_pose.pose.position.z - 0.5) < 0.05) {
            if (sametimes > 10) {
                offb_set_mode.request.custom_mode = "AUTO.LAND";
                if (current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                        ROS_INFO("成功切换到 AUTO.LAND 模式");
                    }
                    last_request = ros::Time::now();
                }
            } else {
                sametimes++;
            }
        } else {
            sametimes = 0;
        }
        return;
    }

    // step>4: 任务完成
    if (step > 4) {
        ROS_INFO("区域%s任务全部完成", bigClass.c_str());
        return;
    }
}

// ================== 主函数 ==================
int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // 订阅与发布
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, poseCallback);
    ros::Subscriber visino_state_sub = nh.subscribe<std_msgs::Bool>("/vision_node/qr_scan/state", 10, VisinoStateCallBack);
    ros::Subscriber vision_sub = nh.subscribe<ac_control::qr_scanner>("/vision_node/qr_scan", 10, VisinoCallBack);

    ros::Publisher local_pose_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ros::Publisher fcu_state_pub = nh.advertise<std_msgs::Bool>("/fcu_node/scan_state", 10);
    ros::Publisher fcu_location_pub = nh.advertise<std_msgs::String>("/fcu_node/location", 10);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 10);

    // 服务客户端
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
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("飞控已连接，开始发布目标位置...");

    for (int i = 100; ros::ok() && i > 0; --i) {
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

    // 记录当前时间
    ros::Time last_request = ros::Time::now();

    std_msgs::Bool fcu_state;
    fcu_state.data = false;
    fcu_state_pub.publish(fcu_state);

    std_msgs::String fcu_location;
    fcu_location.data = "takeoff";
    fcu_location_pub.publish(fcu_location);

    // mode1点位表
    std::vector<Waypoint> mode1_waypoints = {
        {0.0, 0.0, 0.5, "takeoff", "起飞", false},
        {0.75, -0.5, 1.0, "A6", "A6", true},
        {1.25, -0.5, 1.0, "A5", "A5", true},
        {1.75, -0.5, 1.0, "A4", "A4", true},
        {1.75, -0.5, 1.4, "A1", "A1", true},
        {1.25, -0.5, 1.4, "A2", "A2", true},
        {0.75, -0.5, 1.4, "A3", "A3", true},
        {0.0, -0.5, 1.4, "A-B_1", "A-B_1", false},
        {0.0, -1.0, 1.4, "A-B_2", "A-B_2", false},
        {0.75, -1.0, 1.4, "B1", "B1", true},
        {1.25, -1.0, 1.4, "B2", "B2", true},
        {1.75, -1.0, 1.4, "B3", "B3", true},
        {1.75, -1.0, 1.0, "B6", "B6", true},
        {1.25, -1.0, 1.0, "B5", "B5", true},
        {0.75, -1.0, 1.0, "B4", "B4", true},
        {0.75, -2.5, 1.0, "C6", "C6", true},
        {1.25, -2.5, 1.0, "C5", "C5", true},
        {1.75, -2.5, 1.0, "C4", "C4", true},
        {1.75, -2.5, 1.4, "C1", "C1", true},
        {1.25, -2.5, 1.4, "C2", "C2", true},
        {0.75, -2.5, 1.4, "C3", "C3", true},
        {0.0, -2.5, 1.4, "B-C_1", "B-C_1", false},
        {0.0, -3.0, 1.4, "B-C_2", "B-C_2", false},
        {0.75, -3.0, 1.4, "D1", "D1", true},
        {1.25, -3.0, 1.4, "D2", "D2", true},
        {1.75, -3.0, 1.4, "D3", "D3", true},
        {1.75, -3.0, 1.0, "D6", "D6", true},
        {1.25, -3.0, 1.0, "D5", "D5", true},
        {0.75, -3.0, 1.0, "D4", "D4", true},
        {2.50, -3.5, 1.0, "landing", "landing", false} 



    };

    // mode2区域任务点
    TaskMap areaA = {
        {0, {0.0, 0.0, 1.4, "A区起始", "A区起始"}},
        {1, {1.75, -0.5, 1.4, "A1", "A1"}},
        {2, {1.25, -0.5, 1.4, "A2", "A2"}},
        {3, {0.75, -0.5, 1.4, "A3", "A3"}},
        {4, {1.75, -0.5, 1.0, "A4", "A4"}},
        {5, {1.24, -0.5, 1.0, "A5", "A5"}},
        {6, {0.75, -0.5, 1.0, "A6", "A6"}},
        {7, {2.50, -0.5, 1.0, "A区结束", "A区结束"}}
    };
    TaskMap areaB = {
        {0, {0.0, -1.0, 1.4, "B区起始", "B区起始"}},
        {1, {0.75, -1.0, 1.4, "B1", "B1"}},
        {2, {1.25, -1.0, 1.4, "B2", "B2"}},
        {3, {0.75, -1.0, 1.4, "B3", "B3"}},
        {4, {0.75, -1.0, 1.1, "B4", "B4"}},
        {5, {1.25, -1.0, 1.0, "B5", "B5"}},
        {6, {1.75, -1.0, 1.0, "B6", "B6"}},
        {7, {2.50, -1.0, 1.4, "B区结束", "B区结束"}}
    };
    TaskMap areaC = {
        {0, {0.0, -2.5, 1.4, "C区起始", "C区起始"}},
        {1, {1.75, -2.5, 1.4, "C1", "C1"}},
        {2, {1.25, -2.5, 1.4, "C2", "C2"}},
        {3, {0.75, -2.5, 1.4, "C3", "C3"}},
        {4, {1.75, -2.5, 1.0, "C4", "C4"}},
        {5, {1.25, -2.5, 1.0, "C5", "C5"}},
        {6, {0.75, -2.5, 1.0, "C6", "C6"}},
        {7, {2.50, -2.5, 1.0, "C区结束", "C区结束"}}
    };
    TaskMap areaD = {
        {0, {0.0, -3.0, 1.4, "D区起始", "D区起始"}},
        {1, {0.75, -3.0, 1.4, "D1", "D1"}}, 
        {2, {1.25, -3.0, 1.4, "D2", "D2"}},
        {3, {1.75, -3.0, 1.4, "D3", "D3"}},
        {4, {0.75, -3.0, 1.1, "D4", "D4"}},
        {5, {1.25, -3.0, 1.0, "D5", "D5"}},
        {6, {1.75, -3.0, 1.0, "D6", "D6"}},
        {7, {2.50, -3.0, 1.0, "D区结束", "D区结束"}}
    };
    AreaMap allAreas = {
        {"A", areaA},
        {"B", areaB},
        {"C", areaC},
        {"D", areaD}
    };

    int mode1_step = 0, mode2_step = 0, sametimes = 0;
    int mode = 1;
    std::string bigClass;
    int subTask;

    while (ros::ok()) { 
        // 解析视觉任务
        parseTask(vision_date.position, bigClass, subTask);
        mode = vision_date.mode;

        // 模式切换与解锁逻辑
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            } else {
                if (mode == 1) {
                    runMode1(mode1_waypoints, mode1_step, sametimes, pose, local_pose_pub, 
                            fcu_state_pub, fcu_location_pub, fcu_state, fcu_location, local_pose, 
                            vision_state, set_mode_client, last_request);
                } else if (mode == 2) {
                    runMode2(bigClass, subTask, mode2_step, sametimes, pose, local_pose_pub, 
                            fcu_state_pub, fcu_location_pub, fcu_state, fcu_location, local_pose, 
                            vision_state, allAreas, offb_set_mode, set_mode_client, current_state, last_request);
                }
            }
        }

        local_pose_pub.publish(pose);
        path_pub.publish(path);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}