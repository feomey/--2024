#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <cmath>

struct Waypoint {
    float x, y, z;
    bool need_scan;
    const char* info;
};

std::vector<Waypoint> waypoints = {
    {0.0, 0.0, 0.5, false, "起飞"},
    {0.0, 0.0, 1.0, false, "到达(0,0,1)"},
    {0.75, -0.5, 1.0, true, "扫描A6"},
    {1.25, -0.5, 1.0, true, "扫描A5"},
    // ... 继续添加所有点 ...
    {2.5, -3.5, 1.0, false, "降落点"}
};

mavros_msgs::State current_state;
geometry_msgs::PoseStamped local_pose;
std_msgs::Bool vision_state;

void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    local_pose = *msg;
}
void visionCallback(const std_msgs::Bool::ConstPtr& msg) {
    vision_state = *msg;
}

bool isReached(const geometry_msgs::PoseStamped& cur, const Waypoint& wp, float tol=0.02) {
    return std::fabs(cur.pose.position.x - wp.x) < tol &&
           std::fabs(cur.pose.position.y - wp.y) < tol &&
           std::fabs(cur.pose.position.z - wp.z) < tol;
}

int main(int argc, char **argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, poseCallback);
    ros::Subscriber vision_sub = nh.subscribe<std_msgs::Bool>("/vision_node/qr_Scan/state", 10, visionCallback);

    ros::Publisher local_pose_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ros::Publisher fcu_state_pub = nh.advertise<std_msgs::Bool>("/fcu_node/scan_state", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);

    // 等待飞控连接
    while(ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("飞控已连接，开始发布目标位置...");

    mavros_msgs::PositionTarget pose;
    pose.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    // 进入 Offboard 模式前，需持续发送设定点
    for(int i = 100; ros::ok() && i > 0; --i) {
        pose.position.x = waypoints[0].x;
        pose.position.y = waypoints[0].y;
        pose.position.z = waypoints[0].z;
        local_pose_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    int idx = 0, sametimes = 0;
    std_msgs::Bool fcu_state;
    fcu_state.data = false;

    while (ros::ok()) {
        // 模式切换与解锁
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }

        // 主流程
        pose.position.x = waypoints[idx].x;
        pose.position.y = waypoints[idx].y;
        pose.position.z = waypoints[idx].z;

        if (isReached(local_pose, waypoints[idx])) {
            if (waypoints[idx].need_scan) {
                fcu_state.data = true;
                fcu_state_pub.publish(fcu_state);
                if (vision_state.data) {
                    sametimes = 0;
                    fcu_state.data = false;
                    fcu_state_pub.publish(fcu_state);
                    ROS_INFO("%s完成，前往下一个点", waypoints[idx].info);
                    idx++;
                } else {
                    ROS_INFO("等待扫描: %s", waypoints[idx].info);
                }
            } else {
                if (++sametimes > 10) {
                    sametimes = 0;
                    ROS_INFO("%s完成，前往下一个点", waypoints[idx].info);
                    idx++;
                }
            }
        } else {
            sametimes = 0;
        }

        if (idx >= waypoints.size()) {
            offb_set_mode.request.custom_mode = "AUTO.LAND";
            set_mode_client.call(offb_set_mode);
            ROS_INFO("任务完成，切换降落模式");
            break;
        }

        local_pose_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}