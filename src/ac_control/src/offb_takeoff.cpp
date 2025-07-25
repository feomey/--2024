// /**
//  * @file offb_node.cpp
//  * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
//  * Stack and tested in Gazebo Classic SITL
//  */

// #include <ros/ros.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <mavros_msgs/CommandBool.h>
// #include <mavros_msgs/SetMode.h>
// #include <mavros_msgs/State.h>

// mavros_msgs::State current_state;
// void state_cb(const mavros_msgs::State::ConstPtr& msg){
//     current_state = *msg;   // 无人机当前状态
// }

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "offb_node");
//     ros::NodeHandle nh;

//     ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
//             ("mavros/state", 10, state_cb);
//     ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
//             ("mavros/setpoint_position/local", 10);
//     ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
//             ("mavros/cmd/arming");
//     ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
//             ("mavros/set_mode");

//     //the setpoint publishing rate MUST be faster than 2Hz
//     ros::Rate rate(20.0);

//     // wait for FCU connection
//     while(ros::ok() && !current_state.connected){
//         ros::spinOnce();
//         rate.sleep();
//     }

//     geometry_msgs::PoseStamped pose;
//     pose.pose.position.x = 0;
//     pose.pose.position.y = 0;
//     pose.pose.position.z = 2;

//     //send a few setpoints before starting
//     for(int i = 100; ros::ok() && i > 0; --i){
//         local_pos_pub.publish(pose);
//         ros::spinOnce();
//         rate.sleep();
//     }

//     mavros_msgs::SetMode offb_set_mode;
//     offb_set_mode.request.custom_mode = "OFFBOARD";

//     mavros_msgs::CommandBool arm_cmd;
//     arm_cmd.request.value = true;

//     ros::Time last_request = ros::Time::now();

//     // 修改主循环逻辑，避免重复发送模式切换请求
//     while(ros::ok()) {
//         // 优先检查解锁状态
//         if(!current_state.armed) {
//             if(arming_client.call(arm_cmd) && arm_cmd.response.success) {
//                 ROS_INFO("Vehicle armed");
//                 last_request = ros::Time::now();
//             }
//         } 
//         // 已解锁但未进入Offboard模式
//         else if(current_state.armed && current_state.mode != "OFFBOARD") {
            
//             if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
//                 ROS_INFO("Current mode: %s", current_state.mode.c_str());
//                 ROS_INFO("Offboard enabled");
//                 last_request = ros::Time::now();
//             }
//         }
        
//         // 持续发送位置指令
//         local_pos_pub.publish(pose);
        
//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }

/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::PositionTarget pose;
    pose.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Current mode: %s", current_state.mode.c_str());
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


