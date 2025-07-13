#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/PositionTarget.h>

#define RATE 20 // 20hz
#define r 2.5     // radius
#define cycle_s 15 
#define STEP (cycle_s * RATE)
#define PI 3.14

mavros_msgs::State current_state;
void state_cb (const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pos = *msg;
}

int main(int argc, char **argv)
{   

    ros::init(argc, argv, "offb_cfx");
    ros::NodeHandle nh;
    // 订阅无人机当前状态 
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // 发布无人机本地位置（控制）
    ros::Publisher target_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);

    // 服务的客户端（设定无人机的模式、状态）
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(RATE);

    mavros_msgs::PositionTarget Target_P;
    Target_P.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    Target_P.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | 
                         mavros_msgs::PositionTarget::IGNORE_VY |
                         mavros_msgs::PositionTarget::IGNORE_VZ |
                         mavros_msgs::PositionTarget::IGNORE_AFX |
                         mavros_msgs::PositionTarget::IGNORE_AFY |
                         mavros_msgs::PositionTarget::IGNORE_AFZ |
                         mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    Target_P.position.x = 0;
    Target_P.position.y = 0;
    Target_P.position.z = 0;
    Target_P.velocity.x = 0;
    Target_P.velocity.y = 0;
    Target_P.velocity.z = 0;
    Target_P.acceleration_or_force.x = 0;
    Target_P.acceleration_or_force.y = 0;
    Target_P.acceleration_or_force.z = 0;
    Target_P.yaw = 0;
    Target_P.yaw_rate = 0;

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        target_pub.publish(Target_P);
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

//  用于走圈的变量
    int step = 0;
    int sametimes = 0;
    int i = 0;
    double theta = -PI/2;

    while(ros::ok()){
        // 无人机状态设定与判断      
        // 进入while循环后，先循环5s，然后再向客户端发送无人机状态设置的消息
        // set_mode_client.call   arming_client.call 
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
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
            else    //  无人机 Offboard enabled && Vehicle armed 后S
            {    
                ROS_INFO("hello 1");
                switch (step)
                {
                case 0:
                    Target_P.position.z = 2.0;
                    if(local_pos.pose.position.z >1.95 && local_pos.pose.position.z <2.05)
                        step = 1;
                    ROS_INFO("hello case0");
                    break;

                case 1:
                    theta += 2*PI/STEP;
                    Target_P.position.x = r*cos(theta);
                    Target_P.position.y = r*sin(theta) + r;
                    Target_P.position.z = 2.0;

                    Target_P.velocity.x = -r*sin(theta);
                    Target_P.velocity.y = r*cos(theta);
                    Target_P.velocity.z = 0;

                    Target_P.acceleration_or_force.x = -r*cos(theta);
                    Target_P.acceleration_or_force.y = -r*sin(theta);
                    Target_P.acceleration_or_force.x = 0;

                    Target_P.yaw = atan2(Target_P.velocity.y, Target_P.velocity.x = -r*sin(theta));

                    i ++;
                    if(i > STEP)
                    {
                        i = 1; 
                        step = 2;
                    }
                    ROS_INFO("hello case0");

                    break;
                
                case 2:
                    if(sametimes < 40)
                    {
                        sametimes ++;
                    }
                    else
                    {
                        offb_set_mode.request.custom_mode = "AUTO.LAND";
                        if (current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
                        {
                            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                            {
                                ROS_INFO("AUTO.LAND enabled");
                                step = 3;
                            }
                            last_request = ros::Time::now();
                        }
                    }
                    break;
                default:
                    break;
                }
            }
        }
        // 发布位置控制信息
        target_pub.publish(Target_P);
        ros::spinOnce();
        rate.sleep();   // 影响消息发布与更新的周期
    }

    return 0;
}
