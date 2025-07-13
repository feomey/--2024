#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>

// 订阅飞控的状态，用于判断是否解锁以及当前模式，存储到current_state全局变量中
mavros_msgs::State current_state;
void stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

// 订阅本地位姿回调函数，将接收到的位姿存储在全局变量local_pose中
geometry_msgs::PoseStamped local_pose;
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    local_pose = *msg;
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

    // 发布到目标位置的话题
    ros::Publisher local_pose_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);

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
    pose.position.z = 2.0; // 设置初始高度为2米

    // 等待飞控连接
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("飞控已连接，开始发布目标位置...");


    // 进入 Offboard 模式前，需持续发送设定点
    for(int i = 100; ros::ok() && i > 0; --i)
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

    // 用于走圈的变量
    int step = 0;
    int sametimes = 0;

    while(ros::ok())
    {
        // 飞机状态设定与判定
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
        // ROS_INFO("当前模式: %s, 解锁状态: %s", current_state.mode.c_str(), current_state.armed ? "已解锁" : "未解锁");
            else
            {

                switch(step)
                {
                    case 0: // 起飞
                        pose.position.x = 0.0;
                        pose.position.y = 0.0;
                        pose.position.z = 2.0; // 起飞高度2米
                    
                        if(local_pose.pose.position.z > 1.9 && local_pose.pose.position.z < 2.1)
                        {

                            if(sametimes > 20)
                            {
                                step = 1;
                                sametimes = 0;
                                ROS_INFO("起飞完成，开始前进");
                            }else{
                                sametimes++;
                            }

                        }
                        else
                        {
                            sametimes = 0; // 如果高度不对，重置计数器
                        }
                        break;
                    case 1:// x轴前进
                        
                        if(local_pose.pose.position.z > 1.9 && local_pose.pose.position.z < 2.1)
                        {

                            if(sametimes > 20)
                            {
                                step = 2;
                                sametimes = 0;
                                ROS_INFO("(40,0)");
                                pose.position.x = 40;
                                pose.position.y = 0;
                                pose.position.z = 2.0; 

                            }else{
                                sametimes++;
                            }

                        }
                        else
                        {
                            sametimes = 0; // 如果高度不对，重置计数器
                        }
                    
                    break;

                    case 2: // y前进
                        if(local_pose.pose.position.x > 39.9 && local_pose.pose.position.x < 40.1)
                        {
                            
                            if(sametimes > 20)
                            {
                                step = 3;
                                sametimes = 0;
                                ROS_INFO("(40.40)");
                                pose.position.x = 40;
                                pose.position.y = 40;
                                pose.position.z = 2.0; 
                            }
                            else{
                                sametimes++;
                            }
                        }
                        else
                        {
                            sametimes = 0; // 如果位置不对，重置计数器
                        }
                        break;
                    
                    case 3: // y轴前进
                        if(local_pose.pose.position.y > 39.9 && local_pose.pose.position.y < 40.1)
                        {
                            if(sametimes > 20)
                            {
                                step = 4;
                                sametimes = 0;
                                ROS_INFO("(0,40)");
                                pose.position.x = 0;
                                pose.position.y = 40;
                                pose.position.z = 2.0; 
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

                    case 4: // x轴后退
                        if(local_pose.pose.position.x < 0.1 && local_pose.pose.position.x > -0.1)
                        {
                            if(sametimes > 20)
                            {
                                step = 5;
                                sametimes = 0;
                                ROS_INFO("(0,0)");
                                pose.position.x = 0.0;
                                pose.position.y = 0.0;
                                pose.position.z = 2.0; 
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
                    
                    case 5: // z轴下降
                        if(local_pose.pose.position.y < 0.1 && local_pose.pose.position.y > -0.1)
                        {
                            if(sametimes > 20)
                            {
                                step = 6;
                                sametimes = 0;
                                ROS_INFO("y轴后退完成, 开始z轴下降");
                                pose.position.x = 0.0;
                                pose.position.y = 0.0;
                                pose.position.z = 0.5; // 降低高度到0.5米
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
                    
                    case 6: // 降落上锁
                        offb_set_mode.request.custom_mode = "AUTO.LAND"; // 切换到自动降落模式
                        if(current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
                        {
                            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                            {
                                ROS_INFO("成功切换到 AUTO.LAND 模式");
                            }
                            last_request = ros::Time::now();
                        }
                        break;

                    default:
                        ROS_WARN("未知状态，无法执行操作");
                        break;
                        
                }


                    


                
            }
        }
        // 发布位置控制
        local_pose_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    
        
    }

}





