#include <ros/ros.h>
#include "ac_control/visino_fcu.h"


bool vision_flag;
bool fcu_flag;

bool ReadStateCallBack(ac_control::visino_fcu::Request &req, ac_control::visino_fcu::Response &res)
{
    res.fcu_flag = fcu_flag;
    res.vision_flag = vision_flag;
    res.success = true;

    return true;
}

bool FCUUpdateStateCallBack(ac_control::visino_fcu::Request &req, ac_control::visino_fcu::Response &res)
{
    fcu_flag = req.fcu_flag;
    res.success = true;
    ROS_INFO("fcu_flag = %d",fcu_flag);
    
    return true;
}

bool VisionUpdateStateCallBack(ac_control::visino_fcu::Request &req, ac_control::visino_fcu::Response &res)
{
    vision_flag = req.vision_flag;
    res.success = true;
    ROS_INFO("vision_flag = %d", vision_flag);
    
    return true;
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, ""); // 设置中文环境

    ros::init(argc, argv, "visino_fcu_service_node");

    ros::NodeHandle nh;

    // 创建服务
    ros::ServiceServer read_service = nh.advertiseService("/visino_fcu/read_state", ReadStateCallBack);
    ros::ServiceServer fcu_update_service = nh.advertiseService("/visino_fcu/fcu_update_state", FCUUpdateStateCallBack);
    ros::ServiceServer vision_update_service = nh.advertiseService("/visino_fcu/vision_update_state", VisionUpdateStateCallBack);

    ros::spin();

    return 0;
}

