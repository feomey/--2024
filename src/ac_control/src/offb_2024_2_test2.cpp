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

    // 4. 到降落点并降落（假设降落点和结束点重合，也可自定义降落点）
    if (step == 3) {
        const Waypoint& wp = taskMap.at(pathIdx[2]);
        pose.position.x = wp.x;
        pose.position.y = wp.y;
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

    // 5. 任务完成
    if (step > 3) {
        ROS_INFO("区域%s任务全部完成", bigClass.c_str());
        return;
    }
}