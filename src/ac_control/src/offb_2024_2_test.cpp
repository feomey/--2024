
// 区域任务表类型
using TaskMap = std::map<int, Waypoint>;
using AreaMap = std::map<std::string, TaskMap>;

// mode2主流程
void runMode2(const std::string& bigClass, int subTask, int& step, int& sametimes, mavros_msgs::PositionTarget& pose,
              ros::Publisher& local_pose_pub, ros::Publisher& fcu_state_pub, ros::Publisher& fcu_location_pub,
              std_msgs::Bool& fcu_state, std_msgs::String& fcu_location, const geometry_msgs::PoseStamped& local_pose,
              const std_msgs::Bool& vision_state, const AreaMap& allAreas,
              mavros_msgs::SetMode& offb_set_mode, ros::ServiceClient& set_mode_client,
              mavros_msgs::State& current_state, ros::Time& last_request) {
    // step: 0=起飞, 1=前往目标, 2=降落
    if (step == 0) {
        pose.position.x = 0.0;
        pose.position.y = 0.0;
        pose.position.z = 0.5;
        local_pose_pub.publish(pose);
        fcu_location.data = "takeoff";
        fcu_location_pub.publish(fcu_location);
        if (local_pose.pose.position.z > 0.45 && local_pose.pose.position.z < 0.55) {
            if (sametimes > 10) {
                step = 1;
                sametimes = 0;
                ROS_INFO("起飞完成，前往目标点");
            } else {
                sametimes++;
            }
        } else {
            sametimes = 0;
        }
        return;
    }

    // 查找目标点
    auto areaIt = allAreas.find(bigClass);
    if (areaIt == allAreas.end()) {
        ROS_WARN("未知区域: %s", bigClass.c_str());
        return;
    }
    auto taskIt = areaIt->second.find(subTask);
    if (taskIt == areaIt->second.end()) {
        ROS_WARN("未知子任务: %d", subTask);
        return;
    }
    const Waypoint& wp = taskIt->second;

    if (step == 1) {
        pose.position.x = wp.x;
        pose.position.y = wp.y;
        pose.position.z = wp.z;
        local_pose_pub.publish(pose);

        if (isReached(local_pose, wp)) {
            if (sametimes > 10) {
                publishStatus(fcu_state_pub, fcu_location_pub, fcu_state, fcu_location, wp.location, true);
                if (vision_state.data) {
                    step = 2;
                    sametimes = 0;
                    publishStatus(fcu_state_pub, fcu_location_pub, fcu_state, fcu_location, wp.location, false);
                    ROS_INFO("已到达%s并扫描%s，准备降落", wp.location.c_str(), wp.location.c_str());
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

    // 降落
    if (step == 2) {
        pose.position.x = wp.x;
        pose.position.y = wp.y;
        pose.position.z = 0.5; // 降落高度
        local_pose_pub.publish(pose);
        fcu_location.data = "landing";
        fcu_location_pub.publish(fcu_location);
        if (isReached(local_pose, Waypoint{wp.x, wp.y, 0.5, wp.location})) {
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
    }
}

// ================== 任务表初始化示例 ==================
TaskMap areaA = {
    {1, {1.75, -0.5, 1.4, "A1"}},
    {2, {1.25, -0.5, 1.4, "A2"}},
    {3, {0.75, -0.5, 1.4, "A3"}},
};
TaskMap areaB = {
    {1, {0.75, -1.0, 1.4, "B1"}},
    {2, {1.25, -1.0, 1.4, "B2"}},
    {3, {0.75, -1.0, 1.4, "B3"}},
};
// ...C、D区同理
AreaMap allAreas = {
    {"A", areaA},
    {"B", areaB},
    // ...
};

// ================== 主循环调用 ==================
// 只需这样调用
runMode2(bigClass, subTask, mode2_step, sametimes, pose, local_pose_pub, fcu_state_pub, fcu_location_pub,
         fcu_state, fcu_location, local_pose, vision_state, allAreas,
         offb_set_mode, set_mode_client, current_state, last_request);