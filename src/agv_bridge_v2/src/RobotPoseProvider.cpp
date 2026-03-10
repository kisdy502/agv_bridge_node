#include "agv_bridge_v2/RobotPoseProvider.hpp"

namespace agv_bridge
{
    RobotPoseProvider::RobotPoseProvider(rclcpp::Node *node)
        : node_(node), initialized_(false)
    {
        // 订阅 AMCL 位姿话题
        amcl_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", rclcpp::QoS(10),
                                                                                                   std::bind(&RobotPoseProvider::amclPoseCallback, this, std::placeholders::_1));
    }

    void RobotPoseProvider::amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_pose_ = msg->pose.pose;
        last_amcl_time_ = msg->header.stamp;
        initialized_ = true;
    }

    std::optional<geometry_msgs::msg::Pose> RobotPoseProvider::getCurrentPose() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_)
            return std::nullopt;
        return latest_pose_;
    }

    bool RobotPoseProvider::isInitialized() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return initialized_;
    }

    void RobotPoseProvider::setInitialized(bool isInit)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        initialized_ = isInit;
    }

    void RobotPoseProvider::setCurrentPose(geometry_msgs::msg::Pose pose)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // 检查 AMCL 是否在最近时间内有更新
        if (last_amcl_time_.seconds() > 0) // 如果收到过 AMCL 数据
        {
            rclcpp::Time now = node_->now();
            rclcpp::Duration time_since_amcl = now - last_amcl_time_;

            if (time_since_amcl.seconds() < amcl_timeout_sec_)
            {
                // AMCL 在超时时间内有更新，忽略外部设置
                RCLCPP_WARN(node_->get_logger(),
                            "忽略外部位置设置，AMCL 在 %.1f 秒前有更新 (%.1f < %.1f 秒)",
                            time_since_amcl.seconds(),
                            time_since_amcl.seconds(),
                            amcl_timeout_sec_);
            }
        }

        // 更新位置
        latest_pose_ = pose;
        initialized_ = true;

        RCLCPP_INFO(node_->get_logger(),
                    "设置外部位置: (%.3f, %.3f), 朝向: %.1f°",
                    pose.position.x,
                    pose.position.y,
                    agv_bridge::TransformUtils::quaternion_to_yaw(pose.orientation) * 180.0 / M_PI);

    }

} // namespace agv_bridge