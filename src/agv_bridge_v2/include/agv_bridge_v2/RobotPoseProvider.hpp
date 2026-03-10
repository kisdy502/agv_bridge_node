#ifndef ROBOT_POSE_PROVIDER_HPP
#define ROBOT_POSE_PROVIDER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <mutex>
#include <optional>
#include "agv_bridge_v2/utils/TransformUtils.hpp"

namespace agv_bridge
{

    /**
     * @brief 提供机器人当前位姿（map 坐标系下）
     *
     * 内部订阅 /amcl_pose 话题，更新最新位姿，并提供线程安全的访问接口。
     */
    class RobotPoseProvider
    {
    public:
        using SharedPtr = std::shared_ptr<RobotPoseProvider>;

        /**
         * @brief 构造函数，初始化订阅
         * @param node ROS2 节点指针，用于创建订阅
         */
        explicit RobotPoseProvider(rclcpp::Node *node);

        // /**
        //  * @brief 获取当前位姿
        //  * @param[out] pose 返回的位姿
        //  * @return true 如果位姿已初始化，否则 false
        //  */
        // bool getCurrentPose(geometry_msgs::msg::Pose &pose) const;

        /**
         * @brief 获取当前位姿（optional 版本）
         * @return 如果位姿已初始化则返回 pose，否则返回 std::nullopt
         */
        std::optional<geometry_msgs::msg::Pose> getCurrentPose() const;

        /**
         * @brief 判断位姿是否已经初始化
         */
        bool isInitialized() const;

        /**
         * @brief 设置位置是否初始化
         */
        void setInitialized(bool isInit);

        /**
         * @brief 设置初始位置
         */
        void setCurrentPose(geometry_msgs::msg::Pose pose);

    private:
        void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

        rclcpp::Node *node_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
        mutable std::mutex mutex_;
        geometry_msgs::msg::Pose latest_pose_;
        bool initialized_;
        rclcpp::Time last_amcl_time_;  // 最后 AMCL 更新时间
        double amcl_timeout_sec_ = 3;      // AMCL 超时时间（秒）
    };

} // namespace agv_bridge

#endif // ROBOT_POSE_PROVIDER_HPP