// include/bridge/LocalizationMonitor.hpp
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <nav2_msgs/msg/particle_cloud.hpp>
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <mutex>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <optional>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>   // 提供 tf2::fromMsg
#include <tf2/utils.h>                                // 提供 tf2::getYaw

namespace agv_bridge
{
    class LocalizationMonitor
    {
    public:
        struct Config
        {
            double particle_spread_threshold = 1.0;
            int required_stable_count = 3;
            double quality_threshold = 0.8;
        };

        LocalizationMonitor(rclcpp::Node *parent_node, const Config &config);

        void update();
        bool isParticleInitialized() const;
        bool isInitialized() const;
        bool getTransform(
            const std::string &frame,
            const rclcpp::Time &time, // 关键：使用消息时间戳
            geometry_msgs::msg::TransformStamped &transform);
        // bool getRobotPose(geometry_msgs::msg::Pose &pose);
        std::optional<geometry_msgs::msg::Pose> getCurrentPose() const;

    private:
        void particleCallback(nav2_msgs::msg::ParticleCloud::SharedPtr msg);
        void diagnosticCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
        void checkTFStability();
        void timerCallback();

        rclcpp::Node *parent_node_;
        Config config_;

        bool is_initialized_;
        bool tf_initialized_;
        bool particle_initialized_;
        bool diagnostic_initialized_;

        int tf_stable_count_;

        // ROS2组件
        rclcpp::Subscription<nav2_msgs::msg::ParticleCloud>::SharedPtr particle_sub_;
        // rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_sub_;

        // TF2组件
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        rclcpp::Time lastCheckTime;
        rclcpp::TimerBase::SharedPtr timer_;

        geometry_msgs::msg::Pose latest_pose_;
        double continuous_yaw_; // 累积的连续偏航角（弧度）
        double last_raw_yaw_;   // 上一次的原始偏航角（用于差值计算）
        bool have_previous_;    // 是否有上一帧
        mutable std::mutex mutex_;
    };

}