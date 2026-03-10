#include "agv_bridge_v2/LocalizationMonitor.hpp"
#include <tf2/utils.h>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
namespace agv_bridge
{

    LocalizationMonitor::LocalizationMonitor(rclcpp::Node *parent_node, const Config &config)
        : parent_node_(parent_node),
          config_(config),
          is_initialized_(false),
          tf_initialized_(false),
          particle_initialized_(false),
          diagnostic_initialized_(false),
          tf_stable_count_(0),
          tf_buffer_(parent_node_->get_clock()),
          tf_listener_(tf_buffer_)
    {

        if (!parent_node_)
        {
            throw std::invalid_argument("Parent node cannot be null");
        }

        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.best_effort();

        // 修改订阅器类型
        particle_sub_ = parent_node_->create_subscription<nav2_msgs::msg::ParticleCloud>(
            "/particle_cloud", qos,
            [this](const nav2_msgs::msg::ParticleCloud::SharedPtr msg)
            {
                particleCallback(msg);
            });

        // 10Hz 定时器
        timer_ = parent_node->create_wall_timer(std::chrono::milliseconds(100),
                                                std::bind(&LocalizationMonitor::timerCallback, this));

        // diagnostic_sub_ = parent_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        //     "/diagnostics", qos,
        //     [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
        //     {
        //         diagnosticCallback(msg);
        //     });
        lastCheckTime = parent_node_->now();
        RCLCPP_INFO(parent_node_->get_logger(),
                    "LocalizationMonitor initialized with threshold: %.2f m",
                    config_.particle_spread_threshold);
    }

    void LocalizationMonitor::timerCallback()
    {
        try
        {
            // 查找最新变换（时间戳设为 0 表示获取最新可用变换）
            auto transform = tf_buffer_.lookupTransform(
                "map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.1));

            geometry_msgs::msg::Pose pose;
            pose.position.x = transform.transform.translation.x;
            pose.position.y = transform.transform.translation.y;
            pose.position.z = transform.transform.translation.z;
            pose.orientation = transform.transform.rotation;

            // 角度连续性处理
            double raw_yaw = tf2::getYaw(pose.orientation); // 范围 [-π, π]

            std::lock_guard<std::mutex> lock(mutex_);
            if (!have_previous_)
            {
                // 第一帧：初始化连续偏航角为原始值
                continuous_yaw_ = raw_yaw;
                have_previous_ = true;
            }
            else
            {
                // 计算差值，处理环绕
                double diff = raw_yaw - last_raw_yaw_;
                if (diff > M_PI)
                    diff -= 2 * M_PI;
                else if (diff < -M_PI)
                    diff += 2 * M_PI;

                continuous_yaw_ += diff; // 累积连续角度
            }
            last_raw_yaw_ = raw_yaw;
            latest_pose_ = pose; // 保存原始位姿（含四元数）
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(parent_node_->get_logger(), "TF lookup failed: %s", ex.what());
        }
    }

    void LocalizationMonitor::update()
    {
        checkTFStability();

        // 综合判断初始化状态
        // bool was_initialized = is_initialized_;
        // 使用三种检测方法的逻辑OR
        is_initialized_ = tf_initialized_ || particle_initialized_;
    }

    bool LocalizationMonitor::isInitialized() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return is_initialized_;
    }

    bool LocalizationMonitor::isParticleInitialized() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return particle_initialized_;
    }

    // 修改回调函数签名
    void LocalizationMonitor::particleCallback(const nav2_msgs::msg::ParticleCloud::SharedPtr msg)
    {
        if (particle_initialized_)
        {
            return; // 已经初始化了就不继续检查了
        }
        if (msg->particles.empty())
        {
            return;
        }

        // 计算粒子分布的标准差
        double sum_x = 0.0, sum_y = 0.0;
        double sum_x2 = 0.0, sum_y2 = 0.0;
        size_t count = msg->particles.size();

        for (const auto &particle : msg->particles)
        {
            // Nav2 ParticleCloud 中的粒子有 pose 字段
            sum_x += particle.pose.position.x;
            sum_y += particle.pose.position.y;
            sum_x2 += particle.pose.position.x * particle.pose.position.x;
            sum_y2 += particle.pose.position.y * particle.pose.position.y;
        }

        double mean_x = sum_x / count;
        double mean_y = sum_y / count;
        double variance_x = (sum_x2 / count) - (mean_x * mean_x);
        double variance_y = (sum_y2 / count) - (mean_y * mean_y);
        double spread = std::sqrt(variance_x + variance_y);

        RCLCPP_DEBUG(parent_node_->get_logger(),
                     "Particle spread: %.3f m (threshold: %.3f m)",
                     spread, config_.particle_spread_threshold);

        std::lock_guard<std::mutex> lock(mutex_);
        particle_initialized_ = (spread < config_.particle_spread_threshold);
    }

    // void LocalizationMonitor::diagnosticCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
    // {
    //     if (diagnostic_initialized_)
    //     {
    //         return; // 已经初始化了就不继续检查了
    //     }
    //     std::lock_guard<std::mutex> lock(mutex_);
    //     diagnostic_initialized_ = false;

    //     // 检查Nav2相关的诊断状态
    //     for (const auto &status : msg->status)
    //     {
    //         // 检查是否包含Nav2本地化相关的状态
    //         if (status.name.find("lifecycle_manager_localization") != std::string::npos ||
    //             status.name.find("amcl") != std::string::npos ||
    //             status.name.find("localization") != std::string::npos)
    //         {

    //             // 检查Nav2是否active
    //             if (status.message.find("active") != std::string::npos ||
    //                 status.message.find("ACTIVE") != std::string::npos)
    //             {
    //                 // 如果看到active状态，认为定位可能已经初始化
    //                 diagnostic_initialized_ = true;
    //                 RCLCPP_DEBUG(parent_node_->get_logger(),
    //                              "Nav2 localization is active: %s", status.message.c_str());
    //                 return;
    //             }

    //             // 如果还有values，继续检查quality
    //             for (const auto &value : status.values)
    //             {
    //                 if (value.key.find("quality") != std::string::npos ||
    //                     value.key.find("spread") != std::string::npos)
    //                 {
    //                     try
    //                     {
    //                         double quality = std::stod(value.value);
    //                         if (quality > config_.quality_threshold)
    //                         {
    //                             diagnostic_initialized_ = true;
    //                             RCLCPP_DEBUG(parent_node_->get_logger(),
    //                                          "Localization quality: %.3f (threshold: %.3f)",
    //                                          quality, config_.quality_threshold);
    //                         }
    //                     }
    //                     catch (const std::exception &e)
    //                     {
    //                         RCLCPP_WARN(parent_node_->get_logger(),
    //                                     "Failed to parse diagnostic value: %s", value.value.c_str());
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }

    void LocalizationMonitor::checkTFStability()
    {
        auto current_time = parent_node_->now();
        auto elapsed = (current_time - lastCheckTime).seconds();
        try
        {
            // 先查看有哪些可用的TF
            std::string tf_string = tf_buffer_.allFramesAsString();
            if (elapsed > 15)
            {
                RCLCPP_INFO(parent_node_->get_logger(), "Available TF frames:\n%s", tf_string.c_str());
            }

            // 检查是否可以查看到map和base_footprint
            if (!tf_buffer_.canTransform("map", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(0.1)))
            {
                if (elapsed > 15)
                {
                    RCLCPP_WARN(parent_node_->get_logger(), "Cannot transform from map to base_footprint yet");
                }
                throw tf2::TransformException("Transform not available");
            }

            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                "map", "base_footprint", tf2::TimePointZero,
                tf2::durationFromSec(0.1));
            if (elapsed > 15)
            {
                RCLCPP_INFO(parent_node_->get_logger(), "Got transform: timestamp=%d,%d",
                            transform.header.stamp.sec, transform.header.stamp.nanosec); // 临时添加
            }

            // 检查变换的时间戳
            rclcpp::Time transform_time(transform.header.stamp);

            rclcpp::Time now = parent_node_->now(); // 注意：如果设置了use_sim_time，这个也是仿真时间

            double time_diff = std::abs(transform_time.seconds() - now.seconds()); // 取绝对值
            if (elapsed > 15)
            {
                RCLCPP_INFO(parent_node_->get_logger(),
                            "Timestamp check - transform: %.3f, now: %.3f, diff: %.3f",
                            transform_time.seconds(), now.seconds(), time_diff);
            }

            std::lock_guard<std::mutex> lock(mutex_);

            // 如果TF变换稳定且时间戳新鲜
            if (time_diff < 2.0)
            { // 2秒内有效
                tf_stable_count_++;
                tf_initialized_ = true;
                if (elapsed > 15)
                {
                    RCLCPP_INFO(parent_node_->get_logger(),
                                "TF stable count: %d (good), tf_initialized = %s",
                                tf_stable_count_,
                                tf_initialized_ ? "true" : "false");
                }
            }
            else
            {
                tf_stable_count_ = 0;
                tf_initialized_ = false;
            }
            if (elapsed > 15)
            {
                lastCheckTime = current_time;
            }
        }
        catch (const tf2::TransformException &ex)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            tf_stable_count_ = 0;
            tf_initialized_ = false;
            if (elapsed > 15)
            {
                lastCheckTime = current_time;
                RCLCPP_WARN(parent_node_->get_logger(), // 改成WARN级别
                            "TF transform unavailable: %s", ex.what());
            }
        }
    }

    bool LocalizationMonitor::getTransform(
        const std::string &frame,
        const rclcpp::Time &time, // 关键：使用消息时间戳
        geometry_msgs::msg::TransformStamped &transform)
    {
        // // 1. 查询TF树
        try
        {
            transform = tf_buffer_.lookupTransform(
                "map",                      // 目标坐标系（如map或odom）
                frame,                      // 源坐标系（laser_link）
                time,                       // 使用指定时间戳
                tf2::durationFromSec(0.1)); // 容差时间
            return true;
        }
        catch (const tf2::TransformException &ex)
        {

            return false;
        }
    }

    bool LocalizationMonitor::getRobotPose(geometry_msgs::msg::Pose &pose)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!is_initialized_)
        {
            return false;
        }
        geometry_msgs::msg::TransformStamped transform;
        // 获取从 map 到 base_footprint（或 base_link）的最新变换
        if (getTransform("base_footprint", rclcpp::Time(0), transform))
        {
            pose.position.x = transform.transform.translation.x;
            pose.position.y = transform.transform.translation.y;
            pose.orientation = transform.transform.rotation;
            return true;
        }
        return false;
    }

    std::optional<geometry_msgs::msg::Pose> LocalizationMonitor::getCurrentPose() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!have_previous_)
            return std::nullopt;
        return latest_pose_;
    }

}