#include "agv_bridge_v2/NavigationManager.hpp"

#include <cmath>
using namespace agv_bridge;
namespace agv_bridge
{

    NavigationManager::NavigationManager(rclcpp::Node *node, NavigationCallbacks *callbacks, std::shared_ptr<agv_bridge::LocalizationMonitor> pose_provider)
        : node_(node), callbacks_(callbacks), pose_provider_(pose_provider), current_state_(NavigationState::IDLE), initialized_(false)
    {
        speed_limit_pub_ = node_->create_publisher<nav2_msgs::msg::SpeedLimit>("/speed_limit", 10);

        if (!node_)
        {
            throw std::invalid_argument("NavigationManager: node cannot be null");
        }
        if (!callbacks_)
        {
            throw std::invalid_argument("NavigationManager: callbacks cannot be null");
        }
    }

    NavigationManager::~NavigationManager()
    {
        // 如果正在导航，尝试取消
        if (isNavigating())
        {
            cancelNavigation();
        }
    }

    void NavigationManager::setSpeedLimit(double speed)
    {
        nav2_msgs::msg::SpeedLimit msg;
        msg.header.stamp = node_->now();
        msg.percentage = false;
        msg.speed_limit = speed;
        speed_limit_pub_->publish(msg);
        current_speed_limit_ = speed;

        RCLCPP_INFO(node_->get_logger(), "Speed limit set to: %.2f m/s", speed);
    }

    bool NavigationManager::initialize()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (initialized_)
        {
            return true;
        }

        // 创建导航动作客户端
        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");

        if (!nav_action_client_)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create navigation action client");
            return false;
        }

        // 🔴 新增：创建 Spin 客户端
        spin_client_ = rclcpp_action::create_client<Spin>(node_, "spin");
        if (!spin_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_WARN(node_->get_logger(), "Spin action server 不可用，将跳过预旋转");
        }

        // 等待动作服务器可用
        if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_WARN(node_->get_logger(), "Navigation action server not available");
            // 不返回false，因为服务器可能稍后启动
        }

        follow_path_client_ = rclcpp_action::create_client<FollowPath>(node_, "follow_path");
        if (!follow_path_client_)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create follow_path action client");
            return false;
        }
        if (!follow_path_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_WARN(node_->get_logger(), "Follow path action server not available");
        }

        initialized_ = true;
        RCLCPP_INFO(node_->get_logger(), "Navigation manager initialized");
        return true;
    }

    /***********nav2 目标点点 自由导航过去 api START***************/

    bool NavigationManager::startNavigation(const std::string &command_id, const std::string &node_id, double x, double y, double theta)
    {
        if (!initialized_)
        {
            RCLCPP_ERROR(node_->get_logger(), "Navigation manager not initialized");
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        // 创建导航目标
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.stamp = node_->now();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation = TransformUtils::yaw_to_quaternion(theta);

        // 设置回调选项
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            std::bind(&NavigationManager::goalResponseCallback, this, std::placeholders::_1);

        send_goal_options.feedback_callback = std::bind(&NavigationManager::feedbackCallback, this,
                                                        std::placeholders::_1, std::placeholders::_2);

        send_goal_options.result_callback =
            std::bind(&NavigationManager::resultCallback, this, std::placeholders::_1);

        // 发送导航目标
        auto future_goal_handle = nav_action_client_->async_send_goal(goal_msg, send_goal_options);

        setState(NavigationState::EXECUTING);

        RCLCPP_INFO(node_->get_logger(),
                    "Starting navigation to (%.2f, %.2f, %.2f) with command_id: %s",
                    x, y, theta, command_id.c_str());

        return true;
    }

    void NavigationManager::goalResponseCallback(GoalHandleNavigateToPose::SharedPtr goal_handle)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!goal_handle)
        {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
            failTask("FAILED", "到点导航请求被拒绝!");
            return;
        }

        // 保存 goal_handle 以便取消
        nav_goal_handle_ = goal_handle;
        RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");

        // 发送接受确认
        if (callbacks_)
        {
            callbacks_->onCommandAck(current_task_.value().commandId, current_task_.value().nodeId, "ACCEPTED", "准备执行自由导航到点了");
        }
    }

    void NavigationManager::feedbackCallback(GoalHandleNavigateToPose::SharedPtr,
                                             const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        if (!callbacks_)
            return;
        double yaw = TransformUtils::quaternion_to_yaw(feedback->current_pose.pose.orientation);
        callbacks_->onNavigationFeedback(feedback->current_pose.pose.position.x,
                                         feedback->current_pose.pose.position.y, yaw);
    }

    void NavigationManager::resultCallback(const GoalHandleNavigateToPose::WrappedResult &result)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        bool success = false;
        std::string status;
        std::string message;

        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            finishTask(true, "SUCCESS", "导航到点成功！");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            failTask("ABORTED", "导航终止");
            break;

        case rclcpp_action::ResultCode::CANCELED:
            failTask("CANCELED", "导航取消");
            break;

        default:
            failTask("FAILED", "导航结果未知！");
            break;
        }
    }

    /***********nav2 目标点点 自由导航过去 api END***************/

    /***********nav2 旋转 api START***************/
    // 🔴 新增：发送 Spin 命令
    bool NavigationManager::sendSpinCommand(const double spin_angle)
    {
        auto goal_msg = nav2_msgs::action::Spin::Goal();
        goal_msg.target_yaw = spin_angle; // 正数逆时针，负数顺时针
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::Spin>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&NavigationManager::spinGoalResponseCallback, this, std::placeholders::_1);
        send_goal_options.result_callback = std::bind(&NavigationManager::spinResultCallback, this, std::placeholders::_1);
        auto future_goal = spin_client_->async_send_goal(goal_msg, send_goal_options);
        return true;
    }

    // 🔴 新增：Spin 结果回调
    void NavigationManager::spinResultCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
            RCLCPP_INFO(node_->get_logger(), "旋转完成");
            auto current_pose_opt = pose_provider_->getCurrentPose(); // 获取当前位姿（旋转后的）
            if (!current_pose_opt.has_value())
            {
                RCLCPP_WARN(node_->get_logger(), "位置信息为空");
                return;
            }
            const auto &current_pose = current_pose_opt.value();
            if (current_state_ == NavigationState::PRE_ROTATING)
            {
                if (current_task_.value().edgeInfo.isBezierCurve())
                {
                    executeBezierPath(current_pose, current_task_.value());
                }
                else
                {
                    executeFollowPath(current_pose, current_task_.value());
                }
            }
            else if (current_state_ == NavigationState::END_ROTATING)
            {
                finishTask(true, "SUCCESS", "经过最终旋转后，导航到点完成!");
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "旋转完成，但是当前错误状态:%d", static_cast<int>(current_state_));
            }

            break;
        }
        case rclcpp_action::ResultCode::ABORTED:
        {
            std::string reason = (current_state_ == NavigationState::PRE_ROTATING ? "预旋转" : "到点后旋转") + std::string(",旋转被终止");
            failTask("FAILED", reason.c_str());
            break;
        }
        case rclcpp_action::ResultCode::CANCELED:
        {
            std::string reason = (current_state_ == NavigationState::PRE_ROTATING ? "预旋转" : "到点后旋转") + std::string(",旋转被取消");
            failTask("FAILED", reason.c_str());
            break;
        }
        }
    }

    // 🔴 新增：Spin 响应回调
    void NavigationManager::spinGoalResponseCallback(
        std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::Spin>> goal_handle)
    {
        if (!goal_handle)
        {
            std::string reason = (current_state_ == NavigationState::PRE_ROTATING ? "预旋转" : "到点后旋转") + std::string(",旋转目标请求被拒绝");
            failTask("FAILED", reason.c_str());
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "旋转目标已接受");
        }
    }

    /***********nav2 旋转 api END***************/

    // 直线路点导航，public放，外部调用
    bool NavigationManager::followPathNavigation(const agv_bridge::MoveToMessage moveToMessage)
    {
        current_task_ = moveToMessage;

        auto current_pose_opt = pose_provider_->getCurrentPose();
        if (!current_pose_opt.has_value())
        {
            RCLCPP_ERROR(node_->get_logger(), "机器人当前位置信息为空"); // 之前已经判断过，这里利落上不会出现为空的情况
            return false;
        }
        const auto &robot_pose = current_pose_opt.value();

        std::lock_guard<std::mutex> lock(mutex_);
        // 计算robot当前朝向 和robot和目标点连接成的直线路线的角度差

        double robot_yaw = TransformUtils::quaternion_to_yaw(robot_pose.orientation);
        double target_yaw = std::atan2(moveToMessage.y - robot_pose.position.y, moveToMessage.x - robot_pose.position.x);
        double angle_diff = angles::shortest_angular_distance(robot_yaw, target_yaw);
        RCLCPP_INFO(node_->get_logger(), "导航到点，计算夹角: 目标点(%.2f, %.2f, %.2f), 当前朝向:%.2f°, 目标朝向:%.2f°, 角度差:%.2f°",
                    moveToMessage.x, moveToMessage.y, moveToMessage.theta * 180 / M_PI,
                    robot_yaw * 180 / M_PI, target_yaw * 180 / M_PI, angle_diff * 180 / M_PI);

        if (std::abs(angle_diff) > M_PI / 4) // 角度差太小时候，旋转会导致nav2 出错
        {
            // 保存导航信息，旋转完成后继续
            if (!sendSpinCommand(angle_diff))
            {
                RCLCPP_ERROR(node_->get_logger(), "直线路点导航，发送旋转命令失败");
                return false;
            }
            setState(NavigationState::PRE_ROTATING);
            return true;
        }
        else
        {
            return executeFollowPath(robot_pose, current_task_.value());
        }
    }

    bool NavigationManager::executeFollowPath(const geometry_msgs::msg::Pose robot_pose, const agv_bridge::MoveToMessage &msg)
    {
        double robot_yaw = TransformUtils::quaternion_to_yaw(robot_pose.orientation);
        auto path = generateStraightLinePath(robot_pose, msg);

        // 创建 FollowPath 目标
        auto goal_msg = FollowPath::Goal();
        goal_msg.path.poses = path;
        goal_msg.path.header.frame_id = "map";
        goal_msg.controller_id = "FollowPath";

        setSpeedLimit(msg.edgeInfo.maxSpeed);

        auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&NavigationManager::followPathGoalResponseCallback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&NavigationManager::followPathFeedbackCallback, this,
                      std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&NavigationManager::followPathResultCallback, this, std::placeholders::_1);

        follow_path_client_->async_send_goal(goal_msg, send_goal_options);

        setState(NavigationState::EXECUTING);

        RCLCPP_INFO(node_->get_logger(),
                    "开始导航: (%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f), command_id: %s",
                    robot_pose.position.x, robot_pose.position.y, robot_yaw, msg.x, msg.y, msg.theta, msg.commandId.c_str());

        return true;
    }

    std::vector<geometry_msgs::msg::PoseStamped> NavigationManager::generateStraightLinePath(const geometry_msgs::msg::Pose robot_pose,
                                                                                             const agv_bridge::MoveToMessage &msg)
    {
        std::vector<geometry_msgs::msg::PoseStamped> path;
        double dx = msg.x - robot_pose.position.x;
        double dy = msg.y - robot_pose.position.y;
        double distance = std::hypot(dx, dy);
        double estimated_total_time = distance / msg.edgeInfo.maxSpeed * 0.75;

        // 计算点数（最少10个点，确保平滑）
        int num_points = std::max(10, static_cast<int>(distance / msg.edgeInfo.step) + 1);

        double time_step = estimated_total_time / (num_points - 1);

        // 🔴 关键：使用当前仿真时间（不要递增，不要回溯）
        rclcpp::Time current_time = node_->now();

        // 直线方向
        double edge_yaw = std::atan2(dy, dx);

        RCLCPP_INFO(node_->get_logger(),
                    "生成直线路径: 起点(%.2f,%.2f) → 终点(%.2f,%.2f), 距离=%.2fm, 点数=%d",
                    robot_pose.position.x, robot_pose.position.y, msg.x, msg.y, distance, num_points);

        // 🔴 简化：只生成直线点，所有点朝向为直线方向
        // 旋转由 Nav2 行为树处理
        for (int i = 0; i < num_points; ++i)
        {
            double ratio = static_cast<double>(i) / (num_points - 1);

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            // pose.header.stamp = current_time + rclcpp::Duration::from_seconds(i * time_step);
            // 时间戳设为0，Nav2会使用收到消息时的当前时间
            pose.header.stamp = rclcpp::Time(0);
            pose.pose.position.x = robot_pose.position.x + dx * ratio;
            pose.pose.position.y = robot_pose.position.y + dy * ratio;
            pose.pose.position.z = 0.0;
            // 所有点朝向直线方向（Nav2 会在到达后处理最终旋转）
            pose.pose.orientation = TransformUtils::yaw_to_quaternion(edge_yaw);
            path.push_back(pose);
            RCLCPP_INFO(node_->get_logger(), "路点[%d](%.2f,%.2f), stamp: %d.%09d", i, pose.pose.position.x, pose.pose.position.y,
                        pose.header.stamp.sec, pose.header.stamp.nanosec);
        }

        // 确保最后一个点位置精确
        if (!path.empty())
        {
            path.back().pose.position.x = msg.x;
            path.back().pose.position.y = msg.y;
        }

        RCLCPP_INFO(node_->get_logger(), "路径生成完成: %zu 个点", path.size());
        return path;
    }

    // 贝塞尔曲线路点导航，public放，外部调用
    bool NavigationManager::followBezierPathNavigation(const agv_bridge::MoveToMessage moveToMessage)
    {
        current_task_ = moveToMessage;

        auto current_pose_opt = pose_provider_->getCurrentPose();
        if (!current_pose_opt.has_value())
        {
            RCLCPP_DEBUG(node_->get_logger(), "机器人当前位置信息为空");
            return false;
        }

        const geometry_msgs::msg::Pose &robot_pose = current_pose_opt.value();

        double robot_yaw = TransformUtils::quaternion_to_yaw(robot_pose.orientation);

        std::lock_guard<std::mutex> lock(mutex_);

        // 获取边和已反转的控制点（如果reverse_path，外部已处理反转）
        std::vector<double> control_x, control_y;
        for (const auto &cp : moveToMessage.edgeInfo.controlPoints)
        {
            control_x.push_back(cp.x);
            control_y.push_back(cp.y);
        }
        if (moveToMessage.edgeInfo.reverse)
        {
            std::reverse(control_x.begin(), control_x.end());
            std::reverse(control_y.begin(), control_y.end());
        }

        // 计算起点切线方向（t=0）
        double target_raw = calculateBezierTangent(0.0, robot_pose.position.x, robot_pose.position.y,
                                                   control_x, control_y, moveToMessage.x, moveToMessage.y);

        double angle_diff = angles::shortest_angular_distance(robot_yaw, target_raw);

        RCLCPP_INFO(node_->get_logger(), "贝塞尔曲线起点切线方向: %.2f°, 当前朝向: %.2f°, 角度差: %.2f°",
                    target_raw * 180 / M_PI, robot_yaw * 180 / M_PI, angle_diff * 180 / M_PI);
        if (std::abs(angle_diff) > M_PI / 4)
        {
            if (!sendSpinCommand(angle_diff)) // 旋转到切线方向
            {
                RCLCPP_ERROR(node_->get_logger(), "贝塞尔曲线到点，发送旋转命令失败");
                return false;
            }
            setState(NavigationState::PRE_ROTATING);
            return true;
        }
        else
        {
            return executeBezierPath(robot_pose, current_task_.value());
        }
    }

    bool NavigationManager::executeBezierPath(const geometry_msgs::msg::Pose robot_pose, const agv_bridge::MoveToMessage moveToMessage)
    {
        auto path = generateBezierPath(robot_pose, moveToMessage); // 生成贝塞尔曲线路径
        if (path.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to generate Bezier path");
            return false;
        }
        setSpeedLimit(moveToMessage.edgeInfo.maxSpeed); // 发布速度限制

        // 创建FollowPath目标
        auto goal_msg = FollowPath::Goal();
        goal_msg.path.poses = path;
        goal_msg.path.header.frame_id = "map";
        goal_msg.path.header.stamp = node_->now();
        goal_msg.controller_id = "FollowPath";

        // 设置回调选项
        auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&NavigationManager::followPathGoalResponseCallback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&NavigationManager::followPathFeedbackCallback, this,
                      std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&NavigationManager::followPathResultCallback, this, std::placeholders::_1);

        // 发送目标
        follow_path_client_->async_send_goal(goal_msg, send_goal_options);
        setState(NavigationState::EXECUTING);
        return true;
    }

    // 计算二阶/三阶贝塞尔曲线上的点
    std::pair<double, double> NavigationManager::calculateBezierPoint(double t, double p0x, double p0y,
                                                                      const std::vector<double> &cp_x, const std::vector<double> &cp_y, double p1x, double p1y)

    {
        if (cp_x.empty() || cp_y.empty() || cp_x.size() != cp_y.size())
        {
            // 没有控制点，返回线性插值
            return {p0x + t * (p1x - p0x), p0y + t * (p1y - p0y)};
        }

        if (cp_x.size() == 1)
        {
            // 二阶贝塞尔曲线
            double u = 1.0 - t;
            double x = u * u * p0x + 2.0 * u * t * cp_x[0] + t * t * p1x;
            double y = u * u * p0y + 2.0 * u * t * cp_y[0] + t * t * p1y;
            return {x, y};
        }
        else if (cp_x.size() == 2)
        {
            // 三阶贝塞尔曲线
            double u = 1.0 - t;
            double u2 = u * u;
            double t2 = t * t;

            double x = u2 * u * p0x + 3.0 * u2 * t * cp_x[0] + 3.0 * u * t2 * cp_x[1] + t2 * t * p1x;
            double y = u2 * u * p0y + 3.0 * u2 * t * cp_y[0] + 3.0 * u * t2 * cp_y[1] + t2 * t * p1y;
            return {x, y};
        }

        // 不支持更高阶贝塞尔曲线
        return {p0x + t * (p1x - p0x), p0y + t * (p1y - p0y)};
    }

    // 计算贝塞尔曲线在t点的切线方向
    double NavigationManager::calculateBezierTangent(double t, double p0x, double p0y,
                                                     const std::vector<double> &cp_x, const std::vector<double> &cp_y, double p1x, double p1y)
    {
        if (cp_x.empty() || cp_y.empty() || cp_x.size() != cp_y.size())
        {
            // 直线
            return std::atan2(p1y - p0y, p1x - p0x);
        }

        if (cp_x.size() == 1)
        {
            // 二阶贝塞尔曲线导数
            double dx = 2.0 * (1.0 - t) * (cp_x[0] - p0x) + 2.0 * t * (p1x - cp_x[0]);
            double dy = 2.0 * (1.0 - t) * (cp_y[0] - p0y) + 2.0 * t * (p1y - cp_y[0]);
            return std::atan2(dy, dx);
        }
        else if (cp_x.size() == 2)
        {
            // 三阶贝塞尔曲线导数
            double u = 1.0 - t;
            double dx = 3.0 * u * u * (cp_x[0] - p0x) + 6.0 * u * t * (cp_x[1] - cp_x[0]) + 3.0 * t * t * (p1x - cp_x[1]);
            double dy = 3.0 * u * u * (cp_y[0] - p0y) + 6.0 * u * t * (cp_y[1] - cp_y[0]) + 3.0 * t * t * (p1y - cp_y[1]);
            return std::atan2(dy, dx);
        }

        return std::atan2(p1y - p0y, p1x - p0x);
    }

    // 估算贝塞尔曲线长度
    double NavigationManager::calculateBezierLength(double p0x, double p0y,
                                                    const std::vector<double> &cp_x, const std::vector<double> &cp_y, double p1x, double p1y, int segments)
    {
        double length = 0.0;
        double prev_x = p0x;
        double prev_y = p0y;

        for (int i = 1; i <= segments; ++i)
        {
            double t = static_cast<double>(i) / segments;
            auto [x, y] = calculateBezierPoint(t, p0x, p0y, cp_x, cp_y, p1x, p1y);

            double dx = x - prev_x;
            double dy = y - prev_y;
            length += std::hypot(dx, dy);

            prev_x = x;
            prev_y = y;
        }

        return length;
    }
    // 生成贝塞尔曲线路径
    std::vector<geometry_msgs::msg::PoseStamped> NavigationManager::generateBezierPath(
        const geometry_msgs::msg::Pose robot_pose, const agv_bridge::MoveToMessage &move_msg)
    {
        bool reverse_path = move_msg.nodeId == move_msg.edgeInfo.sourceId; // 目标点和通道起点相同，说明是reverse的，主要用途就是反转切点
        // double start_theta = TransformUtils::quaternion_to_yaw(robot_pose.orientation);
        std::vector<geometry_msgs::msg::PoseStamped> path;
        // 处理贝塞尔曲线
        std::vector<double> control_x, control_y;
        const auto &cps = move_msg.edgeInfo.controlPoints; // 已经确认有值
        for (const auto &cp : cps)
        {
            control_x.push_back(cp.x);
            control_y.push_back(cp.y);
        }

        // 如果需要反向，反转控制点
        if (reverse_path)
        {
            std::reverse(control_x.begin(), control_x.end());
            std::reverse(control_y.begin(), control_y.end());
            RCLCPP_INFO(node_->get_logger(), "已反转贝塞尔曲线控制点顺序，控制点数量: %zu", control_x.size());
        }
        // 计算贝塞尔曲线长度
        double length = calculateBezierLength(robot_pose.position.x, robot_pose.position.y, control_x, control_y, move_msg.x, move_msg.y);

        // 计算需要的点数
        int num_points = std::max(2, static_cast<int>(length / move_msg.edgeInfo.step) + 1);
        double estimated_arrival_time = length / move_msg.edgeInfo.maxSpeed * 0.7; // 预估到达时间，乘以0.7作为avg speed

        RCLCPP_INFO(node_->get_logger(), "生成贝塞尔曲线路点: 长度=%.2f, 路点数量=%d, 控制点个数=%zu, 通道速度=%.2f",
                    length, num_points, control_x.size(), move_msg.edgeInfo.maxSpeed);

        rclcpp::Time base_time = node_->now();

        for (int i = 0; i < num_points; ++i)
        {
            double t = static_cast<double>(i) / (num_points - 1);
            // 计算位置
            auto [x, y] = calculateBezierPoint(t, robot_pose.position.x, robot_pose.position.y, control_x, control_y, move_msg.x, move_msg.y);
            // 计算朝向
            double yaw = calculateBezierTangent(t, robot_pose.position.x, robot_pose.position.y, control_x, control_y, move_msg.x, move_msg.y);
            // 创建位姿
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = rclcpp::Time(0);

            pose.pose.position.x = x;
            pose.pose.position.y = y;
            // 第一个点和最后一个点设置角度，中间点间隔一个设置角度，防止频繁设置角度导致nav2 出错
            if (i == 1 || i == num_points - 1 || i % 2 == 0)
            {
                pose.pose.orientation = TransformUtils::yaw_to_quaternion(yaw);
            }

            // // 跳过过于接近的点
            // if (!path.empty())
            // {
            //     const auto &last_pose = path.back();
            //     double dist = std::hypot(x - last_pose.pose.position.x, y - last_pose.pose.position.y);
            //     if (dist < 0.01)
            //     {
            //         continue;
            //     }
            // }

            path.push_back(pose);
            RCLCPP_INFO(node_->get_logger(), "路点[%d](%.2f,%.2f), stamp: %d.%09d", i, pose.pose.position.x, pose.pose.position.y,
                        pose.header.stamp.sec, pose.header.stamp.nanosec);
            // last_x = x;
            // last_y = y;
        }

        if (!path.empty())
        {
            path.back().pose.position.x = move_msg.x;
            path.back().pose.position.y = move_msg.y;

            // 使用估算的到达时间
            // path.back().header.stamp = base_time + rclcpp::Duration::from_seconds(estimated_arrival_time);
            // RCLCPP_INFO(node_->get_logger(), "最后一个点时间戳: +%.2fs", estimated_arrival_time);
        }

        return path;
    }

    bool NavigationManager::cancelNavigation()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!isNavigating() || !nav_action_client_)
        {
            return false;
        }
        // 需要保存follow_path_goal_handle_才能取消
        return true;
    }

    void NavigationManager::pauseNavigation()
    {
        // TODO: 实现暂停逻辑（如果导航栈支持）
        RCLCPP_WARN(node_->get_logger(), "Pause navigation not implemented");
    }

    void NavigationManager::resumeNavigation()
    {
        // TODO: 实现恢复逻辑
        RCLCPP_WARN(node_->get_logger(), "Resume navigation not implemented");
    }

    NavigationState NavigationManager::getCurrentState() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return current_state_;
    }

    bool NavigationManager::isNavigating() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return current_state_ != NavigationState::IDLE && current_state_ != NavigationState::PAUSED;
    }

    // 1. 执行路点导航和贝塞尔曲线路点导航，都到这三个回调
    void NavigationManager::followPathGoalResponseCallback(const GoalHandleFollowPath::SharedPtr &goal_handle)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!goal_handle)
        {
            failTask("FAILED", "路点导航请求被拒绝!");
            return;
        }
        // 保存 goal_handle 以便取消
        follow_path_goal_handle_ = goal_handle;
        if (callbacks_)
        {
            callbacks_->onCommandAck(current_task_.value().commandId, current_task_.value().nodeId, "ACCEPTED", "准备执行导航到点了");
        }
    }

    // 2. 反馈回调
    void NavigationManager::followPathFeedbackCallback(GoalHandleFollowPath::SharedPtr,
                                                       const std::shared_ptr<const FollowPath::Feedback> feedback)
    {
        // FollowPath 反馈通常包含 distance_to_goal、speed 等，不直接提供位姿。
        // 如果你需要实时位置反馈，可以通过 TF 监听获取，或使用 Nav2 的本地规划器反馈。
        // 此处简单记录日志，不触发 onNavigationFeedback
        RCLCPP_DEBUG(node_->get_logger(), "FollowPath feedback: distance remaining: %.2f", feedback->distance_to_goal);
    }

    // 3. 结果回调
    void NavigationManager::followPathResultCallback(const GoalHandleFollowPath::WrappedResult &result)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
            auto current_pose_opt = pose_provider_->getCurrentPose();
            if (!current_pose_opt.has_value())
            {
                RCLCPP_WARN(node_->get_logger(), "最终旋转时，机器人当前位置信息为空");
            }
            const geometry_msgs::msg::Pose &robot_pose = current_pose_opt.value();
            double robot_yaw = TransformUtils::quaternion_to_yaw(robot_pose.orientation);
            double angle_diff = angles::shortest_angular_distance(robot_yaw, current_task_.value().theta);
            RCLCPP_INFO(node_->get_logger(), "followPathResultCallback,目标点方向: %.2f°, 当前朝向: %.2f°, 角度差: %.2f°",
                        current_task_.value().theta * 180 / M_PI, robot_yaw * 180 / M_PI, angle_diff * 180 / M_PI);
            if (current_task_.value().endPoint && std::abs(angle_diff) > M_PI / 40) // 4.5角度
            {
                if (!sendSpinCommand(angle_diff))
                { // 旋转到切线方向
                    failTask("FAILED", "调用nav2 spin接口，发送旋转请求失败");
                    return;
                }
                setState(NavigationState::END_ROTATING);
            }
            else
            {
                finishTask(true, "SUCCESS", "无需最终旋转，即导航到点完成!");
            }

            break;
        }
        case rclcpp_action::ResultCode::ABORTED:
            failTask("FAILED", "路点导航请求终止！");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            failTask("CANCELED", "路点导航请别取消！");
            break;
        default:
            break;
        }
        follow_path_goal_handle_.reset(); // 重置命令信息
    }

    /***其他辅助的方法 */
    void NavigationManager::setState(NavigationState new_state)
    {
        if (current_state_ != new_state)
        {
            NavigationState old_state = current_state_;
            current_state_ = new_state;

            RCLCPP_INFO(node_->get_logger(), "导航状态中%s 转换到 %s", navigationStateToString(old_state).c_str(), navigationStateToString(new_state).c_str());
            if (callbacks_)
            {
                callbacks_->onNavigationStateChanged(new_state, current_task_.value().commandId);
            }
        }
    }

    void NavigationManager::finishTask(bool success, const std::string satus, const std::string message)
    {
        if (!current_task_)
            return;

        if (callbacks_)
        {
            std::string status = success ? "SUCCESS" : "FAILED";
            callbacks_->onCommandAck(current_task_.value().commandId, current_task_.value().nodeId, status, message);
            callbacks_->onNavigationResult(success, message, current_task_.value().commandId, current_task_.value().nodeId);
        }

        setState(success ? NavigationState::COMPLETED : NavigationState::FAILED);
        current_task_.reset();
    }

    void NavigationManager::failTask(const std::string satus, const std::string message)
    {
        finishTask(false, satus, message);
    }
} // namespace agv_bridge