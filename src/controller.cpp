#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include "kalman_filter.hpp"  // 你那个KalmanFilter头文件，记得include

class TargetController : public rclcpp::Node {
public:
    TargetController() : Node("controller"), kf_(0.1) {  // dt=0.1s，合适就行
        // 参数声明
        declare_parameter("linear_kp", 0.5);
        declare_parameter("angular_kp", 1.0);
        declare_parameter("max_linear_speed", 0.5);
        declare_parameter("max_angular_speed", 1.0);
        declare_parameter("target_distance", 0.5);
        declare_parameter("max_target_distance", 5.0); // 目标最大有效距离，超出就不动

        // 读取参数
        update_params();

        // 动态参数回调
        param_cb_handle_ = this->add_on_set_parameters_callback(
            std::bind(&TargetController::on_param_update, this, std::placeholders::_1));

        // 订阅目标3D点
        target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/target_position_3d", 10,
            std::bind(&TargetController::target_callback, this, std::placeholders::_1));

        // 速度发布
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(get_logger(), "Controller node started.");
    }

private:
    void update_params() {
        linear_kp_ = get_parameter("linear_kp").as_double();
        angular_kp_ = get_parameter("angular_kp").as_double();
        max_linear_speed_ = get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = get_parameter("max_angular_speed").as_double();
        target_distance_ = get_parameter("target_distance").as_double();
        max_target_distance_ = get_parameter("max_target_distance").as_double();
    }

    rcl_interfaces::msg::SetParametersResult on_param_update(const std::vector<rclcpp::Parameter> &params) {
        for (const auto &param : params) {
            if (param.get_name() == "linear_kp") linear_kp_ = param.as_double();
            else if (param.get_name() == "angular_kp") angular_kp_ = param.as_double();
            else if (param.get_name() == "max_linear_speed") max_linear_speed_ = param.as_double();
            else if (param.get_name() == "max_angular_speed") max_angular_speed_ = param.as_double();
            else if (param.get_name() == "target_distance") target_distance_ = param.as_double();
            else if (param.get_name() == "max_target_distance") max_target_distance_ = param.as_double();
        }
        RCLCPP_INFO(get_logger(), "参数更新：linear_kp=%.3f, angular_kp=%.3f, max_linear_speed=%.3f, max_angular_speed=%.3f, target_distance=%.3f",
                    linear_kp_, angular_kp_, max_linear_speed_, max_angular_speed_, target_distance_);
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    void target_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        geometry_msgs::msg::Twist cmd_vel;

        double raw_x = msg->point.x;
        double raw_y = msg->point.y;

        // 先滤波，平滑处理
        Eigen::Vector2d z(raw_x, raw_y);
        kf_.update(z);
        Eigen::Vector2d filtered = kf_.getState();
        double x = filtered(0);
        double y = filtered(1);

        // 异常检测：目标距离太远或者非正数，不发指令
        double distance = std::hypot(x, y);
        if (distance > max_target_distance_ || distance < 0.01) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "目标距离异常(%.2f)，停止移动", distance);
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd_vel);
            return;
        }

        // 线速度：距离误差控制
        double distance_error = distance - target_distance_;
        cmd_vel.linear.x = std::clamp(linear_kp_ * distance_error, -max_linear_speed_, max_linear_speed_);

        // 角速度：用atan2计算角度误差，更靠谱
        double angle_error = std::atan2(y, x);
        cmd_vel.angular.z = std::clamp(angular_kp_ * angle_error, -max_angular_speed_, max_angular_speed_);

        cmd_vel_pub_->publish(cmd_vel);
    }

    // 成员变量
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    double linear_kp_, angular_kp_;
    double max_linear_speed_, max_angular_speed_;
    double target_distance_;
    double max_target_distance_;

    KalmanFilter kf_;  // 你的Kalman滤波器对象

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TargetController>());
    rclcpp::shutdown();
    return 0;
}
