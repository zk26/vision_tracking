#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <algorithm>
#include <memory>

using namespace std::chrono_literals;

class Controller : public rclcpp::Node {
public:
    Controller() : Node("controller"), 
                   kp_linear_(0.2), 
                   kp_angular_(0.8),
                   safe_distance_(0.5),
                   target_lost_timeout_(2.0) {
        
        // 参数声明
        this->declare_parameter("kp_linear", 0.2);
        this->declare_parameter("kp_angular", 0.8);
        this->declare_parameter("safe_distance", 0.5);
        this->declare_parameter("target_lost_timeout", 2.0);
        this->declare_parameter("max_linear_vel", 0.5);
        this->declare_parameter("max_angular_vel", 1.0);
        
        // 获取参数
        this->get_parameter("kp_linear", kp_linear_);
        this->get_parameter("kp_angular", kp_angular_);
        this->get_parameter("safe_distance", safe_distance_);
        this->get_parameter("target_lost_timeout", target_lost_timeout_);
        this->get_parameter("max_linear_vel", max_linear_vel_);
        this->get_parameter("max_angular_vel", max_angular_vel_);
        
        // 订阅目标位置
        target_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "target_3d", 10,
            std::bind(&Controller::target_callback, this, std::placeholders::_1));
            
        // 创建控制指令发布者
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // TF2初始化
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // 定时器用于处理目标丢失
        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&Controller::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Controller node started");
    }

private:
    void target_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        last_target_time_ = this->now();
        current_target_ = *msg;
        target_available_ = true;
    }
    
    void timer_callback() {
        auto cmd = geometry_msgs::msg::Twist();
        
        // 检查目标是否超时
        if ((this->now() - last_target_time_).seconds() > target_lost_timeout_) {
            target_available_ = false;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Target lost!");
        }
        
        if (!target_available_) {
            // 停止移动
            cmd_pub_->publish(cmd);
            return;
        }
        
        try {
            // 获取最新变换
            auto transform = tf_buffer_->lookupTransform(
                "base_footprint", 
                current_target_.header.frame_id,
                current_target_.header.stamp,
                rclcpp::Duration::from_seconds(0.1));
            
            // 转换目标点到base_footprint
            geometry_msgs::msg::PointStamped transformed_point;
            tf2::doTransform(current_target_, transformed_point, transform);
            
            // 计算到目标的距离和角度
            double distance = std::hypot(transformed_point.point.x, transformed_point.point.y);
            double angle = std::atan2(transformed_point.point.y, transformed_point.point.x);
            
            // 安全距离检查
            if (distance < safe_distance_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                    "Too close to target! Distance: %.2fm", distance);
                cmd_pub_->publish(cmd);
                return;
            }
            
            // 计算控制指令
            cmd.linear.x = std::clamp(kp_linear_ * distance, 0.0, max_linear_vel_);
            cmd.angular.z = std::clamp(kp_angular_ * angle, -max_angular_vel_, max_angular_vel_);
            
            RCLCPP_DEBUG(this->get_logger(), "Control cmd: lin=%.2f, ang=%.2f, dist=%.2f, ang=%.2f",
                         cmd.linear.x, cmd.angular.z, distance, angle);
            
            // 发布控制指令
            cmd_pub_->publish(cmd);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "TF error in controller: %s", ex.what());
            cmd_pub_->publish(cmd);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Controller error: %s", e.what());
            cmd_pub_->publish(cmd);
        }
    }
    
    // 控制参数
    double kp_linear_ = 0.0, kp_angular_ = 0.0;
    double safe_distance_ = 0.0;
    double target_lost_timeout_ = 0.0;
    double max_linear_vel_ = 0.0, max_angular_vel_ = 0.0;
    
    // 状态变量
    bool target_available_ = false;
    rclcpp::Time last_target_time_;
    geometry_msgs::msg::PointStamped current_target_;
    
    // ROS2接口
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}