#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <algorithm>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>

using namespace std::chrono_literals;

class Controller : public rclcpp::Node {
public:
    Controller() : Node("controller") {
        // 声明参数
        declare_parameter("kp_linear", 0.3);
        declare_parameter("kp_angular", 0.8);
        declare_parameter("safe_distance", 0.5);
        declare_parameter("target_lost_timeout", 2.0);
        declare_parameter("max_linear_vel", 0.5);
        declare_parameter("max_angular_vel", 1.0);
        
        // 获取参数
        kp_linear_ = get_parameter("kp_linear").as_double();
        kp_angular_ = get_parameter("kp_angular").as_double();
        safe_distance_ = get_parameter("safe_distance").as_double();
        target_lost_timeout_ = get_parameter("target_lost_timeout").as_double();
        max_linear_vel_ = get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = get_parameter("max_angular_vel").as_double();
        bool use_sim_time = get_parameter("use_sim_time").as_bool();
        
        // 设置使用仿真时间
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
        parameters_client->set_parameters({
            rclcpp::Parameter("use_sim_time", use_sim_time)
        });
        
        // 打印参数
        RCLCPP_INFO(get_logger(), "Controller parameters:");
        RCLCPP_INFO(get_logger(), "  kp_linear: %.2f", kp_linear_);
        RCLCPP_INFO(get_logger(), "  kp_angular: %.2f", kp_angular_);
        RCLCPP_INFO(get_logger(), "  safe_distance: %.2f", safe_distance_);
        RCLCPP_INFO(get_logger(), "  target_lost_timeout: %.2f", target_lost_timeout_);
        RCLCPP_INFO(get_logger(), "  max_linear_vel: %.2f", max_linear_vel_);
        RCLCPP_INFO(get_logger(), "  max_angular_vel: %.2f", max_angular_vel_);
        RCLCPP_INFO(get_logger(), "  use_sim_time: %s", use_sim_time ? "true" : "false");
        
        // 订阅目标位置
        target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "target_3d", 10, std::bind(&Controller::target_callback, this, std::placeholders::_1));
        
        // 创建控制指令发布者
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // TF2初始化
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // 定时器用于处理目标丢失
        timer_ = create_wall_timer(100ms, std::bind(&Controller::timer_callback, this));
        
        RCLCPP_INFO(get_logger(), "Controller node successfully initialized");
    }

private:
    void target_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        last_target_time_ = get_clock()->now();
        current_target_ = *msg;
        target_available_ = true;
        
        RCLCPP_DEBUG(get_logger(), "Target received: (%.2f, %.2f, %.2f)",
                    msg->point.x, msg->point.y, msg->point.z);
    }
    
    void timer_callback() {
        geometry_msgs::msg::Twist cmd;
        auto current_time = get_clock()->now();
        
        // 处理时间源一致性
        if (current_time.get_clock_type() == last_target_time_.get_clock_type()) {
            auto duration = current_time - last_target_time_;
            
            // 目标丢失处理
            if (duration.seconds() > target_lost_timeout_) {
                if (target_available_) {
                    RCLCPP_WARN(get_logger(), "Target lost! Stopping robot.");
                    target_available_ = false;
                }
                cmd_pub_->publish(cmd);
                return;
            }
        } else {
            RCLCPP_WARN(get_logger(), "Time source mismatch! Resetting.");
            last_target_time_ = current_time;
            cmd_pub_->publish(cmd);
            return;
        }
        
        if (!target_available_) {
            cmd_pub_->publish(cmd);
            return;
        }
        
        try {
            // 获取变换
            auto transform = tf_buffer_->lookupTransform(
                "base_footprint", 
                current_target_.header.frame_id,
                tf2::TimePointZero,
                100ms);
            
            // 转换目标点
            geometry_msgs::msg::PointStamped transformed_point;
            tf2::doTransform(current_target_, transformed_point, transform);
            
            // 计算到目标的距离和角度
            double dx = transformed_point.point.x;
            double dy = transformed_point.point.y;
            double distance = std::hypot(dx, dy);
            double angle = std::atan2(dy, dx);
            
            // 安全距离检查
            if (distance < safe_distance_) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                                    "Too close to target (%.2fm < %.2fm)! Stopping.",
                                    distance, safe_distance_);
                cmd_pub_->publish(cmd);
                return;
            }
            
            // 计算控制指令
            cmd.linear.x = std::clamp(kp_linear_ * distance, 0.0, max_linear_vel_);
            cmd.angular.z = std::clamp(kp_angular_ * angle, -max_angular_vel_, max_angular_vel_);
            
            RCLCPP_DEBUG(get_logger(), "Control command: linear=%.2f, angular=%.2f",
                         cmd.linear.x, cmd.angular.z);
            
            // 发布控制指令
            cmd_pub_->publish(cmd);
        }
        catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "Transform error: %s", ex.what());
        }
        catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Error in controller: %s", e.what());
        }
    }
    
    // 控制参数
    double kp_linear_;
    double kp_angular_;
    double safe_distance_;
    double target_lost_timeout_;
    double max_linear_vel_;
    double max_angular_vel_;
    
    // 状态
    bool target_available_ = false;
    rclcpp::Time last_target_time_;
    geometry_msgs::msg::PointStamped current_target_;
    
    // ROS接口
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}