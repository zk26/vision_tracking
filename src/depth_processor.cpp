#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include "vision_tracking/kalman_filter.hpp"  // 包含卡尔曼滤波器头文件

using namespace std::chrono_literals;

class DepthProcessor : public rclcpp::Node {
public:
    DepthProcessor() : Node("depth_processor"), 
                       camera_model_initialized_(false),
                       kalman_initialized_(false) {  // 添加卡尔曼初始化标志
        
        // 订阅深度图像和相机信息
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10,
            std::bind(&DepthProcessor::depth_callback, this, std::placeholders::_1));
        
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/depth/camera_info", 10,
            std::bind(&DepthProcessor::camera_info_callback, this, std::placeholders::_1));
        
        // 订阅目标在图像中的位置
        position_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "target_position", 10,
            std::bind(&DepthProcessor::position_callback, this, std::placeholders::_1));
            
        // 发布目标在3D空间中的位置
        target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("target_3d", 10);
        
        // TF2初始化
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // 初始化卡尔曼滤波器 (采样时间0.1s，过程噪声0.01，测量噪声0.1)
        kf_ = std::make_unique<KalmanFilter>(0.1, 0.01, 0.1);
        
        RCLCPP_INFO(this->get_logger(), "Depth Processor node started with Kalman filtering");
    }

private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &msg) {
        if (!camera_model_initialized_) {
            camera_model_.fromCameraInfo(msg);
            camera_model_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Camera model initialized");
        }
    }
    
    void position_callback(const geometry_msgs::msg::PointStamped::ConstSharedPtr &msg) {
        current_position_ = *msg;
        
        // 初始化卡尔曼滤波器（第一次收到位置时）
        if (!kalman_initialized_) {
            Eigen::Vector2d z0(current_position_.point.x, current_position_.point.y);
            kf_->initialize(z0);
            kalman_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Kalman filter initialized");
        }
        
        position_received_ = true;
    }
    
    void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        if (!camera_model_initialized_ || !position_received_) return;
        
        try {
            cv::Mat depth_image = cv_bridge::toCvCopy(msg)->image;
            
            // 使用卡尔曼滤波更新位置
            Eigen::Vector2d measurement(current_position_.point.x, current_position_.point.y);
            kf_->update(measurement);
            Eigen::Vector2d filtered = kf_->getState();
            
            // 获取目标位置（使用滤波后的值）
            cv::Point2d uv(filtered[0], filtered[1]);
            
            // 检查位置是否在图像范围内
            if (uv.x < 0 || uv.x >= depth_image.cols || 
                uv.y < 0 || uv.y >= depth_image.rows) {
                RCLCPP_DEBUG(this->get_logger(), "Target out of image bounds");
                return;
            }
            
            // 获取深度值（单位：米）
            float depth = depth_image.at<float>(static_cast<int>(uv.y), static_cast<int>(uv.x));
            
            if (std::isnan(depth) || depth <= 0) {
                RCLCPP_WARN(this->get_logger(), "Invalid depth value: %f", depth);
                return;
            }
            
            // 将2D点转换为3D点
            cv::Point3d ray = camera_model_.projectPixelTo3dRay(uv);
            ray.x *= depth;
            ray.y *= depth;
            ray.z = depth;
            
            // 创建3D点消息
            geometry_msgs::msg::PointStamped target_3d;
            target_3d.header = msg->header;
            target_3d.point.x = ray.x;
            target_3d.point.y = ray.y;
            target_3d.point.z = ray.z;
            
            // 转换到机器人坐标系
            geometry_msgs::msg::PointStamped transformed_point;
            try {
                auto transform = tf_buffer_->lookupTransform(
                    "base_footprint", 
                    target_3d.header.frame_id,
                    rclcpp::Time(0),
                    rclcpp::Duration::from_seconds(0.1));
                    
                tf2::doTransform(target_3d, transformed_point, transform);
                transformed_point.header.stamp = this->now();
                transformed_point.header.frame_id = "base_footprint";
                target_pub_->publish(transformed_point);
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "TF error: %s", ex.what());
            }
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }
    
    // 成员变量
    bool camera_model_initialized_ = false;
    bool position_received_ = false;
    bool kalman_initialized_ = false;  // 卡尔曼滤波器初始化标志
    image_geometry::PinholeCameraModel camera_model_;
    geometry_msgs::msg::PointStamped current_position_;
    std::unique_ptr<KalmanFilter> kf_;  // 卡尔曼滤波器实例
    
    // ROS2接口
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr position_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
    
    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DepthProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}