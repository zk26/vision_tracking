#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "kalman_filter.hpp"
#include <Eigen/Dense>

using namespace std::chrono_literals;

class DepthProcessor : public rclcpp::Node {
public:
    DepthProcessor() : Node("depth_processor"), kf_(0.1) {
        fx_ = 525.0;
        fy_ = 525.0;
        cx_ = 319.5;
        cy_ = 239.5;

        pixel_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "/target_position_2d", 10,
            std::bind(&DepthProcessor::pixel_callback, this, std::placeholders::_1));

        depth_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_raw", 10,
            std::bind(&DepthProcessor::depth_image_callback, this, std::placeholders::_1));

        point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/target_position_3d", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(get_logger(), "DepthProcessor节点启动");
    }

private:
    void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // 确保深度图编码是32FC1，否则报错
            if (msg->encoding != sensor_msgs::image_encodings::TYPE_32FC1) {
                RCLCPP_WARN(get_logger(), "接收到的深度图编码不是32FC1，当前是: %s", msg->encoding.c_str());
                return;
            }

            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            depth_float_ = cv_ptr->image;  // 保存深度图，单位是米（float）
            RCLCPP_INFO(this->get_logger(), "收到深度图，宽:%d 高:%d", depth_float_.cols, depth_float_.rows);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "深度图转换失败: %s", e.what());
        }
    }

    void pixel_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        if (depth_float_.empty()) {
            RCLCPP_WARN(get_logger(), "还没收到深度图，无法处理深度");
            return;
        }

        int u = static_cast<int>(msg->point.x);
        int v = static_cast<int>(msg->point.y);

        // 边界检查
        if (u < 0 || u >= depth_float_.cols || v < 0 || v >= depth_float_.rows) {
            RCLCPP_WARN(get_logger(), "目标像素点超出深度图边界: (%d, %d)", u, v);
            return;
        }

        float depth_val = depth_float_.at<float>(v, u);

        // 过滤无效深度值，注意float类型下nan和0的处理
        if (depth_val <= 0.01 || std::isnan(depth_val)) {
            RCLCPP_WARN(get_logger(), "深度值无效或太小: %f，跳过", depth_val);
            return;
        }

        // 计算3D坐标，深度值单位已经是米，无需除1000
        double x = (u - cx_) * depth_val / fx_;
        double y = (v - cy_) * depth_val / fy_;
        double z = depth_val;

        RCLCPP_INFO(get_logger(), "目标像素(%d, %d)深度值(m): %.3f，计算3D坐标: x=%.3f, y=%.3f, z=%.3f",
                    u, v, depth_val, x, y, z);

        // 卡尔曼滤波
        Eigen::Vector2d z_meas(x, y);
        kf_.update(z_meas);
        Eigen::Vector2d state = kf_.getState();

        geometry_msgs::msg::PointStamped point_msg;
        point_msg.header = msg->header;
        point_msg.point.x = state[0];
        point_msg.point.y = state[1];
        point_msg.point.z = z;

        point_pub_->publish(point_msg);

        // 发布TF
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->now();
        tf_msg.header.frame_id = "base_link";
        tf_msg.child_frame_id = "target";

        tf_msg.transform.translation.x = state[0];
        tf_msg.transform.translation.y = state[1];
        tf_msg.transform.translation.z = z;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr pixel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    KalmanFilter kf_;

    cv::Mat depth_float_;  // 存储深度图，32FC1，单位米

    double fx_, fy_, cx_, cy_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthProcessor>());
    rclcpp::shutdown();
    return 0;
}
