#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
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

        point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/target_position_3d", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        last_time_ = this->now();

        RCLCPP_INFO(get_logger(), "DepthProcessor节点启动");
    }

private:
    void pixel_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // 这里你可以根据dt调整滤波器状态，比如重新构建kf，或者调用你自定义的接口
        // 但你这个kalman_filter.hpp没提供setDt，我们就先不调了

        double u = msg->point.x;
        double v = msg->point.y;

        double depth = 1.0;  // TODO: 替换成实际深度

        double x = (u - cx_) * depth / fx_;
        double y = (v - cy_) * depth / fy_;
        double z = depth;

        // 构造二维测量向量 z_meas (x, y)
        Eigen::Vector2d z_meas(x, y);

        // 先预测，再更新
        // 你的KalmanFilter只有update，没有predict？那就直接update吧
        kf_.update(z_meas);

        // 取滤波后的状态
        Eigen::Vector2d state = kf_.getState();

        geometry_msgs::msg::PointStamped point_msg;
        point_msg.header = msg->header;
        point_msg.point.x = state[0];  // 滤波后的x
        point_msg.point.y = state[1];  // 滤波后的y
        point_msg.point.z = z;         // 深度不滤波，先放着

        point_pub_->publish(point_msg);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = current_time;
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
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    KalmanFilter kf_;

    double fx_, fy_, cx_, cy_;

    rclcpp::Time last_time_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthProcessor>());
    rclcpp::shutdown();
    return 0;
}
