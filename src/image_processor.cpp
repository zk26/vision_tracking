#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

using namespace std::chrono_literals;

class ImageProcessor : public rclcpp::Node {
public:
    ImageProcessor() : Node("image_processor"), frame_width_(640), frame_height_(480) {
        // 参数声明
        this->declare_parameter("h_min", 0);
        this->declare_parameter("s_min", 120);
        this->declare_parameter("v_min", 70);
        this->declare_parameter("h_max", 10);
        this->declare_parameter("s_max", 255);
        this->declare_parameter("v_max", 255);
        this->declare_parameter("min_radius", 20);
        
        // 获取参数
        this->get_parameter("h_min", h_min_);
        this->get_parameter("s_min", s_min_);
        this->get_parameter("v_min", v_min_);
        this->get_parameter("h_max", h_max_);
        this->get_parameter("s_max", s_max_);
        this->get_parameter("v_max", v_max_);
        this->get_parameter("min_radius", min_radius_);
        
        // 订阅RGB图像话题
        image_sub_ = image_transport::create_subscription(
            this, "/camera/image_raw",
            std::bind(&ImageProcessor::image_callback, this, std::placeholders::_1),
            "raw", rmw_qos_profile_sensor_data);
        
        // 创建发布目标位置的发布者
        position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("target_position", 10);
        
        // 可视化话题
        debug_pub_ = image_transport::create_publisher(this, "debug_image");
        
        RCLCPP_INFO(this->get_logger(), "Image Processor node started");
        RCLCPP_INFO(this->get_logger(), "Color parameters: H[%d-%d], S[%d-%d], V[%d-%d], Min radius: %d", 
                   h_min_, h_max_, s_min_, s_max_, v_min_, v_max_, min_radius_);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        try {
            // 转换为OpenCV格式
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            frame_width_ = frame.cols;
            frame_height_ = frame.rows;
            
            cv::Mat hsv, mask;
            
            // 转换为HSV空间并应用模糊
            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
            cv::GaussianBlur(hsv, hsv, cv::Size(9, 9), 2);
            
            // 颜色阈值
            cv::inRange(hsv, 
                        cv::Scalar(h_min_, s_min_, v_min_), 
                        cv::Scalar(h_max_, s_max_, v_max_), 
                        mask);
            
            // 形态学操作
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
            
            // 查找轮廓
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            auto position_msg = geometry_msgs::msg::PointStamped();
            position_msg.header = msg->header;
            position_msg.header.frame_id = "camera_link";
            
            // 处理轮廓
            if (!contours.empty()) {
                // 寻找最大轮廓
                auto max_contour = *std::max_element(contours.begin(), contours.end(),
                    [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                        return cv::contourArea(a) < cv::contourArea(b);
                    });
                
                // 最小外接圆
                cv::Point2f center;
                float radius;
                cv::minEnclosingCircle(max_contour, center, radius);
                
                if (radius > min_radius_) {
                    // 发布目标中心位置
                    position_msg.point.x = center.x;
                    position_msg.point.y = center.y;
                    position_msg.point.z = radius; // 存储半径表示目标大小
                    position_pub_->publish(position_msg);
                    
                    // 可视化
                    cv::circle(frame, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
                    cv::circle(frame, center, 3, cv::Scalar(0, 0, 255), -1);
                    cv::putText(frame, "Target", 
                                cv::Point(static_cast<int>(center.x - radius), static_cast<int>(center.y - radius - 5)),
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
                }
            }
            
            // 发布调试图像
            if (debug_pub_.getNumSubscribers() > 0) {
                cv::Mat color_mask;
                cv::cvtColor(mask, color_mask, cv::COLOR_GRAY2BGR);
                cv::Mat debug_frame;
                cv::vconcat(frame, color_mask, debug_frame);
                auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_frame).toImageMsg();
                debug_pub_.publish(debug_msg);
            }
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }
    
    // 成员变量
    int h_min_ = 0, s_min_ = 0, v_min_ = 0;
    int h_max_ = 0, s_max_ = 0, v_max_ = 0;
    int min_radius_ = 0;
    int frame_width_ = 0, frame_height_ = 0;
    
    // ROS2接口
    image_transport::Subscriber image_sub_;
    image_transport::Publisher debug_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}