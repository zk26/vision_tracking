#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

using namespace std::chrono_literals;

class ImageProcessor : public rclcpp::Node {
public:
    ImageProcessor() : Node("image_processor") {
        declare_parameters();
        get_parameters();

        image_sub_ = image_transport::create_subscription(
            this, "camera/image_raw",
            std::bind(&ImageProcessor::image_callback, this, std::placeholders::_1),
            "raw");

        position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("target_position_2d", 10);
        debug_pub_ = image_transport::create_publisher(this, "debug_image");

        RCLCPP_INFO(this->get_logger(), "Image Processor node started.");
    }

private:
    void declare_parameters() {
        this->declare_parameter("h_min", 0);
        this->declare_parameter("s_min", 120);
        this->declare_parameter("v_min", 70);
        this->declare_parameter("h_max", 10);
        this->declare_parameter("s_max", 255);
        this->declare_parameter("v_max", 255);
        this->declare_parameter("min_radius", 20);
        this->declare_parameter("processing_scale", 0.5);
    }

    void get_parameters() {
        this->get_parameter("h_min", h_min_);
        this->get_parameter("s_min", s_min_);
        this->get_parameter("v_min", v_min_);
        this->get_parameter("h_max", h_max_);
        this->get_parameter("s_max", s_max_);
        this->get_parameter("v_max", v_max_);
        this->get_parameter("min_radius", min_radius_);
        this->get_parameter("processing_scale", processing_scale_);
    }

    std::string detect_shape(const std::vector<cv::Point>& contour) {
        std::string shape = "unidentified";
        double peri = cv::arcLength(contour, true);
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, 0.04 * peri, true);

        if (approx.size() == 3) {
            shape = "Triangle";
        } else if (approx.size() == 4) {
            double aspect_ratio = std::abs(cv::boundingRect(approx).width / static_cast<double>(cv::boundingRect(approx).height));
            shape = (aspect_ratio >= 0.95 && aspect_ratio <= 1.05) ? "Square" : "Rectangle";
        } else if (approx.size() > 4) {
            shape = "Circle";
        }

        return shape;
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        RCLCPP_INFO(this->get_logger(), "Received image frame, size: %dx%d", msg->width, msg->height);
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

            cv::Mat processed_frame;
            if (processing_scale_ > 0.1 && processing_scale_ < 0.99) {
                cv::resize(frame, processed_frame, cv::Size(), processing_scale_, processing_scale_);
            } else {
                processed_frame = frame.clone();
            }

            cv::Mat hsv, mask;
            cv::cvtColor(processed_frame, hsv, cv::COLOR_BGR2HSV);
            cv::GaussianBlur(hsv, hsv, cv::Size(9, 9), 2);

            cv::inRange(hsv,
                        cv::Scalar(h_min_, s_min_, v_min_),
                        cv::Scalar(h_max_, s_max_, v_max_),
                        mask);

            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            geometry_msgs::msg::PointStamped pos;
            pos.header = msg->header;
            pos.header.frame_id = "camera_link";

            if (!contours.empty()) {
                auto max_contour = *std::max_element(contours.begin(), contours.end(),
                    [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                        return cv::contourArea(a) < cv::contourArea(b);
                    });

                cv::Point2f center;
                float radius;
                cv::minEnclosingCircle(max_contour, center, radius);

                center.x /= processing_scale_;
                center.y /= processing_scale_;
                radius /= processing_scale_;

                if (radius > min_radius_) {
                    pos.point.x = center.x;
                    pos.point.y = center.y;
                    pos.point.z = radius;
                    position_pub_->publish(pos);

                    std::string shape = detect_shape(max_contour);
                    RCLCPP_INFO(this->get_logger(), "目标位置：(%.1f, %.1f), 半径：%.1f, 形状：%s",
                                center.x, center.y, radius, shape.c_str());

                    cv::circle(frame, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
                    cv::putText(frame, shape,
                                cv::Point(center.x - radius, center.y - radius - 10),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                }
            }

            // 发布调试图像
            auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
            debug_pub_.publish(debug_msg);

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in image_callback: %s", e.what());
        }
    }

    int h_min_, s_min_, v_min_;
    int h_max_, s_max_, v_max_;
    int min_radius_;
    double processing_scale_;

    image_transport::Subscriber image_sub_;
    image_transport::Publisher debug_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr position_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessor>());
    rclcpp::shutdown();
    return 0;
}
