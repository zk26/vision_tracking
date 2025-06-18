#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class DynamicTfBroadcaster : public rclcpp::Node {
public:
    DynamicTfBroadcaster() : Node("dynamic_tf_broadcaster") {
        // 参数声明和默认值
        declare_parameter("parent_frame", "base_footprint");
        declare_parameter("child_frame", "camera_link");
        declare_parameter("x", 0.0);
        declare_parameter("y", 0.0);
        declare_parameter("z", 1.0);
        declare_parameter("roll", 0.0);
        declare_parameter("pitch", 0.0);
        declare_parameter("yaw", 0.0);

        load_parameters();

        // 动态参数回调
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&DynamicTfBroadcaster::on_parameter_event, this, std::placeholders::_1));

        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 定时器周期发布TF，频率10Hz（100ms）
        timer_ = this->create_wall_timer(
            100ms, std::bind(&DynamicTfBroadcaster::broadcast_tf, this));

        RCLCPP_INFO(get_logger(), "动态TF广播器初始化完成，父框架: %s，子框架: %s",
                    parent_frame_.c_str(), child_frame_.c_str());
    }

private:
    void load_parameters() {
        parent_frame_ = this->get_parameter("parent_frame").as_string();
        child_frame_ = this->get_parameter("child_frame").as_string();
        x_ = this->get_parameter("x").as_double();
        y_ = this->get_parameter("y").as_double();
        z_ = this->get_parameter("z").as_double();
        roll_ = this->get_parameter("roll").as_double();
        pitch_ = this->get_parameter("pitch").as_double();
        yaw_ = this->get_parameter("yaw").as_double();
    }

    rcl_interfaces::msg::SetParametersResult on_parameter_event(
        const std::vector<rclcpp::Parameter> &params) {

        bool updated = false;
        for (const auto &param : params) {
            if (param.get_name() == "parent_frame") {
                parent_frame_ = param.as_string();
                updated = true;
            } else if (param.get_name() == "child_frame") {
                child_frame_ = param.as_string();
                updated = true;
            } else if (param.get_name() == "x") {
                x_ = param.as_double();
                updated = true;
            } else if (param.get_name() == "y") {
                y_ = param.as_double();
                updated = true;
            } else if (param.get_name() == "z") {
                z_ = param.as_double();
                updated = true;
            } else if (param.get_name() == "roll") {
                roll_ = param.as_double();
                updated = true;
            } else if (param.get_name() == "pitch") {
                pitch_ = param.as_double();
                updated = true;
            } else if (param.get_name() == "yaw") {
                yaw_ = param.as_double();
                updated = true;
            }
        }

        if (updated) {
            RCLCPP_INFO(get_logger(), "参数更新：父框架=%s，子框架=%s，位置=(%.3f, %.3f, %.3f)，姿态(rpy)=(%.3f, %.3f, %.3f)",
                        parent_frame_.c_str(), child_frame_.c_str(), x_, y_, z_, roll_, pitch_, yaw_);
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "参数更新成功";
        return result;
    }

    void broadcast_tf() {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->now();
        tf_msg.header.frame_id = parent_frame_;
        tf_msg.child_frame_id = child_frame_;

        tf_msg.transform.translation.x = x_;
        tf_msg.transform.translation.y = y_;
        tf_msg.transform.translation.z = z_;

        tf2::Quaternion q;
        q.setRPY(roll_, pitch_, yaw_);
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        broadcaster_->sendTransform(tf_msg);
    }

    std::string parent_frame_, child_frame_;
    double x_, y_, z_, roll_, pitch_, yaw_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicTfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
