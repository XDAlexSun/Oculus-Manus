#include <array>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "retargeting.hpp"

class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode()
    : Node("teleop_node")
    {
        declare_parameter<std::string>("landmark_topic", "/hand/world_landmarks");
        declare_parameter<std::string>("output_topic", "/hand/joint_command");
        declare_parameter<double>("publish_rate", 50.0);
        declare_parameter<double>("smoothing_tau", 0.09);
        declare_parameter<double>("lost_timeout", 0.3);     // seconds without a hand before we react
        declare_parameter<bool>("relax_on_lost", true);     // true: ease open, false: hold last pose

        declare_parameter<std::vector<double>>("curl_open", std::vector<double>(5, 0.3));
        declare_parameter<std::vector<double>>("curl_closed", std::vector<double>(5, 3.8));
        // Joint angle (rad) at full curl, [mcp, pip, dip] per finger: thumb, index, middle, ring, pinky.
        // Keep every value inside the URDF limits (fingers 1.57, thumb 1.8). The ratios between the
        // three joints are the coupling a tendon would impose.
        declare_parameter<std::vector<double>>(
            "joint_max", {1.0, 1.2, 1.0,
                          1.4, 1.55, 1.2,
                          1.4, 1.55, 1.2,
                          1.4, 1.55, 1.2,
                          1.4, 1.55, 1.2});

        tau_ = get_parameter("smoothing_tau").as_double();
        lost_timeout_ = get_parameter("lost_timeout").as_double();
        relax_on_lost_ = get_parameter("relax_on_lost").as_bool();
        curl_open_ = get_parameter("curl_open").as_double_array();
        curl_closed_ = get_parameter("curl_closed").as_double_array();
        joint_max_ = get_parameter("joint_max").as_double_array();

        if (curl_open_.size() != 5 || curl_closed_.size() != 5 || joint_max_.size() != 15) {
            throw std::runtime_error("curl_open/curl_closed need 5 values, joint_max needs 15");
        }
        rate_ = get_parameter("publish_rate").as_double();

        sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            get_parameter("landmark_topic").as_string(), 10,
            std::bind(&TeleopNode::onLandmarks, this, std::placeholders::_1));
        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            get_parameter("output_topic").as_string(), 10);
        curl_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/hand/finger_curl", 10);

        cal_open_srv_ = create_service<std_srvs::srv::Trigger>(
            "calibrate_open",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) { calibrate(curl_open_, res); });
        cal_closed_srv_ = create_service<std_srvs::srv::Trigger>(
            "calibrate_closed",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) { calibrate(curl_closed_, res); });

        last_tick_ = now();
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_),
                                   std::bind(&TeleopNode::onTick, this));
        RCLCPP_INFO(get_logger(), "Teleop Node Started");
    }

private:
    static constexpr const char *kNames[15] = {
        "thumb_mcp", "thumb_pip", "thumb_dip",
        "index_mcp", "index_pip", "index_dip",
        "middle_mcp", "middle_pip", "middle_dip",
        "ring_mcp", "ring_pip", "ring_dip",
        "pinky_mcp", "pinky_pip", "pinky_dip"
    };

    // Callbacks only store the latest measurement; publishing happens on the fixed timer.
    void onLandmarks(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 63) return;   // 21 landmarks x 3; an empty message means "no hand"
        for (size_t f = 0; f < 5; ++f) {
            const auto a = retarget::chainAngles(msg->data.data(), retarget::kChains[f]);
            raw_[f] = a[0] + a[1] + a[2];
        }
        last_msg_ = now();
        have_msg_ = true;
    }

    void calibrate(std::vector<double> &dst, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        if (!have_msg_ || (now() - last_msg_).seconds() > lost_timeout_) {
            res->success = false;
            res->message = "No hand visible.";
            return;
        }
        for (size_t f = 0; f < 5; ++f) dst[f] = raw_[f];
        res->success = true;
        res->message = "Recorded hand pose";
    }

    void onTick()
    {
        const auto t = now();
        double dt = (t - last_tick_).seconds();
        last_tick_ = t;
        if (dt <= 0.0 || dt > 0.5) dt = 1.0 / rate_;
        const bool fresh = have_msg_ && (t - last_msg_).seconds() <= lost_timeout_;

        std::array<double, 5> curl{};
        for (size_t f = 0; f < 5; ++f) {
            double target;
            if (fresh)
                target = retarget::normalizeCurl(raw_[f], curl_open_[f], curl_closed_[f]);
            else
                target = relax_on_lost_ ? 0.0 : curl_[f];   // hand lost: open up, or hold the last pose
            curl_[f] = retarget::smoothStep(curl_[f], target, dt, tau_);
            curl[f] = curl_[f];
        }

        sensor_msgs::msg::JointState js;
        js.header.stamp = t;
        js.header.frame_id = "base_link";
        js.name.assign(kNames, kNames + 15);
        js.position.resize(15);
        for (size_t f = 0; f < 5; ++f)
            for (size_t j = 0; j < 3; ++j) js.position[3 * f + j] = curl[f] * joint_max_[3 * f + j];
        joint_pub_->publish(js);

        std_msgs::msg::Float64MultiArray cm;
        cm.data.assign(curl.begin(), curl.end());
        curl_pub_->publish(cm);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr curl_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cal_open_srv_, cal_closed_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> curl_open_, curl_closed_, joint_max_;
    std::array<double, 5> raw_{};
    std::array<double, 5> curl_{};
    double tau_{0.09}, lost_timeout_{0.3}, rate_{50.0};
    bool relax_on_lost_{true}, have_msg_{false};
    rclcpp::Time last_msg_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_tick_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}
