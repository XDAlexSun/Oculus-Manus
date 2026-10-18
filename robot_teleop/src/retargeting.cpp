#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


using Vec3 = Eigen::Vector3d;
using VecX = Eigen::Vector3d;
using MatX = Eigen::MatrixXd;


struct Joint {
    Vec3 offset; // origin of joint w.r.t previous joint
    Vec3 axis; // unit rotation axis (same as parent and joint frame)
    double lower;   // joint limits in radians
    double upper;   // due to biological limit
}

class Retargeting : public rclcpp::Node
{
public:  
    Retargeting()
    : Node("regargeting")
    {
        landmark_sub = 
            this->create_subscription<std_msgs::msg::Float64MultiArray>("/hand/landmarks", 10, std::bind(&Retargeting::landmarkCallback, this, std::placeholders::_1));
            // queue size of 10 binding to landmark callback
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10); //publish a queue of 10 to send out joint angles
        previous_joints_.assign(15, 0.0);   //initialize 15 joints as 0.0 -> CHANGE in future if needed
        RCLCPP_INFO(this->get_logger(), "Retargeting Node Started");
    
    }
int dof() const { return static_cast<int>(joints.size())}

    struct Kin {
        std::vector<Vec3> joint_pos;    // pos of each joint origin from mediapipe
        std::vector<Vec3> joint_axis;   // dir of each joint axis from MP
        Vec3 tip;                       // pos of each finger tip from MP
    }

private:
    rclcpp:Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr landmark_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    std::vector<double> previous_joints_;
    double alpha_ = 0.7;

    // IK math here
    // each ith coordinate is p_i = [x3_i, y3_i+1, z3_i+2]

    //basic vector functions:
    std::vector<double> vec(const std::vector<double>& a, const std::vector<double>& b)
    {
        return {b[0] - a[0], b[1] - a[1], b[2] - a[2]}; //coordinates 
    }

    double dot(const std::vector<double>& a, const std::vector<double>& b)
    {
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    }

    double norm(const std::vector<double>& v)
    {
        return std::sqrt(dot(v, v));
    }

    // finger movements are rotational along the MCP, PIP, and DIP joints (linear for small movements)
    // dls is optimal
}