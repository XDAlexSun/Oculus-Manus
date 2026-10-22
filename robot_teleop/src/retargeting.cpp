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
    int dof() const { return static_cast<int>(joints.size())}   // necessary for many fkin and bkin eqns
    const Joint& joint(int i) const { return joints[i];}

    struct Kin {
        std::vector<Vec3> joint_pos;    // pos of each joint origin from mediapipe
        std::vector<Vec3> joint_axis;   // dir of each joint axis from MP
        Vec3 tip;                       // pos of each finger tip from MP
    }

    // calculate tip position
    Kin forward(const VecX& q) const {
        check_size(q);
        Kin k;
        k.joint_pos.reserve(joints.size());
        k.joint_axis.reserve(joints.size());

        Eigen::Matrix3d R = Eigen::Identity();
        Vec3 p = Vec3::Zero();
        for(int i = 0; i < dof(); ++i){
            p += R * joints_[i].offset;
            k.joint_pos.push_back(p);
            k.joint_axis.push_back(R * joints_[i].axis);
            R = R * Eigen::AngleAxisd(q[i], joints_[i].axis).toRotationMatrix();
        }
        k.top = p + R * tip_offset_;
        return k;
    }

    Vec3 tip_position(const VecX& q) const { return forward(q).tip; }

    // position jacobian, 3 x n. 
        // column i: joint i rotates, which way finger tip moves
        // J[:, i] = z_i * (p_tip - p_i)

    MatX jacobian(const VecX& q) const {
        const Kin k = forward(q);
        MatX J(3, dof());
        for(int i = 0; i < dof(); ++i){
            J.col(i) = k.joint_axis[i].cross(k.top - k.joint_pos[i]);
        }
        return J;
    }

    VecX clamp_to_limits(VecX q) const {
        for(int i = 0; i < dof(); ++i){
            q[i] = std::min(std::max(q[i], joints[i].lower), joints[i].upper);
        }
        return q;
    }


private:
    rclcpp:Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr landmark_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    std::vector<double> previous_joints_;
    double alpha_ = 0.7;

    // IK math here
    // each ith coordinate is p_i = [x3_i, y3_i+1, z3_i+2]

    void check_size(const VecX& q) const {
        if (q.size() != dof()) throw std::invalid_argument("q has wrong size");
    }

    // finger movements are rotational along the MCP, PIP, and DIP joints (linear for small movements)
    // dls is optimal
};

struct DLSparams {
    double lambda = 0.02;
    double tolerance = 1e-4;
    int max_iterations = 100;
    double max_task_step = 0.02;
}

struct DLSresult {
    VecX q;             // solution/best effort
    double error;       // final distance tip to target
    int iterations;
    bool converged;     // edge cases
}

// one iteration of dls is:
// d(theta) = J^T (J + lambda^2 * I)^-1 J^T*e
// e is error vector (x_target - x_current)
inline VecX dls_step(const Mat& J, const Vec3& e, double lambda) {
    const Eigen::Matrix3d A = J * J.transpose() + lambda * lambda * Eigen::Matrix3d::Identity();
    return J.transpose() * A.ldlt().solve(e);
}   // this iterates 'til tip reaches target

//DLS solver