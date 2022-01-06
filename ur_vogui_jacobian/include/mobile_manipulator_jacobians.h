#ifndef MOBILE_MANIPULATOR_JACOBIANS_H
#define MOBILE_MANIPULATOR_JACOBIANS_H

#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>

#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_kdl/tf2_kdl.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/kdl.hpp>
#include <kdl/segment.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/frames.hpp>

#include <boost/scoped_ptr.hpp>

class Jacobian {

public:
    Jacobian();
    ~Jacobian();

    struct JointPosition {
        std::mutex mtx;
        KDL::JntArray jnts;
    };
    typedef std::unique_lock<std::mutex> uniq_lck;

private:
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void publishJacobians();
    void tf_listener_thread();

    int num_jnts_;

    std::vector<std::thread> threads;
    std::string _robot_desc;
    std::vector<std::string> joint_names_ =
        {
        "robot_arm_shoulder_pan_joint",
        "robot_arm_shoulder_lift_joint",
        "robot_arm_elbow_joint",
        "robot_arm_wrist_1_joint",
        "robot_arm_wrist_2_joint",
        "robot_arm_wrist_3_joint",
        };

    ros::NodeHandle _nh;
    ros::Subscriber sub_joint_state_, sub_vogui_odom_;
    ros::Publisher pub_jacobian_, pub_jacobian_e_;

    std_msgs::Float64MultiArray jacobian_msg_0_;
    std_msgs::Float64MultiArray jacobian_msg_ee_;

    KDL::Chain _kdl_chain;
    KDL::Rotation frame_rotation_;

    JointPosition joint_positions_;

    boost::scoped_ptr<KDL::ChainJntToJacSolver> _chain_jacobian_solver;

};

#endif // MOBILE_MANIPULATOR_JACOBIANS_H
