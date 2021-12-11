/**
 * @file ur_jacobian.cpp
 * @author Marc Carmichael, Richardo Khonasty
 * @date May 2020
 * @brief ROS node that calculate ur_robot Jacobian based on the URDF
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/console.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_kdl/tf2_kdl.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/kdl.hpp>
#include <kdl/segment.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <boost/scoped_ptr.hpp>

#include <thread>
#include <algorithm>
#include <condition_variable>

class Jacobian{
    
public:
    Jacobian(ros::NodeHandle* n);
    ~Jacobian();
    void tf_listener_thread();
private:
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    std::condition_variable cv_;
    std::mutex mtx_;
    std::atomic<bool> transform_calcd = {false};
    int num_jnt_ = 6;

    ros::NodeHandle _n;
    ros::NodeHandle _nh;
    
    ros::Subscriber _sub_joint_state;
    ros::Publisher _pub_jacobian_0;
    ros::Publisher _pub_jacobian_e;

    std::string _robot_desc;
    std::string _base_frame;
    std::string _tool_frame;

    std::string _joint_0_name;
    std::string _joint_1_name;
    std::string _joint_2_name;
    std::string _joint_3_name;
    std::string _joint_4_name;
    std::string _joint_5_name;
    
    std_msgs::Float64MultiArray jacobian_0_msg_;
    std_msgs::Float64MultiArray jacobian_e_msg_;

    KDL::Tree _kdl_tree;
    KDL::Chain _kdl_chain;
    KDL::Rotation ee_frame_rotation_;

    boost::scoped_ptr<KDL::ChainJntToJacSolver> _jacobian_solver;

};

Jacobian::~Jacobian() {
    transform_calcd = true;
    cv_.notify_all();
}

Jacobian::Jacobian(ros::NodeHandle* n):_n(*n) {
    _nh = ros::NodeHandle("robot/");

    _nh.param<std::string>("base_frame", _base_frame, "robot_arm_base");
    _nh.param<std::string>("tool_frame", _tool_frame, "robot_arm_tool0");
    
    _nh.param<std::string>("joint_0_name", _joint_0_name, "robot_arm_shoulder_pan_joint");
    _nh.param<std::string>("joint_1_name", _joint_1_name, "robot_arm_shoulder_lift_joint");
    _nh.param<std::string>("joint_2_name", _joint_2_name, "robot_arm_elbow_joint");
    _nh.param<std::string>("joint_3_name", _joint_3_name, "robot_arm_wrist_1_joint");
    _nh.param<std::string>("joint_4_name", _joint_4_name, "robot_arm_wrist_2_joint");
    _nh.param<std::string>("joint_5_name", _joint_5_name, "robot_arm_wrist_3_joint");

    _sub_joint_state = _nh.subscribe("joint_states", 1, &Jacobian::jointStateCallback, this);
    _pub_jacobian_0 = _n.advertise<std_msgs::Float64MultiArray>("jacobian_0",1);
    _pub_jacobian_e = _n.advertise<std_msgs::Float64MultiArray>("jacobian_e",1);

    _nh.param("robot_description", _robot_desc, std::string());

    if(!kdl_parser::treeFromString(_robot_desc, _kdl_tree)) {
        ROS_ERROR("KDL tree was not able constructed, shutting down the node");
        ros::shutdown();
        return;
    }
    if (!_kdl_tree.getChain(_base_frame, _tool_frame, _kdl_chain)) {
        ROS_ERROR("KDL chain was not able to be obtained, shutting down the node");
        ros::shutdown();
        return;
    }

    // Initialise the jacobian solver
    _jacobian_solver.reset(new KDL::ChainJntToJacSolver(_kdl_chain));

    // Initialise the jacobian messages
    jacobian_0_msg_.layout.dim.resize(2);
    jacobian_0_msg_.layout.dim[0].label = "rows";
    jacobian_0_msg_.layout.dim[0].size = 6;
    jacobian_0_msg_.layout.dim[0].stride = 1;
    jacobian_0_msg_.layout.dim[1].label = "columns";
    jacobian_0_msg_.layout.dim[1].size = num_jnt_;
    jacobian_0_msg_.layout.dim[1].stride = 1;

    jacobian_e_msg_.layout.dim.resize(2);
    jacobian_e_msg_.layout.dim[0].label = "rows";
    jacobian_e_msg_.layout.dim[0].size = 6;
    jacobian_e_msg_.layout.dim[0].stride = 1;
    jacobian_e_msg_.layout.dim[1].label = "columns";
    jacobian_e_msg_.layout.dim[1].size = num_jnt_;
    jacobian_e_msg_.layout.dim[1].stride = 1;
}

void Jacobian::tf_listener_thread() {
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener(tf_buffer);
    geometry_msgs::TransformStamped transform_stamped;

    ros::Rate rate(30);

    while(ros::ok()) {
        try {
            transform_stamped = tf_buffer.lookupTransform("robot_arm_tool0", "robot_arm_base", ros::Time(0));
            ee_frame_rotation_ = tf2::transformToKDL(transform_stamped).M;
            transform_calcd = true;
            cv_.notify_all();
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        rate.sleep();
    }
    // Have th callback exit out of the cv so code does not hang
    transform_calcd = true;
    cv_.notify_all();
}

void Jacobian::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    std::unique_lock<std::mutex> lck(mtx_);
    cv_.wait(lck, [&]{return transform_calcd.load() || !ros::ok();});
    transform_calcd = false;

    KDL::JntArray kdl_jnt(num_jnt_); // Create a joint array that will store joint position values

    kdl_jnt.data[0] = msg->position[std::find(msg->name.begin(), msg->name.end(), _joint_0_name) - msg->name.begin()];
    kdl_jnt.data[1] = msg->position[std::find(msg->name.begin(), msg->name.end(), _joint_1_name) - msg->name.begin()];
    kdl_jnt.data[2] = msg->position[std::find(msg->name.begin(), msg->name.end(), _joint_2_name) - msg->name.begin()];
    kdl_jnt.data[3] = msg->position[std::find(msg->name.begin(), msg->name.end(), _joint_3_name) - msg->name.begin()];
    kdl_jnt.data[4] = msg->position[std::find(msg->name.begin(), msg->name.end(), _joint_4_name) - msg->name.begin()];
    kdl_jnt.data[5] = msg->position[std::find(msg->name.begin(), msg->name.end(), _joint_5_name) - msg->name.begin()];

    KDL::Jacobian jacob_0(num_jnt_);
    KDL::Jacobian jacob_e(num_jnt_);

    _jacobian_solver->JntToJac(kdl_jnt,jacob_0); // Obtain the jacobian in robot's base frame
    KDL::changeBase(jacob_0, ee_frame_rotation_, jacob_e); // Obtain the jacobian in the EE frame of reference

    // Clear the Jacobian msg, populate it and publish
    jacobian_0_msg_.data.clear();
    jacobian_e_msg_.data.clear();

    for(int r = 0; r < 6; r++) {
        for(int c = 0; c < num_jnt_; c++) {
            jacobian_0_msg_.data.push_back(jacob_0(r,c));
            jacobian_e_msg_.data.push_back(jacob_e(r,c));
        }
    }
    _pub_jacobian_0.publish(jacobian_0_msg_);
    _pub_jacobian_e.publish(jacobian_e_msg_);
}

int main(int argc, char **argv) {
    
    /// Initialize ROS node
    ros::init(argc, argv, "jacobian");
    ros::NodeHandle n;
    std::shared_ptr<Jacobian> jacobian(new Jacobian(&n));
    std::thread t(&Jacobian::tf_listener_thread, jacobian);
    ros::spin();
    ros::shutdown();
    t.join();

    return 0;
}
