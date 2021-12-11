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

#include <eigen3/Eigen/Dense>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/kdl.hpp>
#include <kdl/segment.hpp>

#include <boost/scoped_ptr.hpp>

#include <algorithm>

class Jacobian{
    
public:
    Jacobian(ros::NodeHandle* n);
private:
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

    ros::NodeHandle _n;
    ros::NodeHandle _nh;
    
    ros::Subscriber _sub_joint_state;
    ros::Publisher _pub_jacobian;

    std::string _robot_desc;
    std::string _base_frame;
    std::string _tool_frame;

    std::string _joint_0_name;
    std::string _joint_1_name;
    std::string _joint_2_name;
    std::string _joint_3_name;
    std::string _joint_4_name;
    std::string _joint_5_name;
    
    KDL::Tree _kdl_tree;
    KDL::Chain _kdl_chain;

    boost::scoped_ptr<KDL::ChainJntToJacSolver> _jacobian_solver;

    Eigen::MatrixXd _eig_jacobian_old;

};

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

    _sub_joint_state = _nh.subscribe("joint_states", 10, &Jacobian::jointStateCallback, this);
    _pub_jacobian = _n.advertise<std_msgs::Float64MultiArray>("jacobian_0",1);

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

    _jacobian_solver.reset(new KDL::ChainJntToJacSolver(_kdl_chain));
}

void Jacobian::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    int jnt = 6; // msg->position.size();

    KDL::JntArray kdl_jnt;
    kdl_jnt.resize(jnt);

    // for(int i = 0; i < jnt; i++) {
    //     kdl_jnt.data[i] = msg->position[i];
    // }
    
    // Couldn't use the simple direct method because the joints in the joint_states message from the driver is in a weird order
    // So to get the right order, need to find the name in the joint_states msg and use that as the index

    // ROS_INFO_STREAM_THROTTLE(0.25,"joint_0_position:"<< std::find(msg->name.begin(), msg->name.end(), _joint_0_name) - msg->name.begin());
    kdl_jnt.data[0] = msg->position[std::find(msg->name.begin(), msg->name.end(), _joint_0_name) - msg->name.begin()];
    kdl_jnt.data[1] = msg->position[std::find(msg->name.begin(), msg->name.end(), _joint_1_name) - msg->name.begin()];
    kdl_jnt.data[2] = msg->position[std::find(msg->name.begin(), msg->name.end(), _joint_2_name) - msg->name.begin()];
    kdl_jnt.data[3] = msg->position[std::find(msg->name.begin(), msg->name.end(), _joint_3_name) - msg->name.begin()];
    kdl_jnt.data[4] = msg->position[std::find(msg->name.begin(), msg->name.end(), _joint_4_name) - msg->name.begin()];
    kdl_jnt.data[5] = msg->position[std::find(msg->name.begin(), msg->name.end(), _joint_5_name) - msg->name.begin()];

    KDL::Jacobian kdl_jacobian;
    kdl_jacobian.resize(jnt);

    _jacobian_solver->JntToJac(kdl_jnt,kdl_jacobian);
    Eigen::MatrixXd eig_jacobian = kdl_jacobian.data;

    std_msgs::Float64MultiArray jacobian_msg;

    jacobian_msg.data.clear();
    jacobian_msg.layout.dim.resize(2);
    jacobian_msg.layout.dim[0].label = "rows";
    jacobian_msg.layout.dim[0].size = 6;
    jacobian_msg.layout.dim[0].stride = 1;
    jacobian_msg.layout.dim[1].label = "columns";
    jacobian_msg.layout.dim[1].size = jnt;
    jacobian_msg.layout.dim[1].stride = 1;

    for(int r = 0; r < 6; r++) {
        for(int c = 0; c < jnt; c++) {
            jacobian_msg.data.push_back(eig_jacobian(r,c));
        }
    }

    _pub_jacobian.publish(jacobian_msg);
    _eig_jacobian_old = eig_jacobian;
}

int main(int argc, char **argv) {
    
    /// Initialize ROS node
    ros::init(argc, argv, "jacobian");
    ros::NodeHandle n;
    Jacobian jacobian(&n);
    ros::spin();

    return 0;
}
