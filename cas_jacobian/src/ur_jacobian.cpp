/**
 * @file ur_jacobian.cpp
 * @author Marc Carmichael, Richardo Khonasty
 * @date May 2020
 * @brief ROS node that calculate ur_robot Jacobian based on the URDF
 */

#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/console.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_kdl/tf2_kdl.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/treejnttojacsolver.hpp>
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
    std::vector<std::thread> threads;
    std::atomic<bool> transform_calcd = {false};

    ros::NodeHandle _n;
    ros::NodeHandle _nh;
    
    ros::Subscriber _sub_joint_state;
    ros::Publisher _pub_jacobian;

    std::string _robot_desc;

    struct MobileBaseFrames {
        std::string front_right;
        std::string front_left;
        std::string back_right;
        std::string back_left;
    } mobile_base_frames_;

    struct MobileToolFrames {
        std::string front_right;
        std::string front_left;
        std::string back_right;
        std::string back_left;
    } mobile_tool_frames_;

    /*   14 joints in total. Joint names:
     robot_arm_shoulder_pan_joint
     robot_arm_shoulder_lift_joint
     robot_arm_elbow_joint
     robot_arm_wrist_1_joint
     robot_arm_wrist_2_joint
     robot_arm_wrist_3_joint
     robot_back_left_motor_wheel_joint
     robot_back_left_wheel_joint
     robot_back_right_motor_wheel_joint
     robot_back_right_wheel_joint
     robot_front_left_motor_wheel_joint
     robot_front_left_wheel_joint
     robot_front_right_motor_wheel_joint
     robot_front_right_wheel_joint
*/
    std::vector<std::string> joint_names_ = {
        "robot_arm_shoulder_pan_joint",
        "robot_arm_shoulder_lift_joint",
        "robot_arm_elbow_joint",
        "robot_arm_wrist_1_joint",
        "robot_arm_wrist_2_joint",
        "robot_arm_wrist_3_joint",
        "robot_back_left_motor_wheel_joint",
        "robot_back_left_wheel_joint",
        "robot_back_right_motor_wheel_joint",
        "robot_back_right_wheel_joint",
        "robot_front_left_motor_wheel_joint",
        "robot_front_left_wheel_joint",
        "robot_front_right_motor_wheel_joint",
        "robot_front_right_wheel_joint"
    };
    
    KDL::Tree _kdl_tree;
    KDL::Chain _kdl_chain;
    KDL::Rotation frame_rotation_;

    boost::scoped_ptr<KDL::TreeJntToJacSolver> _jacobian_solver;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> _chain_jacobian_solver;

};

Jacobian::~Jacobian(){
    std::cout << "destroying class" << std::endl;
    transform_calcd = true;
    cv_.notify_all();
    for (auto &t : threads) {
        t.join();
        std::cout << "joining threads" << std::endl;
    }
}

Jacobian::Jacobian(ros::NodeHandle* n):_n(*n) {
    _nh = ros::NodeHandle("robot/");

//    _nh.param<std::string>("base_frame", mobile_base_frames_.front_right, "robot_front_right_wheel");
//    _nh.param<std::string>("tool_frame", mobile_tool_frames_.front_right, "robot_arm_tool0");
    _nh.param<std::string>("base_frame", mobile_base_frames_.front_right, "robot_base_footprint"); //robot_arm_base
    _nh.param<std::string>("tool_frame", mobile_tool_frames_.front_right, "robot_arm_tool0");

    //    _nh.param<std::string>("base_frame", mobile_base_frames_.front_left, "robot_front_left_wheel");
    //    _nh.param<std::string>("tool_frame", mobile_tool_frames_.front_left, "robot_arm_tool0");

    //    _nh.param<std::string>("base_frame", mobile_base_frames_.back_right, "robot_back_right_wheel");
    //    _nh.param<std::string>("tool_frame", mobile_tool_frames_.back_right, "robot_arm_tool0");

    //    _nh.param<std::string>("base_frame", mobile_base_frames_.back_left, "robot_back_left_wheel");
    //    _nh.param<std::string>("tool_frame", mobile_tool_frames_.back_left, "robot_arm_tool0");

    _sub_joint_state = _nh.subscribe("joint_states", 10, &Jacobian::jointStateCallback, this);
    _pub_jacobian = _n.advertise<std_msgs::Float64MultiArray>("jacobian_0",1);

    _nh.param("robot_description", _robot_desc, std::string());

    if(!kdl_parser::treeFromString(_robot_desc, _kdl_tree)) {
        ROS_ERROR("KDL tree was not able constructed, shutting down the node");
        ros::shutdown();
        return;
    }
    ROS_INFO_STREAM(_kdl_tree.getNrOfJoints() << " is the number of joints in the tree");
    ROS_INFO_STREAM("The root segment in the tree is " << _kdl_tree.getRootSegment()->first);
    ROS_INFO_STREAM(_kdl_tree.getNrOfSegments() << " is the number of segments in the tree. "
                                                   "The name of each segment from the of the SegmentMap is as follows:");
    for (auto it = _kdl_tree.getSegments().begin(); it != _kdl_tree.getSegments().end(); it++) {
        ROS_INFO_STREAM( "  " << it->first);
        ROS_INFO_STREAM("       This segment has a joint of type "
                        << it->second.segment.getJoint().getTypeName() << " and name "
                        << it->second.segment.getJoint().getName());
    }

    if (!_kdl_tree.getChain(mobile_base_frames_.front_right, mobile_tool_frames_.front_right, _kdl_chain)) {
        ROS_ERROR("KDL chain was not able to be obtained, shutting down the node");
        ros::shutdown();
        return;
    }

    ROS_INFO_STREAM(_kdl_chain.getNrOfSegments() << " is the number of segments in the chain");
    ROS_INFO_STREAM(_kdl_chain.getNrOfJoints() << " is the number of joints in the chain");
    ROS_INFO_STREAM("Name of each joint in each segment is as follows: ");
    for (int seg_num = 0; seg_num  < _kdl_chain.getNrOfSegments(); ++seg_num) {
        ROS_INFO_STREAM("   " << _kdl_chain.getSegment(seg_num).getJoint().getName());

    }

    threads.push_back(std::thread(&Jacobian::tf_listener_thread, this));

    _jacobian_solver.reset(new KDL::TreeJntToJacSolver(_kdl_tree));
    _chain_jacobian_solver.reset(new KDL::ChainJntToJacSolver(_kdl_chain));
}

void Jacobian::tf_listener_thread() {
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener(tf_buffer);
    geometry_msgs::TransformStamped transform_stamped;

    ros::Rate rate(30);

    while(ros::ok()) {
        try {
            transform_stamped = tf_buffer.lookupTransform("robot_odom", "robot_base_footprint", ros::Time(0));
            frame_rotation_ = tf2::transformToKDL(transform_stamped).M;
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
    // Have the callback exit out of the cv so code does not hang
    transform_calcd = true;
    cv_.notify_all();
}

void Jacobian::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    std::unique_lock<std::mutex> lck(mtx_);
    cv_.wait(lck, [&]{return transform_calcd.load() || !ros::ok();});
    transform_calcd = false;

    // 6 for robot arm, 8 for robot arm + 2DoF for steering/moving or 14 for robot system msg->position.size();
    int jnt = 14;

    KDL::JntArray kdl_jnt(jnt);
    for (int i = 0; i < jnt; i++) {
        kdl_jnt.data[i] = msg->position[std::find(msg->name.begin(), msg->name.end(), joint_names_[i]) - msg->name.begin()];
    }


    KDL::Jacobian kdl_jacobian(jnt);
    KDL::Jacobian kdl_jacobian_base(jnt);
    _jacobian_solver->JntToJac(kdl_jnt, kdl_jacobian, "robot_arm_tool0");
//    _chain_jacobian_solver->JntToJac(kdl_jnt, kdl_jacobian);
    KDL::changeBase(kdl_jacobian, frame_rotation_, kdl_jacobian_base);
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
            jacobian_msg.data.push_back(kdl_jacobian_base(r,c));
        }
    }

    _pub_jacobian.publish(jacobian_msg);

}

int main(int argc, char **argv) {
    
    /// Initialize ROS node
    ros::init(argc, argv, "jacobian");
    ros::NodeHandle n;
    Jacobian jacobian(&n);
    ros::spin();
    ros::shutdown();

    return 0;
}
