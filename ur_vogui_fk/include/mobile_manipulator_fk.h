#ifndef MOBILE_MANIPULATOR_FK_H
#define MOBILE_MANIPULATOR_FK_H

#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_kdl/tf2_kdl.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/kdl.hpp>

#include <boost/scoped_ptr.hpp>

class forward_kinematics
{
public:
    forward_kinematics();
    ~forward_kinematics();

    struct JointPosition {
        std::mutex mtx;
        KDL::JntArray jnts;
    };

    typedef std::unique_lock<std::mutex> uniq_lck;

private:
    void joint_state_cb(const sensor_msgs::JointStateConstPtr &msg);
    void odom_cb(const nav_msgs::OdometryConstPtr &msg);
    void visualise_trajectory();

    int num_jnts_;

    std::thread* t_;
    std::string robot_desc_;
    std::vector<std::string> joint_names_ =
        {
            "robot_arm_shoulder_pan_joint",
            "robot_arm_shoulder_lift_joint",
            "robot_arm_elbow_joint",
            "robot_arm_wrist_1_joint",
            "robot_arm_wrist_2_joint",
            "robot_arm_wrist_3_joint",
            };

    ros::NodeHandle nh_;
    ros::Subscriber sub_joint_state_, sub_odom_;
    ros::Publisher pub_ee_traj_;

    KDL::Chain kdl_chain_;

    JointPosition jnt_positions_;

    boost::scoped_ptr<KDL::ChainFkSolverPos> fk_solver_;
};

#endif // MOBILE_MANIPULATOR_FK_H
