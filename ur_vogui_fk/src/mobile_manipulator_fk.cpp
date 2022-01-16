#include "../include/mobile_manipulator_fk.h"

forward_kinematics::~forward_kinematics() {
    t_->join();
}

forward_kinematics::forward_kinematics()
{
    nh_ = ros::NodeHandle("robot/");
    ros::NodeHandle nh;
    std::string robot_arm_base = "robot_arm_base", robot_arm_tool = "robot_arm_tool0";
    sub_odom_ = nh_.subscribe("robotnik_base_control/odom", 10, &forward_kinematics::odom_cb, this);
    sub_joint_state_ = nh_.subscribe("joint_states", 10, &forward_kinematics::joint_state_cb, this);
    pub_ee_traj_ = nh.advertise<visualization_msgs::Marker>("vis_ee_traj",1);

    nh_.param("robot_description", robot_desc_, std::string());

    KDL::Tree tree;
    if(!kdl_parser::treeFromString(robot_desc_, tree)) {
        ROS_ERROR("KDL tree was not able constructed, shutting down the node");
        ros::shutdown();
        return;
    }

    KDL::Chain arm;
    if (!tree.getChain(robot_arm_base, robot_arm_tool, arm)) {
        ROS_ERROR("KDL chain was not able to be obtained, shutting down the node");
        ros::shutdown();
        return;
    }

    // Define the origin of the joints and axis they rotate around or translate
    // along. Must coincide with the UR10's base joint
    KDL::Vector z_origin(0, 0, 0.127959), xy_origin(0,0,0);
    KDL::Vector rotz_axis(0,0,1), prisy_axis(0,1,0), prisx_axis(1,0,0);

    // Model the RB-VOGUI using 1 revolute joint around z and 2 prismatic joints
    // along the y and x
    KDL::Joint rotz("rotz", z_origin, rotz_axis, KDL::Joint::JointType::RotAxis),
        prisy("prisy", xy_origin, prisy_axis, KDL::Joint::JointType::TransAxis),
        prisx("prisx", xy_origin, prisx_axis, KDL::Joint::JointType::TransAxis);

    // Define segment in which each joint will be attached to and then add it on the chain
    KDL::Segment segz("segz", rotz), segy("segy", prisy), segx("segx", prisx);
    kdl_chain_.addSegment(segz);
    kdl_chain_.addSegment(segy);
    kdl_chain_.addSegment(segx);
    for (int i = 1; i < 8; i++) { // Add each segment on the robot arm on the chain
        kdl_chain_.addSegment(arm.getSegment(i));
    }
    num_jnts_ = kdl_chain_.getNrOfJoints();


    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    jnt_positions_.jnts.resize(num_jnts_);

    t_ = new std::thread(&forward_kinematics::visualise_trajectory, this);
}

void forward_kinematics::joint_state_cb(const sensor_msgs::JointStateConstPtr &msg) {
    uniq_lck lck(jnt_positions_.mtx);
    int j = 0;
    for (int i = 3; i < num_jnts_; i++) {
        jnt_positions_.jnts.data[i] = msg->position[std::find(msg->name.begin(), msg->name.end(), joint_names_[j]) - msg->name.begin()];
        j++;
    }
}

void forward_kinematics::odom_cb(const nav_msgs::OdometryConstPtr &msg) {
    uniq_lck lck(jnt_positions_.mtx);
    jnt_positions_.jnts.data[0] = tf::getYaw(msg->pose.pose.orientation);
    jnt_positions_.jnts.data[1] = msg->pose.pose.position.y;
    jnt_positions_.jnts.data[2] = msg->pose.pose.position.x;

}

void forward_kinematics::visualise_trajectory() {
    ros::Duration(2).sleep();
    unsigned long int id = 0;
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time();
    marker.header.frame_id = "robot_odom";
    marker.ns = "ns";
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points.resize(1);
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(6);
    KDL::Frame frame;
    geometry_msgs::Point points;

    while(ros::ok()) {
        ros::Duration(0.2).sleep();

        for (int i = 0; i < 6; i++) {
            uniq_lck lck(jnt_positions_.mtx);
            KDL::JntArray jnt_pos = jnt_positions_.jnts;
            lck.unlock();
            fk_solver_->JntToCart(jnt_pos, frame);
            points.x = frame.p.x();
            points.y = frame.p.y();
            points.z = frame.p.z();
            ROS_INFO_STREAM(points.x << ", " << points.y << ", " << points.z+0.5545);

            marker.id = id;
            marker.points.at(0).x = points.x;
            marker.points.at(0).y = points.y;
            marker.points.at(0).z = points.z+0.5545;

            id++;

            pub_ee_traj_.publish(marker);
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "fk");
    forward_kinematics fk;
    ros::spin();
    ros::shutdown();
    return 0;
}
