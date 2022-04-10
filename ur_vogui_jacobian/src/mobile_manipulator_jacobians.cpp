#include "../include/mobile_manipulator_jacobians.h"

//#define DEBUG

Jacobian::~Jacobian(){
    std::cout << "destroying class" << std::endl;
    for (auto &t : threads) {
        t.join();
        std::cout << "joining threads" << std::endl;
    }
}

Jacobian::Jacobian() {
    ros::NodeHandle _n;
    _nh = ros::NodeHandle("robot/");

    std::string robot_arm_base = "robot_arm_base", robot_arm_tool = "robot_arm_tool0";

    sub_vogui_odom_ = _nh.subscribe("robotnik_base_control/odom", 10, &Jacobian::odomCallback, this);
    sub_joint_state_ = _nh.subscribe("joint_states", 10, &Jacobian::jointStateCallback, this);
    pub_jacobian_ = _n.advertise<std_msgs::Float64MultiArray>("jacobian_0",1);
    pub_jacobian_e_ = _n.advertise<std_msgs::Float64MultiArray>("jacobian_e",1);

    _nh.param("robot_description", _robot_desc, std::string());

    KDL::Tree tree;
    if(!kdl_parser::treeFromString(_robot_desc, tree)) {
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
    KDL::Vector xy_origin(0,0,0);
    KDL::Vector prisy_axis(0,1,0), prisx_axis(1,0,0);

    // Model the RB-VOGUI using 2 prismatic joints along the y and x
    KDL::Joint prisy("prisy", xy_origin, prisy_axis, KDL::Joint::JointType::TransAxis),
        prisx("prisx", xy_origin, prisx_axis, KDL::Joint::JointType::TransAxis);

    // Define segment in which each joint will be attached to and then add it on the chain
    KDL::Segment segy("segy", prisy), segx("segx", prisx);
    _kdl_chain.addSegment(segy);
    _kdl_chain.addSegment(segx);
    for (int i = 1; i < 8; i++) { // Add each segment on the robot arm on the chain
        _kdl_chain.addSegment(arm.getSegment(i));
    }
    num_jnts_ = _kdl_chain.getNrOfJoints();

    // Start the tf thread and reset the jacobian solver and initialise member variables
    threads.push_back(std::thread(&Jacobian::tf_listener_thread, this));
    _chain_jacobian_solver.reset(new KDL::ChainJntToJacSolver(_kdl_chain));
    joint_positions_.jnts.resize(num_jnts_);
    jacobian_msg_0_.layout.dim.resize(2);
    jacobian_msg_0_.layout.dim[0].label = "rows";
    jacobian_msg_0_.layout.dim[0].size = 6;
    jacobian_msg_0_.layout.dim[0].stride = 1;
    jacobian_msg_0_.layout.dim[1].label = "columns";
    jacobian_msg_0_.layout.dim[1].size = num_jnts_;
    jacobian_msg_0_.layout.dim[1].stride = 1;
    jacobian_msg_ee_.layout.dim.resize(2);
    jacobian_msg_ee_.layout.dim[0].label = "rows";
    jacobian_msg_ee_.layout.dim[0].size = 6;
    jacobian_msg_ee_.layout.dim[0].stride = 1;
    jacobian_msg_ee_.layout.dim[1].label = "columns";
    jacobian_msg_ee_.layout.dim[1].size = num_jnts_;
    jacobian_msg_ee_.layout.dim[1].stride = 1;


#ifdef DEBUG
    ROS_INFO_STREAM("Obtaining names of joints in the chain");
    for(int i = 0; i < _kdl_chain.getNrOfSegments(); i++) {
        ROS_INFO_STREAM("Segment name" << _kdl_chain.getSegment(i).getName());
        ROS_INFO_STREAM("Joint name" << _kdl_chain.getSegment(i).getJoint().getName());
        auto originn = _kdl_chain.getSegment(i).getJoint().JointOrigin();
        auto axis = _kdl_chain.getSegment(i).getJoint().JointAxis();
        ROS_INFO_STREAM("   origin " << 1.001*originn.data[0] << " " << 1.001*originn.data[1] << " " << 1.001*originn.data[2]);
        ROS_INFO_STREAM("   axis " << 1.001*axis.data[0] << " " << 1.001*axis.data[1] << " " << 1.001*axis.data[2]);
    }

    ros::shutdown(); return;
#endif

}

void Jacobian::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    uniq_lck lck(joint_positions_.mtx);
    joint_positions_.jnts.data[0] = msg->pose.pose.position.y;
    joint_positions_.jnts.data[1] = msg->pose.pose.position.x;
}

void Jacobian::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {

    uniq_lck lck(joint_positions_.mtx);
    int j = 0;
    for (int i = 2; i < num_jnts_; i++) {
        joint_positions_.jnts.data[i] = msg->position[std::find(msg->name.begin(), msg->name.end(), joint_names_[j]) - msg->name.begin()];
        j++;
    }
}

void Jacobian::tf_listener_thread() {
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener(tf_buffer);
    geometry_msgs::TransformStamped transform_stamped;

    ros::Rate rate(25);

    while(ros::ok()) {
        try {
            transform_stamped = tf_buffer.lookupTransform("robot_arm_tool0", "robot_arm_base", ros::Time(0));
            frame_rotation_ = tf2::transformToKDL(transform_stamped).M;
            frame_rotation_.DoRotZ(M_PI);
            publishJacobians();
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        rate.sleep();
    }
}

void Jacobian::publishJacobians() {

    KDL::Jacobian kdl_jacobian(num_jnts_);
    KDL::Jacobian kdl_jacobian_ee(num_jnts_);

#ifdef DEBUG
    std::vector<double> test_joints{0, 0, 0, -0.87266, -1.2217, 1.2217, 0, 0, 0};
    for (unsigned long i = 0; i < test_joints.size(); i++) {
        joint_positions_.jnts.data[i] = test_joints[i];
    }
#endif

    uniq_lck lck(joint_positions_.mtx);
    _chain_jacobian_solver->JntToJac(joint_positions_.jnts, kdl_jacobian);
    KDL::changeBase(kdl_jacobian, frame_rotation_, kdl_jacobian_ee);
    lck.unlock();

    // Clear the Jacobians, populate it and then publish
    jacobian_msg_0_.data.clear();
    jacobian_msg_ee_.data.clear();

    for(int r = 0; r < 6; r++) {
        for(int c = 0; c < num_jnts_; c++) {
            jacobian_msg_0_.data.push_back(kdl_jacobian(r,c));
            jacobian_msg_ee_.data.push_back(kdl_jacobian_ee(r,c));

        }
    }

    pub_jacobian_.publish(jacobian_msg_0_);
    pub_jacobian_e_.publish(jacobian_msg_ee_);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "jacobian");
    Jacobian jacobian;
    ros::spin();
    ros::shutdown();
    return 0;
}
