// ###################################################################
// ################### Importing Necessary Libraries #################
// ###################################################################

#include <ros/ros.h>
#include <std_msgs/String.h>

// include messages to store robot configurations
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// nist
#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/VacuumGripperControl.h>

// standard library
#include <string>
#include <vector>
#include <array>
#include <cstdarg>

#include "utils.h"


// group2 msg imports
#include <group2_rwa4/KittingLocation.h>
#include <group2_rwa4/kitting_part_details.h>
#include <group2_rwa4/dispose_faulty_part.h>

// ####################################################################


/**
 * @brief Class for Gantry robot to perform pick, place, trash and complete a kitting and assembly shipments
 * 
 */
class GantryRobot {

    public:

    /**
     * @brief Construct a new Gantry Robot object
     * 
     * @param node 
     */
    GantryRobot(ros::NodeHandle& node);

    /**
     * @brief Init function
     * 
     */
    void init(); // initialize the kitting robot class

    // gantry kitting predefined locations
    // std::vector<double> arm_preset_at_home =   {-2.87, -1.6, 0 , 1.73, -1.00, -0.05, -0.72, 1.54, 0.83 }; // {Torso(3), ARM(6)}
    // std::vector<double> arm_preset_at_bins0 =  {-2.87, -1.6, 0 , 1.73, -1.00, -0.05, -0.72, 1.54, 0.83 };
    // std::vector<double> arm_preset_at_bins1 =  {-2.87, -1.6, 0 , 1.73, -1.00, -0.05, -0.72, 1.54, 0.83 };
    // std::vector<double> arm_preset_at_agv1 =   {-1.01, -2.48, 3.69, 0.68, -0.88, 0, 3.38, 1.6, 0 };
    // std::vector<double> arm_preset_at_agv2 =   {-1.01, -0.85, 2.43, 0.68, -0.88, 0, 3.38, 1.6, 0 };
    // std::vector<double> arm_preset_at_agv3 =   {-1.01, -2.48,-2.43, 0.68, -0.88, 0, 3.38, 1.6, 0 };
    // std::vector<double> arm_preset_at_agv4 =   {-1.01, -0.85,-3.69, 0.68, -0.88, 0, 3.38, 1.6, 0 };
////////////////////////////////////////////////////////////////////

    // Preset locations
        // ^^^^^^^^^^^^^^^^
        // Joints for the gantry are in this order:
        // For the torso
        // - small_long_joint
        // - torso_rail_joint
        // - torso_base_main_joint
        // For the arm
        // - gantry_arm_shoulder_pan_joint
        // - gantry_arm_shoulder_lift_joint
        // - gantry_arm_elbow_joint
        // - gantry_arm_wrist_1
        // - gantry_arm_wrist_2
        // - gantry_arm_wrist_3
        // For the full robot = torso + arm
    std::vector<double> test =   {-3.63, 0.0, 0.0 ,0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };

    std::vector<double> arm_preset_at_world_center =   {-5, 0, 0 , 0.0, 0.0, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_center_0 =   {-1.51, 0, 0.0 ,0.0 , -0.87 , 1.25 ,-0.30 ,1.54 ,0};
    std::vector<double> arm_preset_at_center_1 =   {-5, 0.0, 0.0 ,-0.26, -0.60 , 1.37 , -0.80 ,1.54 ,0};
    std::vector<double> arm_preset_at_center_2 =   {-10, 0, 0 ,0.0, 0.0 , -0.87 , 1.25 ,-0.30 ,1.54 ,0};


    std::vector<double> arm_preset_at_home =   {-2.00, -1.60, 0 , 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };

    std::vector<double> arm_preset_at_bins_set_0 =  {-2.0, -2.9, 0.0 , 0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83};
    std::vector<double> arm_preset_at_bins2 =  {-1.23, -3, 0.0 , 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0};
    std::vector<double> arm_preset_at_bins3 =  {-1.2, -2.3, -1.5708 ,0.0, -0.785398, 1.74533, -0.785398, 1.5708, 0.0};

    std::vector<double> arm_preset_at_bins_set_1 =  {-2.00, 2.88, 0.0  , 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_bins6 =  {-1.23, 3.04, 0.0, 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_bins7 =  {-1.23, 3.76, 0.0 , 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };

    // std::vector<double> arm_preset_at_agv1 =   {-1.6, -3.88, 0.0 , 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_agv1 =   {-4.02, -3.33, 1.62 , 0.0, -1.87, 1.75, 0.20, 1.54, 0.83 };
    std::vector<double> arm_preset_at_agv2 =   {-1.6, -1.70, 0.0, 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_agv3 =   {-1.6, 2.08, 0.0 , 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };
    std::vector<double> arm_preset_at_agv4 =   {-1.6, 4.14, 0.0 , 0.0, -0.785398, 1.5708, -0.785398, 1.5708, 0.0 };

    std::vector<double> arm_preset_at_pre_as1 =   {-2.69, -3.13, 1.5, 0.0, -1.87, 1.75, 0.20, 1.54, 0};
    std::vector<double> arm_preset_at_pre_as3 =   {-2.40, 2.82, 1.5, 0.0, -1.87, 1.75, 0.20, 1.54, 0};
    std::vector<double> arm_preset_at_pre_as2 =   {-7.37, -3.12, 1.5, 0.0, -1.87, 1.75, 0.20, 1.54, 0};    
    std::vector<double> arm_preset_at_pre_as4 =   {-7.52, 2.82, 1.5, 0.0, -1.87, 1.75, 0.20, 1.54, 0};

    std::vector<double> arm_preset_at_as1 =   {-3.87, -3.07, 1.44, 0 , -1.88 , 1.50 ,0.38 ,1.55 ,1.50};
    std::vector<double> arm_preset_at_as3 =   {-4.03, 2.82, 1.5, 0.0, -1.87, 1.75, 0.20, 1.54, 1.50};
    std::vector<double> arm_preset_at_as2 =   {-9.00, -3.12, 1.5, 0.0, -1.87, 1.75, 0.20, 1.54, 1.50};
    std::vector<double> arm_preset_at_as4 =   {-9.00, 2.82, 1.5, 0.0, -1.87, 1.75, 0.20, 1.54, 1.50};

    // std::vector<double> arm_preset_at_as1_agv1 =   { -2.70, -4.5, 1.19  ,0 , -1.13 , 1.88 ,-0.72 ,1.55 ,0.83 };
    std::vector<double> arm_preset_at_as1_agv1 =   { -2.69, -4.50, 1.24 , 0 , -0.87 , 1.25 ,-0.30 ,1.54 ,0};
    std::vector<double> arm_preset_at_as1_agv2 =   { -2.78, -1.77, 2.0 , 0 , -0.87 , 1.25 ,-0.30 ,1.54 ,0};
    std::vector<double> arm_preset_at_as2_agv1 =   { -7.70, -4.2, 0.99 , 0 , -0.87 , 1.25 ,-0.30 ,1.54 ,0};
    std::vector<double> arm_preset_at_as2_agv2 =   { -7.70, -1.86, 2.00 , 0 , -0.87 , 1.25 ,-0.30 ,1.54 ,0};
    std::vector<double> arm_preset_at_as3_agv3 =   { -2.78, 1.74, 0.99 , 0 , -0.87 , 1.25 ,-0.30 ,1.54 ,0};
    std::vector<double> arm_preset_at_as3_agv4 =   { -2.78, 4.26, 2.0 , 0 , -0.87 , 1.25 ,-0.30 ,1.54 ,0};
    std::vector<double> arm_preset_at_as4_agv3 =   { -7.81, 1.74, 0.99 , 0 , -0.87 , 1.25 ,-0.30 ,1.54 ,0};
    std::vector<double> arm_preset_at_as4_agv4 =   { -7.81, 4.17, 2.00 , 0 , -0.87 , 1.25 ,-0.30 ,1.54 ,0};


    private:
    ros::NodeHandle nh;
    std::string gantry_robot_planing_grp;

    std::vector<double> current_joint_group_positions;
    std::vector<double> joint_arm_positions_;

    // gripper attributes
    nist_gear::VacuumGripperState current_gripper_state;

    // msgs to store robot states
    nist_gear::VacuumGripperState gantry_robot_gripper_state;
    sensor_msgs::JointState current_gantry_robot_joint_states;
    control_msgs::JointTrajectoryControllerState gantry_robot_controller_state;
    control_msgs::JointTrajectoryControllerState gantry_arm_controller_state_;

    // gantry robot arm moveit configurations
    moveit::planning_interface::MoveGroupInterface::Options gantry_robot_moveit_options;
    moveit::planning_interface::MoveGroupInterface::Options gantry_robot_arm_moveit_options;
    moveit::planning_interface::MoveGroupInterface gantry_robot_moveit_group;
    moveit::planning_interface::MoveGroupInterface gantry_robot_arm_moveit_group;

    // Publisher nodes
    ros::Publisher gantry_robot_joint_trajectory_publisher;
    ros::Publisher gantry_robot_arm_joint_trajectory_publisher;
    ros::Subscriber gantry_robot_joint_states_subscriber;
    ros::Subscriber gantry_robot_gripper_state_subscriber;
    ros::Subscriber gantry_robot_controller_state_subscriber;
    ros::Subscriber gantry_arm_controller_state_subscriber;

    // Service clients and servers
    ros::ServiceClient gantry_robot_gripper_control_service_client;
    ros::ServiceServer gantry_robot_pick_place_service;
    ros::Subscriber gantry_arm_controller_state_subscriber_;
    // ros::ServiceServer gantry_robot_dispose_faulty_service;

    /**
     * @brief method to move the gantry robot on the linear joint actuator 
     * 
     * @param location 
     */
    void go_to_location(std::vector<double> location);

//################# gripper methods ############# 

    /**
     * @brief method to activate the gripper for picking up parts
     * 
     */
    void activate_gripper();

    /**
     * @brief method to de-activate the gripper for placing and trashing parts
     * 
     */
    void deactivate_gripper();

    /**
     * @brief method to get the gripper state object
     * 
     * @return nist_gear::VacuumGripperState 
     */
    nist_gear::VacuumGripperState get_gripper_state();

    /**
     * @brief method to pick up a part
     * 
     * @param part_pose 
     * @return true 
     * @return false 
     */
    bool pickup_part(geometry_msgs::Pose part_pose, double part_offset);

    /**
     * @brief methd to place the part
     * 
     * @param target_pose 
     * @return true 
     * @return false 
     */
    bool place_part(geometry_msgs::Pose target_pose); 

//############## callback functions ############# 

    /**
     * @brief callback function for the joint states of gantry robot
     * 
     * @param joint_state_msg 
     */
    void gantry_robot_joint_states_callback(const sensor_msgs::JointState::ConstPtr& joint_state_msg);

    /**
     * @brief callback function for the gripper state of gantry robot
     * 
     * @param vacuum_state_msg 
     */
    void gantry_robot_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& vacuum_state_msg);

    /**
     * @brief callback function for the controller state of gantry robot
     * 
     * @param controller_state_msg 
     */
    void gantry_robot_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& controller_state_msg); 

    /**
     * @brief callback function for the controller state of gantry robot
     * 
     * @param controller_state_msg 
     */
    void gantry_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& controller_state_msg); 

    /**
     * @brief subscriber callback function to perform kitting 
     * 
     * @param kitting_location 
     * @return true 
     * @return false 
     */
    bool perform_kitting_sub_callback(const group2_rwa4::KittingLocation kitting_location);

    /**
     * @brief service callback function for picking and placing using the kitting robot
     * 
     * @param req 
     * @param res 
     * @return true 
     * @return false 
     */
    bool gantry_robot_pick_place_service_callback(group2_rwa4::kitting_part_details::Request &req, group2_rwa4::kitting_part_details::Response &res);
    
    /**
     * @brief service callback function to trash a part using gantry robot
     * 
     * @param req 
     * @param res 
     * @return true 
     * @return false 
     */
    bool gantry_robot_dispose_faulty_service_callback(group2_rwa4::dispose_faulty_part::Request &req, group2_rwa4::dispose_faulty_part::Response &res);
};

