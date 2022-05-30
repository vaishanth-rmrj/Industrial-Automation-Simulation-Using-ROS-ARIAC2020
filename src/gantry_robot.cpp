// #include "kitting_robot.h"
#include "gantry_robot.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <tf2/convert.h>


GantryRobot::GantryRobot(ros::NodeHandle& node) : nh("/ariac/gantry"),
    gantry_robot_planing_grp("/ariac/gantry/robot_description"),
    gantry_robot_moveit_options("gantry_full", gantry_robot_planing_grp, nh),
    gantry_robot_moveit_group(gantry_robot_moveit_options),
    gantry_robot_arm_moveit_options("gantry_arm", gantry_robot_planing_grp, nh),
    gantry_robot_arm_moveit_group(gantry_robot_arm_moveit_options)
{
    // nh = node;
    init();
}

void GantryRobot::init()
{
    ROS_INFO("Gantry Robot initialized :)");
    gantry_robot_joint_trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/gantry_controller/command", 10);

    gantry_robot_arm_joint_trajectory_publisher = nh.advertise<trajectory_msgs::JointTrajectory>("/ariac/gantry/gantry_arm_controller/command", 10);
    gantry_robot_joint_states_subscriber = nh.subscribe("/ariac/gantry/joint_states", 10, &GantryRobot::gantry_robot_joint_states_callback, this);
    gantry_robot_gripper_state_subscriber = nh.subscribe("/ariac/gantry/arm/gripper/state", 10, &GantryRobot::gantry_robot_gripper_state_callback, this);
    gantry_robot_controller_state_subscriber = nh.subscribe("/ariac/gantry/gantry_controller/state", 10, &GantryRobot::gantry_robot_controller_state_callback, this);
    gantry_arm_controller_state_subscriber_ = nh.subscribe("/ariac/gantry/gantry_arm_controller/state", 10, &GantryRobot::gantry_arm_controller_state_callback, this);
    gantry_robot_gripper_control_service_client = nh.serviceClient<nist_gear::VacuumGripperControl>("/ariac/gantry/arm/gripper/control");
    gantry_robot_gripper_control_service_client.waitForExistence(); 
    gantry_robot_pick_place_service = nh.advertiseService("/group2/gantry_robot_pick_and_place", &GantryRobot::gantry_robot_pick_place_service_callback, this);
    nh.setParam("/gantry_placed_part",false);


    const moveit::core::JointModelGroup* gantry_robot_joint_model_group = gantry_robot_moveit_group.getCurrentState()->getJointModelGroup("gantry_full");
    moveit::core::RobotStatePtr gantry_robot_current_state = gantry_robot_moveit_group.getCurrentState();
    gantry_robot_current_state->copyJointGroupPositions(gantry_robot_joint_model_group, current_joint_group_positions);


    const moveit::core::JointModelGroup* joint_arm_group = gantry_robot_arm_moveit_group.getCurrentState()->getJointModelGroup("gantry_arm");
    moveit::core::RobotStatePtr current_state_arm = gantry_robot_arm_moveit_group.getCurrentState();
    current_state_arm->copyJointGroupPositions(joint_arm_group, joint_arm_positions_);

}

void GantryRobot::gantry_robot_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr& vacuum_state_msg)
{   
    gantry_robot_gripper_state = *vacuum_state_msg;
}

void GantryRobot::gantry_robot_joint_states_callback(const sensor_msgs::JointState::ConstPtr& joint_state_msg)
{
    if (joint_state_msg->position.size() == 0) {
        ROS_ERROR("[Arm][arm_joint_states_callback_] joint_state_msg->position.size() == 0!");
    }
    current_gantry_robot_joint_states = *joint_state_msg;
}

void GantryRobot::gantry_robot_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& controller_state_msg)
{
    gantry_robot_controller_state = *controller_state_msg;
}

void GantryRobot::gantry_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
    gantry_arm_controller_state_ = *msg;
}


void GantryRobot::go_to_location(std::vector<double> location)
{


    current_joint_group_positions.clear();

    for (int i = 0; i < location.size(); i++)
    {
        current_joint_group_positions.push_back(location.at(i));
    }


    gantry_robot_moveit_group.setJointValueTarget(current_joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan gantry_robot_path_plan;
    bool success = (gantry_robot_moveit_group.plan(gantry_robot_path_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
        ROS_INFO("Gantry Robot is moving");
        gantry_robot_moveit_group.move();
    }

}

void GantryRobot::activate_gripper()
{
    nist_gear::VacuumGripperControl srv_msg;
    srv_msg.request.enable = true;
    gantry_robot_gripper_control_service_client.call(srv_msg);

    ROS_INFO_STREAM("Activate gantry robot gripper response : " << srv_msg.response);
}


void GantryRobot::deactivate_gripper()
{
    nist_gear::VacuumGripperControl srv_msg;
    srv_msg.request.enable = false;
    gantry_robot_gripper_control_service_client.call(srv_msg);

}

nist_gear::VacuumGripperState GantryRobot::get_gripper_state()
{
    return gantry_robot_gripper_state;
}


bool GantryRobot::pickup_part(geometry_msgs::Pose part_pose, double part_offset)
{
    activate_gripper();
    ros::Duration(0.5).sleep();        

    geometry_msgs::Pose gantry_ee_link_pose = gantry_robot_arm_moveit_group.getCurrentPose().pose;

    tf2::Quaternion flat_orientation;
    flat_orientation.setRPY(0, 1.57, 0);
        
    gantry_ee_link_pose.orientation.x = flat_orientation.getX();
    gantry_ee_link_pose.orientation.y = flat_orientation.getY();
    gantry_ee_link_pose.orientation.z = flat_orientation.getZ();
    gantry_ee_link_pose.orientation.w = flat_orientation.getW();

    tf2::Quaternion  gantry_ee_link_orientation(
        gantry_ee_link_pose.orientation.x,
        gantry_ee_link_pose.orientation.y,
        gantry_ee_link_pose.orientation.z,
        gantry_ee_link_pose.orientation.w
    );
    
    geometry_msgs::Pose part_init_pose_in_world = part_pose;
    part_init_pose_in_world.position.z = part_init_pose_in_world.position.z + part_offset;
    part_init_pose_in_world.orientation.x = gantry_ee_link_pose.orientation.x;
    part_init_pose_in_world.orientation.y = gantry_ee_link_pose.orientation.y;
    part_init_pose_in_world.orientation.z = gantry_ee_link_pose.orientation.z;
    part_init_pose_in_world.orientation.w = gantry_ee_link_pose.orientation.w;

    auto state = get_gripper_state();
    while (!state.enabled) {
        activate_gripper();
        state = get_gripper_state();        
    }

    if (!state.enabled) {
        ROS_FATAL_STREAM("Gantry robot gripper state: Failed");
        ros::shutdown();
    }

    if (state.enabled) {
        ROS_INFO("Gantry robot gripper state: Enabled");

        // move arm to part offset
        gantry_robot_arm_moveit_group.setPoseTarget(part_init_pose_in_world);
        gantry_robot_arm_moveit_group.move();        

        // move the arm closer until the object is attached
        state = get_gripper_state();        
        while (!state.attached) {
            part_init_pose_in_world.position.z = part_init_pose_in_world.position.z - 0.0005;
            gantry_robot_arm_moveit_group.setPoseTarget(part_init_pose_in_world);
            gantry_robot_arm_moveit_group.move();
            gantry_robot_arm_moveit_group.setPoseTarget(gantry_ee_link_pose); 
            ros::Duration(0.5).sleep();               
            state = get_gripper_state();            
        }

        geometry_msgs::Pose post_grasp_pose = gantry_robot_arm_moveit_group.getCurrentPose().pose;
        post_grasp_pose.position.z += 0.2;
        gantry_robot_arm_moveit_group.setPoseTarget(post_grasp_pose);
        gantry_robot_arm_moveit_group.move();
        ros::Duration(2.0).sleep();

        ROS_INFO_STREAM("Part attached to the gripper");
        return true;
    }
    return false;

}



bool GantryRobot::place_part(geometry_msgs::Pose part_in_world_frame)
{
    ROS_INFO_STREAM("Part pose when in place part func is: " << part_in_world_frame);

    geometry_msgs::Pose target_pose_in_world;
    target_pose_in_world = part_in_world_frame;
    target_pose_in_world.position.x = part_in_world_frame.position.x;
    target_pose_in_world.position.y = part_in_world_frame.position.y;
    target_pose_in_world.position.z = part_in_world_frame.position.z;
    target_pose_in_world.orientation.x = part_in_world_frame.orientation.x;
    target_pose_in_world.orientation.y = part_in_world_frame.orientation.y;
    target_pose_in_world.orientation.z = part_in_world_frame.orientation.z;
    target_pose_in_world.orientation.w = part_in_world_frame.orientation.w;

    // geometry_msgs::Pose arm_ee_link_pose = gantry_robot_arm_moveit_group.getCurrentPose().pose;
    // auto flat_orientation = motioncontrol::quaternionFromEuler(0, 1.57, 0);
    // arm_ee_link_pose = gantry_robot_arm_moveit_group.getCurrentPose().pose;
    // arm_ee_link_pose.orientation.x = flat_orientation.getX();
    // arm_ee_link_pose.orientation.y = flat_orientation.getY();
    // arm_ee_link_pose.orientation.z = flat_orientation.getZ();
    // arm_ee_link_pose.orientation.w = flat_orientation.getW();



    ROS_INFO("Target World Position: %f, %f, %f",
        part_in_world_frame.position.x,
        part_in_world_frame.position.y,
        part_in_world_frame.position.z);

    ROS_INFO("Target World Orientation: %f, %f, %f, %f",
        part_in_world_frame.orientation.x,
        part_in_world_frame.orientation.y,
        part_in_world_frame.orientation.z,
        part_in_world_frame.orientation.w);

    auto ee_pose = gantry_robot_arm_moveit_group.getCurrentPose().pose;

    tf2::Quaternion q_current(
        ee_pose.orientation.x,
        ee_pose.orientation.y,
        ee_pose.orientation.z,
        ee_pose.orientation.w);

    ee_pose.position.x = target_pose_in_world.position.x;
    ee_pose.position.y = target_pose_in_world.position.y;

    gantry_robot_arm_moveit_group.setMaxVelocityScalingFactor(1.0);
    // gantry_robot_arm_moveit_group.setPoseTarget(ee_pose);
    // gantry_robot_arm_moveit_group.move();
    
    // orientation of the part in the bin, in world frame
    tf2::Quaternion q_init_part(
        target_pose_in_world.orientation.x,
        target_pose_in_world.orientation.y,
        target_pose_in_world.orientation.z,
        target_pose_in_world.orientation.w);
    // orientation of the part in the tray, in world frame
    tf2::Quaternion q_target_part(
        part_in_world_frame.orientation.x,
        part_in_world_frame.orientation.y,
        part_in_world_frame.orientation.z,
        part_in_world_frame.orientation.w);

    // relative rotation between init and target
    tf2::Quaternion q_rot = q_target_part * q_init_part.inverse();
    // apply this rotation to the current gripper rotation
    tf2::Quaternion q_rslt = q_rot * q_current;
    q_rslt.normalize();

    // orientation of the gripper when placing the part in the tray
    part_in_world_frame.orientation.x = q_rslt.x();
    part_in_world_frame.orientation.y = q_rslt.y();
    part_in_world_frame.orientation.z = q_rslt.z();
    part_in_world_frame.orientation.w = q_rslt.w();
    part_in_world_frame.position.z += 0.09;

    
    ros::Duration(2).sleep();
    ROS_INFO_STREAM("Placing part at : " << part_in_world_frame);
    gantry_robot_arm_moveit_group.setPoseTarget(part_in_world_frame);
    gantry_robot_arm_moveit_group.move();
    ros::Duration(0.5).sleep();
    deactivate_gripper();
    
    auto state = get_gripper_state();
    if (state.attached) {
        return true;
    }
    
    else
        return false;
}



bool GantryRobot::gantry_robot_pick_place_service_callback(group2_rwa4::kitting_part_details::Request &req, group2_rwa4::kitting_part_details::Response &res)
{   
    std::vector<double> gantry_robot_initial_joint_pose;
    std::string gantry_current_preset_location;

    //############################# Kitting Presets #################################################
    if(req.initial_preset_location == "bins0")
    {
        go_to_location(arm_preset_at_bins_set_0);
        pickup_part(req.part_pose, req.part_offset);
        go_to_location(arm_preset_at_bins_set_0);

    } else if(req.initial_preset_location == "bins1")
    {
        go_to_location(arm_preset_at_bins_set_1);
        pickup_part(req.part_pose, req.part_offset);
        go_to_location(arm_preset_at_bins_set_1);

    } else if(req.initial_preset_location == "agv1")
    {   
        go_to_location(arm_preset_at_agv1);
        pickup_part(req.part_pose, req.part_offset);
        go_to_location(arm_preset_at_agv1);      
    } else if(req.initial_preset_location == "agv2")
    {
        go_to_location(arm_preset_at_agv2);
        pickup_part(req.part_pose, req.part_offset);
        go_to_location(arm_preset_at_agv2);

    } else if(req.initial_preset_location == "agv3")
    {
        go_to_location(arm_preset_at_agv3);
        pickup_part(req.part_pose, req.part_offset);
        go_to_location(arm_preset_at_agv3);
    } else if(req.initial_preset_location == "agv4")
    {
        go_to_location(arm_preset_at_agv4);
        pickup_part(req.part_pose, req.part_offset);
        go_to_location(arm_preset_at_agv4);


    //############################# Assembly Presets #################################################
    }else if(req.initial_preset_location == "as1_agv1")
    {
        nh.getParam("/gantry_current_preset_location", gantry_current_preset_location);
        if(gantry_current_preset_location == "pre_as2" || gantry_current_preset_location == "pre_as3" || gantry_current_preset_location == "center_1")
        {
            go_to_location(arm_preset_at_center_1);
            go_to_location(arm_preset_at_center_0);
        }

        go_to_location(arm_preset_at_pre_as1);
        go_to_location(arm_preset_at_as1_agv1);
        pickup_part(req.part_pose, req.part_offset);
        go_to_location(arm_preset_at_pre_as1);

    }else if(req.initial_preset_location == "as1_agv2")
    {
        nh.getParam("/gantry_current_preset_location", gantry_current_preset_location);
        if(gantry_current_preset_location == "pre_as2" || gantry_current_preset_location == "pre_as4" || gantry_current_preset_location == "center_1")
        {
            go_to_location(arm_preset_at_center_1);
            go_to_location(arm_preset_at_center_0);
        }


        go_to_location(arm_preset_at_as1_agv2);
        pickup_part(req.part_pose, req.part_offset);      
        go_to_location(arm_preset_at_pre_as1);

    }else if(req.initial_preset_location == "as2_agv1")
    {
        nh.getParam("/gantry_current_preset_location", gantry_current_preset_location);
        if(gantry_current_preset_location == "pre_as2" || gantry_current_preset_location == "pre_as4" || gantry_current_preset_location == "center_0")
        {
            go_to_location(arm_preset_at_center_0);
            go_to_location(arm_preset_at_center_1);            
        }

        go_to_location(arm_preset_at_pre_as2);
        go_to_location(arm_preset_at_as2_agv1);
        pickup_part(req.part_pose, req.part_offset);     
        go_to_location(arm_preset_at_pre_as2);

    }else if(req.initial_preset_location == "as2_agv2")
    {

        nh.getParam("/gantry_current_preset_location", gantry_current_preset_location);
        if(gantry_current_preset_location == "pre_as1" || gantry_current_preset_location == "pre_as3" || gantry_current_preset_location == "center_0")
        {
            go_to_location(arm_preset_at_center_0);
            go_to_location(arm_preset_at_center_1);            
        }
        go_to_location(arm_preset_at_as2_agv2);
        pickup_part(req.part_pose, req.part_offset);      
        go_to_location(arm_preset_at_pre_as2);

    }else if(req.initial_preset_location == "as3_agv3")
    {   
        nh.getParam("/gantry_current_preset_location", gantry_current_preset_location);
        if(gantry_current_preset_location == "pre_as2" || gantry_current_preset_location == "pre_as4" || gantry_current_preset_location == "center_1")
        {
            go_to_location(arm_preset_at_center_1);
            go_to_location(arm_preset_at_center_0);
        }

        go_to_location(arm_preset_at_as3_agv3);
        pickup_part(req.part_pose, req.part_offset);       
        go_to_location(arm_preset_at_pre_as3);


    }else if(req.initial_preset_location == "as3_agv4")
    {   
        
        nh.getParam("/gantry_current_preset_location", gantry_current_preset_location);
        if(gantry_current_preset_location == "pre_as2" || gantry_current_preset_location == "pre_as4" || gantry_current_preset_location == "center_1")
        {
            ros::Duration(3).sleep();
            go_to_location(arm_preset_at_center_1);
            go_to_location(arm_preset_at_center_0);
        }

        go_to_location(arm_preset_at_pre_as3);
        go_to_location(arm_preset_at_as3_agv4);
        pickup_part(req.part_pose, req.part_offset);       
        go_to_location(arm_preset_at_pre_as3);

    }else if(req.initial_preset_location == "as4_agv3")
    {
        nh.getParam("/gantry_current_preset_location", gantry_current_preset_location);
        if(gantry_current_preset_location == "pre_as1" || gantry_current_preset_location == "pre_as3" || gantry_current_preset_location == "center_0")
        {
            ros::Duration(3).sleep();
            go_to_location(arm_preset_at_center_0);
            go_to_location(arm_preset_at_center_1);            
        }

        go_to_location(arm_preset_at_as3_agv4);
        pickup_part(req.part_pose, req.part_offset);       
        go_to_location(arm_preset_at_pre_as4);


    }else if(req.initial_preset_location == "as4_agv4")
    {
        nh.getParam("/gantry_current_preset_location", gantry_current_preset_location);
        if(gantry_current_preset_location == "pre_as1" || gantry_current_preset_location == "pre_as3" || gantry_current_preset_location == "center_0")
        {
            go_to_location(arm_preset_at_center_0);
            go_to_location(arm_preset_at_center_1);            
        }

        go_to_location(arm_preset_at_pre_as4);
        go_to_location(arm_preset_at_as4_agv4);
        pickup_part(req.part_pose, req.part_offset);      
        go_to_location(arm_preset_at_pre_as4);
    }



    //############################# Kitting Final Presets #################################################
    // if(req.final_preset_location == "bins0")
    // {
    //     go_to_location(arm_preset_at_bins_set_0);
    //     place_part(req.part_target_pose);
    //     go_to_location(arm_preset_at_bins_set_0);

    // } else if(req.final_preset_location == "bins1")
    // {
    //     go_to_location(arm_preset_at_bins_set_1);
    //     place_part(req.part_target_pose);
    //     go_to_location(arm_preset_at_bins_set_1);

    // } else if(req.final_preset_location == "agv1")
    // {   
    //     go_to_location(arm_preset_at_agv1);
    //     place_part(req.part_target_pose);
    //     go_to_location(arm_preset_at_agv1);

    // } else if(req.final_preset_location == "agv2")
    // {
    //     go_to_location(arm_preset_at_agv2);
    //     place_part(req.part_target_pose,, req.part_pose_in_world);
    //     go_to_location(arm_preset_at_agv2);

    // } else if(req.final_preset_location == "agv3")
    // {
    //     go_to_location(arm_preset_at_agv3);
    //     place_part(req.part_target_pose,, req.part_pose_in_world);
    //     go_to_location(arm_preset_at_agv3);

    // } else if(req.final_preset_location == "agv4")
    // {
    //     go_to_location(arm_preset_at_agv4);
    //     place_part(req.part_target_pose, , req.part_pose_in_world);
    //     go_to_location(arm_preset_at_agv4);

    // }


    if(req.final_preset_location == "as1")
    {   
        ROS_INFO_STREAM("Target part pose :" << req.part_target_pose);
        go_to_location(arm_preset_at_as1);
        place_part(req.part_target_pose);
        go_to_location(arm_preset_at_as1);
        go_to_location(arm_preset_at_pre_as1);

        nh.setParam("/gantry_current_preset_location", "pre_as1");

    } 
    else if(req.final_preset_location == "as2")
    {   
        go_to_location(arm_preset_at_as2);
        place_part(req.part_target_pose);
        go_to_location(arm_preset_at_as2);
        go_to_location(arm_preset_at_pre_as2);

        nh.setParam("/gantry_current_preset_location", "pre_as2");

    } 
    else if(req.final_preset_location == "as3")
    {
        go_to_location(arm_preset_at_as3);
        place_part(req.part_target_pose);
        go_to_location(arm_preset_at_as3);
        go_to_location(arm_preset_at_pre_as3);

        nh.setParam("/gantry_current_preset_location", "pre_as3");

    } 
    else if(req.final_preset_location == "as4")
    {
        go_to_location(arm_preset_at_as4);
        place_part(req.part_target_pose);
        go_to_location(arm_preset_at_as4);
        go_to_location(arm_preset_at_pre_as4);

        nh.setParam("/gantry_current_preset_location", "pre_as4");

    }    

    nh.setParam("/gantry_placed_part",true);


    return true;
    
}





