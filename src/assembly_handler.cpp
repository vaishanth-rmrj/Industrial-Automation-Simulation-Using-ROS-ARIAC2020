#include "assembly_handler.h"


AssemblyHandler::AssemblyHandler(ros::NodeHandle &node_handler)
{
    nh = node_handler;
    init();
}

void AssemblyHandler::init()
{
    ROS_INFO("AssemblyHandler initialized :)");

    order_assembly_shipment_client = nh.serviceClient<group2_rwa4::order_assembly_shipment_details>("/group2/get_order_assembly_shipment_details");
    order_completion_status_client = nh.serviceClient<group2_rwa4::order_completion_status>("/group2/order_completion_status");   
    gantry_robot_pick_place_client = nh.serviceClient<group2_rwa4::kitting_part_details>("/group2/gantry_robot_pick_and_place");

    assembly_task_client = nh.serviceClient<group2_rwa4::assembly_task>("/group2/assembly_task");

    logical_camera_as1_agv1_sub = nh.subscribe("/ariac/logical_camera_11", 10, &AssemblyHandler::logical_camera_as1_agv1_callback, this);
    logical_camera_as1_agv2_sub = nh.subscribe("/ariac/logical_camera_12", 10, &AssemblyHandler::logical_camera_as1_agv2_callback, this);
    logical_camera_as2_agv1_sub = nh.subscribe("/ariac/logical_camera_13", 10, &AssemblyHandler::logical_camera_as2_agv1_callback, this);
    logical_camera_as2_agv2_sub = nh.subscribe("/ariac/logical_camera_14", 10, &AssemblyHandler::logical_camera_as2_agv2_callback, this);
    logical_camera_as3_agv3_sub = nh.subscribe("/ariac/logical_camera_15", 10, &AssemblyHandler::logical_camera_as3_agv3_callback, this);
    logical_camera_as3_agv4_sub = nh.subscribe("/ariac/logical_camera_16", 10, &AssemblyHandler::logical_camera_as3_agv4_callback, this);
    logical_camera_as4_agv3_sub = nh.subscribe("/ariac/logical_camera_17", 10, &AssemblyHandler::logical_camera_as4_agv3_callback, this);
    logical_camera_as4_agv4_sub = nh.subscribe("/ariac/logical_camera_18", 10, &AssemblyHandler::logical_camera_as4_agv4_callback, this);

    ros::Duration(2).sleep();
    assembly_task_sub = nh.subscribe("/group2/assembly_task", 1, &AssemblyHandler::assembly_task_sub_callback, this);


    // perform_part_assembly();
}

void AssemblyHandler::assembly_task_sub_callback(group2_rwa4::Task assembly_task)
{
    ROS_INFO("Assembly Handler : Performing part assembly");    
    perform_part_assembly_v2(assembly_task);
    
}

void AssemblyHandler::find_part_v2(assembly_part_location *current_part, std::string part_type, std::string final_preset_location)
{

    ROS_INFO_STREAM("Finding part" << part_type);
    if (final_preset_location == "as1")
    {
        if(as1_agv1_parts.size() > 0)
        {   
            for(int i=0; i < as1_agv1_parts.size(); i++)
            {
                if(as1_agv1_parts[i].type == part_type)
                {
                    current_part->part_pose = as1_agv1_parts[i].pose;
                    current_part->initial_preset_location = "as1_agv1";
                    current_part->target_frame = "logical_camera_11_frame";
                    break;
                }
            }
            
        }else{
            for(int i=0; i < as1_agv2_parts.size(); i++)
            {              
                if(as1_agv2_parts[i].type == part_type)
                {
                    current_part->part_pose = as1_agv2_parts[i].pose;
                    current_part->initial_preset_location = "as1_agv2";
                    current_part->target_frame = "logical_camera_12_frame";
                    break;
                }
            }
        }
    }
    if (final_preset_location == "as2")
    {
        if(as2_agv1_parts.size() > 0)
        {

            for(int i=0; i < as2_agv1_parts.size(); i++)
            {
                if(as2_agv1_parts[i].type == part_type)
                {
                    current_part->part_pose = as2_agv1_parts[i].pose;
                    current_part->initial_preset_location = "as2_agv1";
                    current_part->target_frame = "logical_camera_13_frame";
                    break;
                }
            }                                    
        }else{
            for(int i=0; i < as2_agv2_parts.size(); i++)
            {
                if(as2_agv2_parts[i].type == part_type)
                {
                    current_part->part_pose = as2_agv2_parts[i].pose;
                    current_part->initial_preset_location = "as2_agv2";
                    current_part->target_frame = "logical_camera_14_frame";
                    break;
                }
            }
        }
    }
    if (final_preset_location == "as3")
    {
        if(as3_agv3_parts.size() > 0)
        {
            for(int i=0; i < as3_agv3_parts.size(); i++)
            {
                if(as3_agv3_parts[i].type == part_type)
                {
                    current_part->part_pose = as3_agv3_parts[i].pose;
                    current_part->initial_preset_location = "as3_agv3";
                    current_part->target_frame = "logical_camera_15_frame";
                    break;
            
                }
            }
            
        }else{

            for(int i=0; i < as3_agv4_parts.size(); i++)
            {
                if(as3_agv4_parts[i].type == part_type)
                {
                    current_part->part_pose = as3_agv4_parts[i].pose;
                    current_part->initial_preset_location = "as3_agv4";
                    current_part->target_frame = "logical_camera_16_frame";
                    break;
                }
            }            
        }
    }
    if (final_preset_location == "as4")
    {
        if(as4_agv3_parts.size() > 0)
        {

            for(int i=0; i < as4_agv3_parts.size(); i++)
            {
                if(as4_agv3_parts[i].type == part_type)
                {
                    current_part->part_pose = as4_agv3_parts[i].pose;
                    current_part->initial_preset_location = "as4_agv3";
                    current_part->target_frame = "logical_camera_17_frame";
                    break;
            
                }
            }                    
        }else{

            for(int i=0; i < as4_agv4_parts.size(); i++)
            {
                if(as4_agv4_parts[i].type == part_type)
                {
                    current_part->part_pose = as4_agv4_parts[i].pose;
                    current_part->initial_preset_location = "as4_agv4";
                    current_part->target_frame = "logical_camera_18_frame";
                    break;
                }

            }

        }

    }
    
}

void AssemblyHandler::perform_part_assembly_v2(group2_rwa4::Task assembly_task)
{
    nh.setParam("/is_gantry_robot_busy", true);

    if(assembly_task.task_type == "assembly")
    {   
        nh.setParam("/current_order_id", assembly_task.order_id);
        group2_rwa4::kitting_part_details gantry_srv_msg;

        // struct to get part location
        assembly_part_location assembly_part_location_info;
        find_part_v2(&assembly_part_location_info, assembly_task.part_type, assembly_task.final_preset_location);

        gantry_srv_msg.request.final_preset_location = assembly_task.final_preset_location;
        gantry_srv_msg.request.part_target_pose = assembly_task.part_target_pose;  
        gantry_srv_msg.request.initial_preset_location = assembly_part_location_info.initial_preset_location;

        if(assembly_part_location_info.part_pose.orientation.w == 0 || assembly_part_location_info.part_pose.orientation.z == 0)
        {
            assembly_part_location_info.part_pose.orientation.w = 1.0;
            assembly_part_location_info.part_pose.orientation.z = 0;
        }

        gantry_srv_msg.request.part_pose = motioncontrol::get_part_target_pose(assembly_part_location_info.part_pose, assembly_part_location_info.target_frame, "assembly_part_"+std::to_string(child_frame_id));
        child_frame_id += 1;

        // setting part offset
        std::string delimiter = "_";
        std::vector<std::string> split_words{};
        std::size_t pos;
        while ((pos = assembly_task.part_type.find(delimiter)) != std::string::npos) {
            split_words.push_back(assembly_task.part_type.substr(0, pos));
            assembly_task.part_type.erase(0, pos + delimiter.length());
        }
        std::string part_name = split_words[1];        

        if(part_name == "battery")
        {
            gantry_srv_msg.request.part_offset = 0.042;
            gantry_srv_msg.request.part_type = part_name;
        } 
        else if(part_name == "sensor")
        {
            gantry_srv_msg.request.part_offset = 0.06;
            gantry_srv_msg.request.part_type = part_name;
        }
        else if(part_name == "regulator")
        {
            gantry_srv_msg.request.part_offset = 0.06;
            gantry_srv_msg.request.part_type = part_name;
        }
        else if(part_name == "pump")
        {
            gantry_srv_msg.request.part_offset = 0.07;
            gantry_srv_msg.request.part_type = part_name;
        }   

        ROS_INFO_STREAM(gantry_srv_msg.request);     

        if (!gantry_robot_pick_place_client.call(gantry_srv_msg))
        {
            ROS_INFO("Gantry part pick and place srv called");
            gantry_robot_pick_place_client.call(gantry_srv_msg);
            
            
        }
        ROS_INFO_STREAM("Assembly task completed");

        task_completed_srv_client = nh.serviceClient<std_srvs::Trigger>("/group2/assembly_task_completed");
        std_srvs::Trigger task_completed_srv_msg;
        task_completed_srv_client.call(task_completed_srv_msg);
    }
}

void AssemblyHandler::find_part(assembly_part_location *current_part, std::string required_part, std::string final_preset_location)
{    
}


void AssemblyHandler::perform_part_assembly()
{
}


geometry_msgs::Pose AssemblyHandler::transform_to_world_frame(const geometry_msgs::Pose& part_pose, std::string logicam_frame, std::string child_frame) 
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = logicam_frame;
    transformStamped.child_frame_id = child_frame;
    transformStamped.transform.translation.x = part_pose.position.x;
    transformStamped.transform.translation.y = part_pose.position.y;
    transformStamped.transform.translation.z = part_pose.position.z;
    transformStamped.transform.rotation.x = part_pose.orientation.x;
    transformStamped.transform.rotation.y = part_pose.orientation.y;
    transformStamped.transform.rotation.z = part_pose.orientation.z;
    transformStamped.transform.rotation.w = part_pose.orientation.w;

    // broadcasting transformed frame
    for (int i{ 0 }; i < 10; ++i)
        br.sendTransform(transformStamped);

    // ros::Duration(60.0).sleep();

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(15);
    // ros::Duration timeout(3.0);
    geometry_msgs::TransformStamped world_target_tf;
    ros::Duration(2.0).sleep();


    for (int i = 0; i < 10; i++) {
        try {
            
            world_target_tf = tfBuffer.lookupTransform("world", child_frame, ros::Time(0), ros::Duration(5));

        }
        catch (tf2::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

      
    }

    geometry_msgs::Pose world_target{};

    world_target.position.x = world_target_tf.transform.translation.x;
    world_target.position.y = world_target_tf.transform.translation.y;
    world_target.position.z = world_target_tf.transform.translation.z;
    world_target.orientation.x = world_target_tf.transform.rotation.x;
    world_target.orientation.y = world_target_tf.transform.rotation.y;
    world_target.orientation.z = world_target_tf.transform.rotation.z;
    world_target.orientation.w = world_target_tf.transform.rotation.w;

    ROS_INFO_STREAM("World transform for part  "<< world_target);
    
    return world_target;
}


void AssemblyHandler::logical_camera_as1_agv1_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    int sim_time = (int)round(ros::Time::now().toSec());
    if(logicam_msg->models.size() > 0 && sim_time%2 == 0)
    {
        as1_agv1_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            as1_agv1_parts.push_back(logicam_msg->models[i]);
        }
        as1_agv1_parts_count = as1_agv1_parts.size();

    }
}

void AssemblyHandler::logical_camera_as1_agv2_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    int sim_time = (int)round(ros::Time::now().toSec());
    if(logicam_msg->models.size() > 0 && sim_time%2 == 0)
    {
        as1_agv2_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            as1_agv2_parts.push_back(logicam_msg->models[i]);

        }
        as1_agv2_parts_count = as1_agv2_parts.size();

    }
}

void AssemblyHandler::logical_camera_as2_agv1_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    int sim_time = (int)round(ros::Time::now().toSec());
    if(logicam_msg->models.size() > 0 && sim_time%2 == 0)
    {
        as2_agv1_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            as2_agv1_parts.push_back(logicam_msg->models[i]);
        }
        as2_agv1_parts_count = as2_agv1_parts.size();

    }
}

void AssemblyHandler::logical_camera_as2_agv2_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    int sim_time = (int)round(ros::Time::now().toSec());
    if(logicam_msg->models.size() > 0 && sim_time%2 == 0)
    {
        as2_agv2_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            as2_agv2_parts.push_back(logicam_msg->models[i]);
        }
        as2_agv2_parts_count = as2_agv2_parts.size();

    }
}
void AssemblyHandler::logical_camera_as3_agv3_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    int sim_time = (int)round(ros::Time::now().toSec());
    if(logicam_msg->models.size() > 0 && sim_time%2 == 0)
    {        
        as3_agv3_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            as3_agv3_parts.push_back(logicam_msg->models[i]);
        }
        as3_agv3_parts_count = as3_agv3_parts.size();

    }
}

void AssemblyHandler::logical_camera_as3_agv4_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    int sim_time = (int)round(ros::Time::now().toSec());
    if(logicam_msg->models.size() > 0 && sim_time%2 == 0)
    {
        as3_agv4_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            as3_agv4_parts.push_back(logicam_msg->models[i]);
        }
        as3_agv4_parts_count = as3_agv4_parts.size();

    }
}

void AssemblyHandler::logical_camera_as4_agv3_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    int sim_time = (int)round(ros::Time::now().toSec());
    if(logicam_msg->models.size() > 0 && sim_time%2 == 0)
    {
        as4_agv3_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            as4_agv3_parts.push_back(logicam_msg->models[i]);
        }
        as4_agv3_parts_count = as4_agv3_parts.size();

    }
}

void AssemblyHandler::logical_camera_as4_agv4_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{
    int sim_time = (int)round(ros::Time::now().toSec());
    if(logicam_msg->models.size() > 0 && sim_time%2 == 0)
    {
        as4_agv4_parts.clear();
        for(int i = 0; i < logicam_msg->models.size(); i++)
        {            
            as4_agv4_parts.push_back(logicam_msg->models[i]);
        }
        as4_agv4_parts_count = as4_agv4_parts.size();
    }
}