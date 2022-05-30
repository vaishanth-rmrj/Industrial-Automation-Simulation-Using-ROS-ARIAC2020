#include <ros/ros.h>
#include "kitting_handler.h"
#include "order_handler.h"
#include <string.h>

#include <group2_rwa4/list_all_parts.h>
#include "sensor_array.h"


#include "kitting_robot.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <tf2/convert.h>




// This passes the parameters necessary for the planning group in the rviz gui
KittingHandler::KittingHandler(ros::NodeHandle& node) 
{
    nh = node;
    init();    
}

void KittingHandler::init()
{
    ROS_INFO("KittingHandler initialized :)");
    order_kitting_shipment_client = nh.serviceClient<group2_rwa4::order_kitting_shipment_details>("/group2/get_order_kitting_shipment_details");
    sensor_bins0_client = nh.serviceClient<group2_rwa4::check_exists>("/group2/check_part_bins0");
    sensor_bins1_client = nh.serviceClient<group2_rwa4::check_exists>("/group2/check_part_bins1");
    kitting_robot_pick_place_client = nh.serviceClient<group2_rwa4::kitting_part_details>("/group2/kitting_robot_pick_and_place"); 
    order_completion_status_client = nh.serviceClient<group2_rwa4::order_completion_status>("/group2/order_completion_status");   

    kitting_task_sub = nh.subscribe("/group2/kitting_task", 1, &KittingHandler::kitting_task_sub_callback, this);
    task_completed_srv_client = nh.serviceClient<std_srvs::Trigger>("/group2/task_completed");
    dispose_faulty_srv_client = nh.serviceClient<std_srvs::Trigger>("/group2/dispose_faulty");
    gantry_robot_pick_place_client = nh.serviceClient<group2_rwa4::kitting_part_details>("/group2/gantry_robot_pick_and_place");

    // sensor_bins0_client = nh.serviceClient<group2_rwa4::check_exists>("/group2/check_part_bins0");
    // sensor_bins1_client = nh.serviceClient<group2_rwa4::check_exists>("/group2/check_part_bins1");
    
}

void KittingHandler::kitting_task_sub_callback(group2_rwa4::Task kitting_task)
{
    ROS_INFO_STREAM(kitting_task);
    perform_part_kitting(kitting_task);
    
}

bool KittingHandler::process_order_kitting()
{
    if (!order_kitting_shipment_client.exists())
    {
        order_kitting_shipment_client.waitForExistence();
    };

    while (true)
    {
        group2_rwa4::order_kitting_shipment_details order_kitting_shipment_details_srv_msg; 
        group2_rwa4::order_completion_status order_completion_status_srv_msg;      


        order_kitting_shipment_client.call(order_kitting_shipment_details_srv_msg); 

        if(order_kitting_shipment_details_srv_msg.response.kitting_shipment_present)
        {    
            kitting_shipment_details = order_kitting_shipment_details_srv_msg.response;  

            perform_order_kitting();

            order_completion_status_srv_msg.request.order_id = kitting_shipment_details.order_id;
            order_completion_status_srv_msg.request.completion_status = true;
            order_completion_status_client.call(order_completion_status_srv_msg);
        }
        else
        {
            ROS_INFO("Waiting 10 seconds for next order to arrive");
            ros::Duration(10).sleep();
            order_kitting_shipment_client.call(order_kitting_shipment_details_srv_msg); 
            if(!order_kitting_shipment_details_srv_msg.response.kitting_shipment_present)
            {   
                ROS_INFO("All orders processed :)");
                ROS_INFO("XXX ENDING COMPETION XXX");
                break;

            }
        }
    }
    

          
}



void KittingHandler::perform_part_kitting(group2_rwa4::Task kitting_task)
{     
    ROS_INFO("Performing part kitting");
    nh.setParam("/is_kitting_robot_busy", true);
    if(kitting_task.task_type == "kitting")
    {
        group2_rwa4::kitting_part_details kitting_part_details_srv_msg;
        kitting_part_details_srv_msg.request.initial_preset_location = kitting_task.initial_preset_location;
        kitting_part_details_srv_msg.request.final_preset_location = kitting_task.final_preset_location;
        kitting_part_details_srv_msg.request.part_pose = kitting_task.part_pose;
        kitting_part_details_srv_msg.request.part_target_pose = kitting_task.part_target_pose;
        
        // spliting the part type string
        std::string part_type = kitting_task.part_type;
        std::string delimiter = "_";
        std::vector<std::string> split_words{};

        std::size_t pos;
        while ((pos = part_type.find(delimiter)) != std::string::npos) {
            split_words.push_back(part_type.substr(0, pos));
            part_type.erase(0, pos + delimiter.length());
        }

        std::string part_name = split_words[1];

        // setting part offset pose value
        if(part_name == "battery")
        {
            kitting_part_details_srv_msg.request.part_offset = 0.045;
        } 
        else if(part_name == "sensor")
        {
            kitting_part_details_srv_msg.request.part_offset = 0.053;
        }
        else if(part_name == "regulator")
        {
            kitting_part_details_srv_msg.request.part_offset = 0.06;
        }
        else
        {
            kitting_part_details_srv_msg.request.part_offset = 0.08;
        } 

        ROS_INFO_STREAM("Moving to part offset("<< kitting_part_details_srv_msg.request.part_offset<<") location for pickup");
    
        ///////////////////////////////////////////////Sai//////////////////////////////////////////////////////////////////////
        if(kitting_task.part_pose.position.x > (-2.2753))
            {
                if(!kitting_robot_pick_place_client.call(kitting_part_details_srv_msg))
                {
                    ROS_INFO("Retrying part pick and place");
                    kitting_robot_pick_place_client.call(kitting_part_details_srv_msg);
                }
            }

        if(kitting_task.part_pose.position.x < (-2.2753))

        {
            ROS_INFO_STREAM(kitting_part_details_srv_msg.request);
            if(!gantry_robot_pick_place_client.call(kitting_part_details_srv_msg))
            {
                ROS_INFO("Retrying part pick and place");
                gantry_robot_pick_place_client.call(kitting_part_details_srv_msg);
            }
        }
        ///////////////////////////////////////////////Sai//////////////////////////////////////////////////////////////////////
        group2_rwa4::check_agv_faulty_parts agv_faulty_part_srv_msg;

        // checking for faulty part on the agv
        std::string agv_id = kitting_task.final_preset_location;
        ros::ServiceClient agv_qc_client = nh.serviceClient<group2_rwa4::check_agv_faulty_parts>("/group2/check_"+agv_id+"_faulty_parts");

        agv_faulty_part_srv_msg.request.agv_id = agv_id;
        agv_qc_client.call(agv_faulty_part_srv_msg);

        ROS_INFO("Checking for faulty part");
        ROS_INFO_STREAM(agv_faulty_part_srv_msg.response.faulty_part_poses.size());

        // delay to add task having higher priority
        nh.setParam("/is_kitting_robot_busy", false);
        ros::Duration(2).sleep();

        if(agv_faulty_part_srv_msg.response.faulty_part_poses.size() > 0)
        {   
            std_srvs::Trigger dispose_faulty_srv_msg;
            dispose_faulty_srv_client.call(dispose_faulty_srv_msg);
        }
        else
        {
            std_srvs::Trigger task_completed_srv_msg;
            task_completed_srv_client.call(task_completed_srv_msg);
        }
        
    }

    if(kitting_task.task_type == "dispose_faulty")
    {    

        group2_rwa4::dispose_faulty_part faulty_part_pose;
        faulty_part_pose.request.part_pose = kitting_task.part_target_pose;

        // spliting the part type string
        std::string part_type = kitting_task.part_type;
        std::string delimiter = "_";
        std::vector<std::string> split_words{};

        std::size_t pos;
        while ((pos = part_type.find(delimiter)) != std::string::npos) {
            split_words.push_back(part_type.substr(0, pos));
            part_type.erase(0, pos + delimiter.length());
        }

        std::string part_name = split_words[1];

        // setting part offset pose value
        if(part_name == "battery")
        {
            faulty_part_pose.request.part_offset = 0.045;
        } 
        else if(part_name == "sensor")
        {
            faulty_part_pose.request.part_offset = 0.09;
        }
        else if(part_name == "regulator")
        {
            faulty_part_pose.request.part_offset = 0.06;
        }
        else
        {
            faulty_part_pose.request.part_offset = 0.08;
        } 

        ROS_INFO_STREAM("Moving to part offset("<< faulty_part_pose.request.part_offset<<") location for pickup");
    
        ros::ServiceClient kitting_robot_dispose_faulty_client = nh.serviceClient<group2_rwa4::dispose_faulty_part>("/group2/kitting_dispose_faulty");
        
        ROS_INFO("Disposing faulty part");
        kitting_robot_dispose_faulty_client.call(faulty_part_pose);

        // delay to add task having higher priority
        nh.setParam("/is_kitting_robot_busy", false);
        ros::Duration(2).sleep();

        std_srvs::Trigger task_completed_srv_msg;
        task_completed_srv_client.call(task_completed_srv_msg);
    }

    if(kitting_task.task_type == "conveyor_task")
    {
        group2_rwa4::kitting_part_details kitting_part_details_srv_msg;
        kitting_part_details_srv_msg.request.initial_preset_location = kitting_task.initial_preset_location;
        kitting_part_details_srv_msg.request.final_preset_location = kitting_task.final_preset_location;
        kitting_part_details_srv_msg.request.part_target_pose = kitting_task.part_target_pose;
        
        // spliting the part type string
        std::string part_type = kitting_task.part_type;
        std::string delimiter = "_";
        std::vector<std::string> split_words{};

        std::size_t pos;
        while ((pos = part_type.find(delimiter)) != std::string::npos) {
            split_words.push_back(part_type.substr(0, pos));
            part_type.erase(0, pos + delimiter.length());
        }

        std::string part_name = split_words[1];

        // setting part offset pose value
        if(part_name == "battery")
        {
            kitting_part_details_srv_msg.request.part_offset = 0.045;
        } 
        else if(part_name == "sensor")
        {
            kitting_part_details_srv_msg.request.part_offset = 0.053;
        }
        else if(part_name == "regulator")
        {
            kitting_part_details_srv_msg.request.part_offset = 0.06;
        }
        else
        {
            kitting_part_details_srv_msg.request.part_offset = 0.08;
        } 

        ROS_INFO_STREAM("Moving to part offset("<< kitting_part_details_srv_msg.request.part_offset<<") location for pickup");
    
        
        if(!kitting_robot_pick_place_client.call(kitting_part_details_srv_msg))
        {
            ROS_INFO("Retrying part pick and place");
            kitting_robot_pick_place_client.call(kitting_part_details_srv_msg);
        }       

        
        std_srvs::Trigger task_completed_srv_msg;
        task_completed_srv_client.call(task_completed_srv_msg);
        
        
    }



}


bool KittingHandler::perform_order_kitting()
{
    if (!sensor_bins0_client.exists())
    {
        sensor_bins0_client.waitForExistence();
    };

    if (!sensor_bins1_client.exists())
    {
        sensor_bins1_client.waitForExistence();
    };

    

    // int kitting_part_left = 0;

    // for(int j=0; j< kitting_shipment_details.kitting_shipments.size(); j++)
    // {
    //     ROS_INFO_STREAM("kitting_part_found  "<< kitting_shipment_details.kitting_shipments[j].products.size());
    //     kitting_part_left += kitting_shipment_details.kitting_shipments[j].products.size();
    // }

    // ROS_INFO_STREAM("kitting_part_left  "<<kitting_part_left);

       
    for(int j=0; j< kitting_shipment_details.kitting_shipments.size(); j++)
    {   
        int kitting_part_left = kitting_shipment_details.kitting_shipments[j].products.size();
        while (kitting_part_left > 0)
        { 
            for(int i=0; i< kitting_shipment_details.kitting_shipments[j].products.size(); i++)
            {
                if(!kitting_shipment_details.kitting_shipments[j].products[i].part_kitting_success)
                {   
                    
                    ROS_INFO_STREAM("Kitting product type : "<< kitting_shipment_details.kitting_shipments[j].products[i].type);
                    ROS_INFO_STREAM("Kitting on : "<< kitting_shipment_details.kitting_shipments[j].agv_id);
                    group2_rwa4::check_exists bins0_srv_msg;
                    group2_rwa4::check_exists bins1_srv_msg;

                    group2_rwa4::kitting_part_details kitting_part_details_srv_msg;
                    

                    bins0_srv_msg.request.part_type = kitting_shipment_details.kitting_shipments[j].products[i].type;
                    sensor_bins0_client.call(bins0_srv_msg);

                    bins1_srv_msg.request.part_type = kitting_shipment_details.kitting_shipments[j].products[i].type;
                    sensor_bins1_client.call(bins1_srv_msg);

                    geometry_msgs::Pose desired_part_pose_in_agv = kitting_shipment_details.kitting_shipments[j].products[i].pose;


                    int kit_tray_number = 1;
                    if(kitting_shipment_details.kitting_shipments[j].agv_id == "agv2")
                    {
                        kit_tray_number = 2;
                    }
                    else if(kitting_shipment_details.kitting_shipments[j].agv_id == "agv3")
                    {
                        kit_tray_number = 3;
                    }
                    else if(kitting_shipment_details.kitting_shipments[j].agv_id == "agv4")
                    {
                        kit_tray_number = 4;
                    }
                    else{}                    
                    std::string target_frame = "kit_tray_"+std::to_string(kit_tray_number);                    

                    geometry_msgs::Pose target_part_pose = get_part_target_pose(desired_part_pose_in_agv, target_frame, "frame"+i);


                    if (bins0_srv_msg.response.success)
                    {
                        ROS_INFO_STREAM(kitting_shipment_details.kitting_shipments[j].products[i].type << " part found in Bins 0");

                        kitting_part_details_srv_msg.request.initial_preset_location = "bins0";
                        kitting_part_details_srv_msg.request.part_pose = bins0_srv_msg.response.part_poses[0]; 
                        kitting_part_details_srv_msg.request.final_preset_location = kitting_shipment_details.kitting_shipments[j].agv_id;
                        kitting_part_details_srv_msg.request.part_target_pose = target_part_pose;  


                    }
                    else if (bins1_srv_msg.response.success)
                    {
                        ROS_INFO_STREAM(kitting_shipment_details.kitting_shipments[j].products[i].type << " part found in Bins 1");

                        kitting_part_details_srv_msg.request.initial_preset_location = "bins1";
                        kitting_part_details_srv_msg.request.part_pose = bins1_srv_msg.response.part_poses[0]; 
                        kitting_part_details_srv_msg.request.final_preset_location = kitting_shipment_details.kitting_shipments[j].agv_id;
                        kitting_part_details_srv_msg.request.part_target_pose = target_part_pose; 
                    }

                    std::string part_type = kitting_shipment_details.kitting_shipments[j].products[i].type;
                    std::string delimiter = "_";
                    std::vector<std::string> split_words{};

                    std::size_t pos;
                    while ((pos = part_type.find(delimiter)) != std::string::npos) {
                        split_words.push_back(part_type.substr(0, pos));
                        part_type.erase(0, pos + delimiter.length());
                    }

                    std::string part_name = split_words[1];

                    if(part_name == "battery")
                    {
                        kitting_part_details_srv_msg.request.part_offset = 0.045;
                    } 
                    else if(part_name == "sensor")
                    {
                        kitting_part_details_srv_msg.request.part_offset = 0.053;
                    }
                    else if(part_name == "regulator")
                    {
                        kitting_part_details_srv_msg.request.part_offset = 0.06;
                    }
                    else
                    {
                        kitting_part_details_srv_msg.request.part_offset = 0.08;
                    }           

                    ROS_INFO_STREAM("Moving to part offset("<< kitting_part_details_srv_msg.request.part_offset<<") location for pickup");

                    if(bins0_srv_msg.response.parts_present_in_bin0 || bins0_srv_msg.response.parts_present_in_bin1 || bins1_srv_msg.response.parts_present_in_bin4 || bins1_srv_msg.response.parts_present_in_bin5)
                        {
                            if(!kitting_robot_pick_place_client.call(kitting_part_details_srv_msg))
                            {
                                ROS_INFO("Retrying part pick and place");
                                kitting_robot_pick_place_client.call(kitting_part_details_srv_msg);
                            }
                        }

                    if(bins0_srv_msg.response.parts_present_in_bin2 || bins0_srv_msg.response.parts_present_in_bin3 || bins1_srv_msg.response.parts_present_in_bin6 || bins1_srv_msg.response.parts_present_in_bin7)
                    {
                        ROS_INFO_STREAM(kitting_part_details_srv_msg.request);
                        if(!gantry_robot_pick_place_client.call(kitting_part_details_srv_msg))
                        {
                            ROS_INFO("Retrying part pick and place");
                            gantry_robot_pick_place_client.call(kitting_part_details_srv_msg);
                        }
                    }

                    group2_rwa4::check_agv_faulty_parts agv_faulty_part_srv_msg;

                    // checking for faulty part on the agv
                    std::string agv_id = kitting_shipment_details.kitting_shipments[j].agv_id;
                    ros::ServiceClient agv_qc_client = nh.serviceClient<group2_rwa4::check_agv_faulty_parts>("/group2/check_"+agv_id+"_faulty_parts");

                    agv_faulty_part_srv_msg.request.agv_id = agv_id;
                    agv_qc_client.call(agv_faulty_part_srv_msg);

                    if(agv_faulty_part_srv_msg.response.faulty_part_poses.size() > 0)
                    {   
                        
                        ROS_INFO("Faulty part detected !!");

                        int sensor_id = 4;
                        if(kitting_shipment_details.kitting_shipments[j].agv_id == "agv1")
                        {
                            sensor_id = 1;

                        } else if(kitting_shipment_details.kitting_shipments[j].agv_id == "agv2")
                        {
                            sensor_id = 2;

                        } else if(kitting_shipment_details.kitting_shipments[j].agv_id == "agv3")
                        {
                            sensor_id = 3;

                        } else {}


                        std::string agv_qc_frame = "quality_control_sensor_"+std::to_string(sensor_id)+"_frame";
                        geometry_msgs::Pose qc_faulty_part_pose = agv_faulty_part_srv_msg.response.faulty_part_poses[0];
                        geometry_msgs::Pose faulty_part_world_pose = get_part_target_pose(qc_faulty_part_pose, agv_qc_frame, "faulty_frame"+i); 

                        group2_rwa4::dispose_faulty_part faulty_part_pose;
                        faulty_part_pose.request.part_pose = faulty_part_world_pose;
                        faulty_part_pose.request.part_offset = kitting_part_details_srv_msg.request.part_offset;

                        ros::ServiceClient kitting_robot_dispose_faulty_client = nh.serviceClient<group2_rwa4::dispose_faulty_part>("/group2/kitting_dispose_faulty");
                        
                        ROS_INFO("Disposing faulty part");
                        kitting_robot_dispose_faulty_client.call(faulty_part_pose);
                    }
                    else
                    {
                        kitting_part_left -= 1;
                        kitting_shipment_details.kitting_shipments[j].products[i].part_kitting_success = true;
                    }

                    ROS_INFO_STREAM("Parts remaining to kit : "<< kitting_part_left);
                }
            }
                 

        }

        ROS_INFO("Kitting done!!");  
        nh.setParam("/"+kitting_shipment_details.kitting_shipments[j].agv_id+"_ready_to_ship", true); // updating param server

        // ROS_INFO_STREAM("Submitting "<<kitting_shipment_details.kitting_shipments[j].agv_id<<" to : "<< kitting_shipment_details.kitting_shipments[j].station_id);
        // submit_agv(kitting_shipment_details.kitting_shipments[j].agv_id, 
        //             kitting_shipment_details.kitting_shipments[j].station_id, 
        //             kitting_shipment_details.kitting_shipments[j].shipment_type);
        

    }   
    return true;
}

void KittingHandler::submit_agv(std::string agv_id, std::string assembly_station, std::string shipment_type)
{
    // sumbitting agv
    ros::ServiceClient shipment_submit_client = nh.serviceClient<nist_gear::AGVToAssemblyStation>("/ariac/"+agv_id+"/submit_shipment");
    if (!shipment_submit_client.exists())
    {
        shipment_submit_client.waitForExistence();
    }

    ROS_INFO("Requesting the service...");
    nist_gear::AGVToAssemblyStation srv_msg;
    srv_msg.request.assembly_station_name = assembly_station;
    srv_msg.request.shipment_type = shipment_type;

    shipment_submit_client.call(srv_msg);

    if (!srv_msg.response.success)
    {
        ROS_ERROR_STREAM("Failed to submit agv"<< agv_id <<" to assembly station " << srv_msg.request.assembly_station_name);
    }
    else
    {
        ROS_INFO_STREAM(agv_id << " moving to: " << srv_msg.request.assembly_station_name);
    }
}

geometry_msgs::Pose KittingHandler::get_part_target_pose(const geometry_msgs::Pose& target_pose, std::string target_frame, std::string child_frame_id) 
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = target_frame;
    transformStamped.child_frame_id = child_frame_id;
    transformStamped.transform.translation.x = target_pose.position.x;
    transformStamped.transform.translation.y = target_pose.position.y;
    transformStamped.transform.translation.z = target_pose.position.z;
    transformStamped.transform.rotation.x = target_pose.orientation.x;
    transformStamped.transform.rotation.y = target_pose.orientation.y;
    transformStamped.transform.rotation.z = target_pose.orientation.z;
    transformStamped.transform.rotation.w = target_pose.orientation.w;

    // broadcasting transformed frame
    for (int i{ 0 }; i < 15; ++i)
        br.sendTransform(transformStamped);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10);
    ros::Duration timeout(1.0);

    geometry_msgs::TransformStamped world_target_tf;

    for (int i = 0; i < 10; i++) {
        try {
            world_target_tf = tfBuffer.lookupTransform("world", child_frame_id, ros::Time(0), timeout);
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

    return world_target;
}































// #include <ros/ros.h>
// #include "kitting_handler.h"
// #include "order_handler.h"
// #include <string.h>
// #include <group2_rwa4/order_kitting_shipment_details.h>
// #include <group2_rwa4/list_all_parts.h>
// #include "sensor_array.h"


// int main(int argc, char ** argv)
// {   
//     ros::init(argc, argv, "kitting_handler");

//     ros::NodeHandle nh;
//     ros::ServiceClient order_client;
//     ros::ServiceClient sensor_conveyor_client;
//     ros::ServiceClient sensor_bins_client;
//     ros::ServiceClient sensor_agv_qc_client;
//     ros::ServiceClient sensor_agv_lcam_client;

//     group2_rwa4::order_kitting_shipment_details::Response kitting_shipment_details;
//     group2_rwa4::list_all_parts::Response conveyor_parts;
//     group2_rwa4::list_all_parts::Response bins_parts;
//     group2_rwa4::list_all_parts::Response agv_lcam_parts;
//     group2_rwa4::list_all_parts::Response agv_qc_parts;

//     // OrderHandler order_handler(nh);
//     // sensor_array::BinSensors bins_sensors(nh);
//     // sensor_array::AgvSensors agv_sensors(nh);
//     // sensor_array::ConveyorSensors conveyor_sensor(nh);

//     ros::AsyncSpinner spinner(2);
//     spinner.start();

//     // while (ros::ok())
//     // {
//             order_client = nh.serviceClient<group2_rwa4::order_kitting_shipment_details>("/group2/get_order_kitting_shipment_details");
            

//             group2_rwa4::order_kitting_shipment_details srv; 
//             order_client.call(srv); 
//             kitting_shipment_details = srv.response;    
//             ROS_INFO_STREAM("order client  : "<<kitting_shipment_details);

//             sensor_conveyor_client = nh.serviceClient<group2_rwa4::list_all_parts>("/group2/list_all_parts_conveyor");
//             sensor_bins_client = nh.serviceClient<group2_rwa4::list_all_parts>("/group2/list_all_parts_bins");
//             sensor_agv_lcam_client = nh.serviceClient<group2_rwa4::list_all_parts>("/group2/list_all_parts_agv_lcam");
//             sensor_agv_qc_client = nh.serviceClient<group2_rwa4::list_all_parts>("/group2/list_all_parts_agv_qc");

//             group2_rwa4::list_all_parts srv_conveyor;
//             sensor_conveyor_client.call(srv_conveyor);
//             conveyor_parts = srv_conveyor.response;
//             ROS_INFO_STREAM("Conveyor client  : "<<conveyor_parts);

//             group2_rwa4::list_all_parts srv_bins;
//             std::string a{"1"};
//             srv_bins.request.request=a;
//             sensor_bins_client.call(srv_bins);
//             bins_parts = srv_bins.response;
//             ROS_INFO_STREAM("Bins client  : "<<bins_parts);
            
//             group2_rwa4::list_all_parts srv_agv_lcam;
//             std::string b{"2"};
//             srv_bins.request.request=b;
//             sensor_agv_lcam_client.call(srv_agv_lcam);
//             agv_lcam_parts = srv_agv_lcam.response;
//             ROS_INFO_STREAM("AGV Lcam client  : "<<agv_lcam_parts);
            
//             group2_rwa4::list_all_parts srv_agv_qc;
//             std::string c{"2"};
//             srv_agv_qc.request.request = c;
//             sensor_agv_qc_client.call(srv_agv_qc);
//             agv_qc_parts = srv_agv_qc.response;
//             ROS_INFO_STREAM("AGV QC client : "<<agv_qc_parts);


//     // }
    

//     ros::waitForShutdown();
// }