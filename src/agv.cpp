#include "agv.h"

AGV::AGV(ros::NodeHandle &node_handler, int id)
{
    nh = node_handler;    
    agv_id = std::to_string(id);
    agv_name = "agv"+ std::to_string(id);

    init();
}

void AGV::init()
{   
    ROS_INFO_STREAM(agv_name<< " is initialized :)" );
    quality_sensor_sub = nh.subscribe("/ariac/quality_control_sensor_"+agv_id, 10, &AGV::check_faulty_part, this);
    check_agv_faulty_part_service = nh.advertiseService("/group2/check_agv"+agv_id+"_faulty_parts", &AGV::check_agv_faulty_part_service_callback, this);

    agv_logicam_sub = nh.subscribe("/ariac/logical_camera_agv"+agv_id, 10, &AGV::agv_logicam_callback, this);

}


void AGV::check_faulty_part(const nist_gear::LogicalCameraImage::ConstPtr &quality_sensor_msg)
{   
    if(agv_station == "ks"+agv_id)
    {
        if(quality_sensor_msg->models.size() > 0)
        {
            ROS_INFO_STREAM_ONCE("Faulty Part detected in "<< agv_name);
            faulty_part_count = quality_sensor_msg->models.size();
            faulty_part_poses.clear();
            for(int i=0; i< quality_sensor_msg->models.size(); i++)
            {

                faulty_part_poses.push_back(quality_sensor_msg->models[i].pose);                
                
            }
        }
        else
        {
            faulty_part_count = 0;
        }
    }   
}


bool AGV::check_agv_faulty_part_service_callback(group2_rwa4::check_agv_faulty_parts::Request &req, group2_rwa4::check_agv_faulty_parts::Response &res)
{       
          
    if(agv_name == req.agv_id)
    {
        res.success = true;
        if(faulty_part_count > 0)
        {
            res.faulty_part_count = std::to_string(faulty_part_count);
            for(int i=0; i< faulty_part_poses.size(); i++)
            {   
                res.faulty_part_poses.push_back(faulty_part_poses[i]);                
            }
        }
        else
        {
            res.faulty_part_count = std::to_string(0);
        }
        
    }
       
    return true;
}




void AGV::agv_logicam_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg)
{    
}

// void AGV::submit_agv(std::string assembly_station, std::string shipment_type)
// {
//     // sumbitting agv    
//     if (!shipment_submit_client.exists())
//     {
//         shipment_submit_client.waitForExistence();
//     }

//     ROS_INFO("Requesting the service...");
//     nist_gear::AGVToAssemblyStation srv_msg;
//     srv_msg.request.assembly_station_name = assembly_station;
//     srv_msg.request.shipment_type = shipment_type;

//     ros::Duration(2).sleep();
//     shipment_submit_client.call(srv_msg);

//     if (!srv_msg.response.success)
//     {
//         ROS_ERROR_STREAM("Failed to submit "<< agv_name <<" to assembly station " << srv_msg.request.assembly_station_name);
//     }
//     else
//     {
//         ROS_INFO_STREAM(agv_id << " moving to: " << srv_msg.request.assembly_station_name);
//     }
// }