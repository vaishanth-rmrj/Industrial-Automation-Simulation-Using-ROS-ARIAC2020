// ###################################################################
// ################### Importing Necessary Libraries #################
// ###################################################################

#include <vector>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <nist_gear/Order.h>
#include <nist_gear/AGVToAssemblyStation.h>
#include <nist_gear/LogicalCameraImage.h>
#include <group2_rwa4/agv_location.h>
#include <group2_rwa4/check_agv_faulty_parts.h>
#include <group2_rwa4/FaultyPartPose.h>
#include <group2_rwa4/order_kitting_shipment_details.h>
#include <group2_rwa4/list_order_details.h>

// ####################################################################

/**
 * @brief The AGV class comtains the details of part and respective assembly station coordinates
 * 
 */

class AGV{

    private:
    ros::NodeHandle nh;
    std::string agv_id;
    std::string agv_name;
    std::string agv_station;
    std::string agv_state;
    std::string agv_destination;
    std::string agv_shipment_type;
    bool shipment_delivered;
    int faulty_part_count;
    std::vector<geometry_msgs::Pose> faulty_part_poses;
    nist_gear::Order order;

    // Subscribers 
    ros::Subscriber quality_sensor_sub;
    ros::Subscriber agv_logicam_sub;

    // Services
    ros::ServiceServer check_agv_faulty_part_service;

    // Service messages
    geometry_msgs::Pose faulty_part_pose;

    /**
     * @brief Defining the required functions for the AGV
     * 
     */

    public:

    /**
     * @brief Construct a new AGV object
     * 
     * @param node_handler 
     * @param id 
     */
    AGV(ros::NodeHandle &node_handler, int id);

    /**
     * @brief Initializing 
     * 
     */

    void init();

    /**
     * @brief Set the agv state 
     * 
     * @param state 
     */
    
    void set_agv_state(const std_msgs::String::ConstPtr &state);

    /**
     * @brief Set the agv station
     * 
     * @param station 
     */

    void set_agv_station(const std_msgs::String::ConstPtr &station);

    /**
     * @brief Check if part is faulty
     * 
     * @param quality_sensor_msg 
     */

    void check_faulty_part(const nist_gear::LogicalCameraImage::ConstPtr &quality_sensor_msg);

    /**
     * @brief Send AGV to assembly station
     * 
     */

    // void submit_agv(std::string assembly_station, std::string shipment_type);

    /**
     * @brief Service to get agv location
     * 
     * @param req 
     * @param res 
     * @return true 
     * @return false 
     */

    bool get_agv_location_service_callback(group2_rwa4::agv_location::Request &req, group2_rwa4::agv_location::Response &res);

    /**
     * @brief Service to get faulty parts count on agv
     * 
     * @param req 
     * @param res 
     * @return true 
     * @return false 
     */

    bool check_agv_faulty_part_service_callback(group2_rwa4::check_agv_faulty_parts::Request &req, group2_rwa4::check_agv_faulty_parts::Response &res);


    void agv_logicam_callback(const nist_gear::LogicalCameraImage::ConstPtr &logicam_msg);

    
};