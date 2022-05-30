#include <ros/ros.h>
// #include <array>

#include "kitting_handler.h"
#include "kitting_robot.h"


int main(int argc, char ** argv)
{   
    ros::init(argc, argv, "kitting_robot_handler_node");

    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // wait till order handler has started
    bool is_order_initialized = false;
    nh.getParam("/is_order_initialized", is_order_initialized);
    while (!is_order_initialized)
    {
        nh.getParam("/is_order_initialized", is_order_initialized);
    }  

    ros::Duration(5).sleep();
    ROS_INFO("Starting kitting handler");


    KittingRobot kitting_robot(nh);
    KittingHandler kitting_handler(nh);

    kitting_handler.perform_order_kitting();
    
    ros::waitForShutdown();
}




