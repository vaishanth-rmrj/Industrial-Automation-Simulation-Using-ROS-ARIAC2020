#ifndef __UTILS_H__
#define __UTILS_H__

// ###################################################################
// ################### Importing Necessary Libraries #################
// ###################################################################

// ros
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
// standard library
#include <string>
#include <cstdint>
#include <tf2/LinearMath/Quaternion.h>
#include <array>

// ###################################################################



namespace motioncontrol {
    geometry_msgs::Pose transformToWorldFrame(
        const geometry_msgs::Pose& target,
        std::string agv,
        std::string robot);

    std::array<double, 3> eulerFromQuaternion(const geometry_msgs::Pose& pose);
    std::array<double, 3> eulerFromQuaternion(double x, double y, double z, double w);
    std::array<double, 3> eulerFromQuaternion(const tf2::Quaternion& quat);
    tf2::Quaternion quaternionFromEuler(double r, double p, double y);

    geometry_msgs::Pose get_part_target_pose(const geometry_msgs::Pose& target_pose, std::string target_frame, std::string child_frame_id); 

    /**
     * @brief Print the components of a quaternion
     * 
     * @param quat tf2::Quaternion to print
     */
    void print(const tf2::Quaternion& quat);

    /**
     * @brief Print the components of a Pose
     * 
     * @param pose 
     */
    void print(const geometry_msgs::Pose& pose);
}  // namespace motioncontrol

#endif