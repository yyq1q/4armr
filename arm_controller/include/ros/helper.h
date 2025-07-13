#pragma once

#include <ros/ros.h>
#include <arm_controller/arm.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>

class ArmVisualizer {
public:
    ArmVisualizer(ros::NodeHandle& nh);
    ~ArmVisualizer();

    void publishTransforms(Arm& arm, double angle1, double angle2, double angle3, const std::string& frame_prefix);
    void createMarkers(const std::tuple<Vector3d, Vector3d, Vector3d>& pos, const std::string& base_frame, 
                      const std::string& ns_prefix, int arm_id, visualization_msgs::MarkerArray& marker_array);
    void publishBaseLinkTransform(const Eigen::Vector3d& body_pos, const Eigen::Quaterniond& body_q);
    void publishStaticTransforms();
    void publishMarkers(const visualization_msgs::MarkerArray& marker_array);
    void publishSplineWaypoints(const visualization_msgs::Marker& spline_marker);
    void publishJointStates();

    sensor_msgs::JointState goal_joint_state;

private:
    tf2_ros::TransformBroadcaster* br_;
    tf2_ros::StaticTransformBroadcaster* static_br_;
    ros::Publisher pub_marker_;
    ros::Publisher pub_target_angle_;
    ros::Publisher pub_spline_waypoints_;
    ros::Publisher pub_joint_states_;
    
    std::string frame_prefix_;
    std::string frame_prefix_goal_;
};