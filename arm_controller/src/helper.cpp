#include <ros/helper.h>
#include <arm_controller/core.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/ColorRGBA.h>

ArmVisualizer::ArmVisualizer(ros::NodeHandle& nh) 
    : frame_prefix_("current_link"), frame_prefix_goal_("goal_link")
{
    
    br_ = new tf2_ros::TransformBroadcaster();
    static_br_ = new tf2_ros::StaticTransformBroadcaster();
    
    pub_target_angle_ = nh.advertise<geometry_msgs::Vector3>("target_angle", 1);
    pub_marker_ = nh.advertise<visualization_msgs::MarkerArray>("arm_markers", 1);
    pub_spline_waypoints_ = nh.advertise<visualization_msgs::Marker>("spline_waypoints", 1);
    pub_joint_states_ = nh.advertise<sensor_msgs::JointState>("joint_states2", 1);

    goal_joint_state.name.resize(12);
    goal_joint_state.name[ 0] = "lf_hip_joint";
    goal_joint_state.name[ 1] = "lf_upper_leg_joint";
    goal_joint_state.name[ 2] = "lf_lower_leg_joint";
    goal_joint_state.name[ 3] = "rf_hip_joint";
    goal_joint_state.name[ 4] = "rf_upper_leg_joint";
    goal_joint_state.name[ 5] = "rf_lower_leg_joint";
    goal_joint_state.name[ 6] = "lh_hip_joint";
    goal_joint_state.name[ 7] = "lh_upper_leg_joint";
    goal_joint_state.name[ 8] = "lh_lower_leg_joint";
    goal_joint_state.name[ 9] = "rh_hip_joint";
    goal_joint_state.name[10] = "rh_upper_leg_joint";
    goal_joint_state.name[11] = "rh_lower_leg_joint";
    goal_joint_state.position.resize(12);
}

ArmVisualizer::~ArmVisualizer() {
    delete br_;
    delete static_br_;
}

void ArmVisualizer::publishStaticTransforms() {
    for (int i = 0; i < arm_num; ++i) {
        geometry_msgs::TransformStamped static_transformStamped;
        static_transformStamped.header.stamp = ros::Time::now();
        static_transformStamped.header.frame_id = "base_link";
        static_transformStamped.child_frame_id = frame_prefix_ + std::to_string(i) + "_0";
        static_transformStamped.transform.translation.x = default_leg_offsets[i](0);
        static_transformStamped.transform.translation.y = default_leg_offsets[i](1);
        static_transformStamped.transform.translation.z = default_leg_offsets[i](2);
        static_transformStamped.transform.rotation.x = 0.0;
        static_transformStamped.transform.rotation.y = 0.0;
        static_transformStamped.transform.rotation.z = 0.0;
        static_transformStamped.transform.rotation.w = 1.0;
        static_br_->sendTransform(static_transformStamped);

        static_transformStamped.child_frame_id = frame_prefix_goal_ + std::to_string(i) + "_0";
        static_br_->sendTransform(static_transformStamped);
    }
}

void ArmVisualizer::publishTransforms(Arm& arm, double angle1, double angle2, double angle3, const std::string& frame_prefix) {
    auto pos = arm.calculateEndEffectorPosition(angle1, angle2, angle3);

    tf2::Quaternion q_ini;
    q_ini.setRPY(0, 0, M_PI / 2);
    tf2::Quaternion q1, q2, q3;
    q1.setRotation(tf2::Vector3(arm.getAxis1()(0), arm.getAxis1()(1), arm.getAxis1()(2)), angle1 * M_PI / 180.0);
    q2.setRotation(tf2::Vector3(arm.getAxis2()(0), arm.getAxis2()(1), arm.getAxis2()(2)), angle2 * M_PI / 180.0);
    q3.setRotation(tf2::Vector3(arm.getAxis3()(0), arm.getAxis3()(1), arm.getAxis3()(2)), angle3 * M_PI / 180.0);

    std::vector<Eigen::Vector3d> positions = {
        std::get<0>(pos),
        std::get<1>(pos),
        std::get<2>(pos)
    };
    
    std::vector<tf2::Quaternion> rotations = {
        q1 * q_ini,
        q1 * q2 * q_ini,
        q1 * q2 * q3 * q_ini
    };

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    
    for (int i = 0; i < 3; ++i) {
        transformStamped.header.frame_id = frame_prefix + "0";
        transformStamped.child_frame_id = frame_prefix + std::to_string(i + 1);
        transformStamped.transform.translation.x = positions[i](0);
        transformStamped.transform.translation.y = positions[i](1);
        transformStamped.transform.translation.z = positions[i](2);
        transformStamped.transform.rotation.x = rotations[i].x();
        transformStamped.transform.rotation.y = rotations[i].y();
        transformStamped.transform.rotation.z = rotations[i].z();
        transformStamped.transform.rotation.w = rotations[i].w();
        br_->sendTransform(transformStamped);
    }
}

void ArmVisualizer::createMarkers(const std::tuple<Vector3d, Vector3d, Vector3d>& pos, const std::string& base_frame, 
                                 const std::string& ns_prefix, int arm_id, visualization_msgs::MarkerArray& marker_array) {
    std_msgs::ColorRGBA colors[4];
    colors[0].r = 1.0; colors[0].g = 0.0; colors[0].b = 0.0; colors[0].a = 1.0;
    colors[1].r = 0.0; colors[1].g = 1.0; colors[1].b = 0.0; colors[1].a = 1.0;
    colors[2].r = 0.0; colors[2].g = 0.0; colors[2].b = 1.0; colors[2].a = 1.0;
    colors[3].r = 1.0; colors[3].g = 1.0; colors[3].b = 0.0; colors[3].a = 1.0;

    geometry_msgs::Vector3 scale;
    scale.x = 0.01;
    scale.y = 0.01;
    scale.z = 0.01;

    std::vector<Eigen::Vector3d> positions = {
        Eigen::Vector3d(0, 0, 0),
        std::get<0>(pos),
        std::get<1>(pos),
        std::get<2>(pos)
    };

    Eigen::Vector3d z_axis(0, 0, 1);

    for (int i = 1; i < 4; ++i) {
        visualization_msgs::Marker link_marker;
        link_marker.header.frame_id = base_frame;
        link_marker.header.stamp = ros::Time::now();
        link_marker.ns = ns_prefix + "_" + std::to_string(i) + "_links";
        link_marker.id = arm_id * 10 + i;
        link_marker.type = visualization_msgs::Marker::CYLINDER;
        link_marker.action = visualization_msgs::Marker::ADD;
        
        Eigen::Vector3d link_vec = positions[i] - positions[i-1];
        Eigen::Vector3d mid_point = (positions[i-1] + positions[i]) / 2.0;
        
        link_marker.pose.position.x = mid_point(0);
        link_marker.pose.position.y = mid_point(1);
        link_marker.pose.position.z = mid_point(2);
        
        Eigen::Vector3d rotation_axis = z_axis.cross(link_vec.normalized());
        double angle = acos(z_axis.dot(link_vec.normalized()));
        
        if (rotation_axis.norm() > 1e-6) {
            tf2::Quaternion q_orient;
            q_orient.setRotation(tf2::Vector3(rotation_axis.x(), rotation_axis.y(), rotation_axis.z()), angle);
            link_marker.pose.orientation.x = q_orient.x();
            link_marker.pose.orientation.y = q_orient.y();
            link_marker.pose.orientation.z = q_orient.z();
            link_marker.pose.orientation.w = q_orient.w();
        } else {
            link_marker.pose.orientation.w = 1.0;
        }
        
        link_marker.scale.x = scale.x;
        link_marker.scale.y = scale.y;
        link_marker.scale.z = link_vec.norm();
        link_marker.color = colors[arm_id % 4];
        marker_array.markers.push_back(link_marker);
    }

    visualization_msgs::Marker joint_marker;
    joint_marker.header.frame_id = base_frame;
    joint_marker.header.stamp = ros::Time::now();
    joint_marker.ns = ns_prefix + "_joints";
    joint_marker.id = arm_id * 10;
    joint_marker.type = visualization_msgs::Marker::SPHERE;
    joint_marker.action = visualization_msgs::Marker::ADD;
    joint_marker.pose.position.x = 0.0;
    joint_marker.pose.position.y = 0.0;
    joint_marker.pose.position.z = 0.0;
    joint_marker.pose.orientation.w = 1.0;
    joint_marker.scale.x = scale.x * 2.0;
    joint_marker.scale.y = scale.y * 2.0;
    joint_marker.scale.z = scale.z * 2.0;
    joint_marker.color.r = 0.5;
    joint_marker.color.g = 0.5;
    joint_marker.color.b = 0.5;
    joint_marker.color.a = 1.0;
    marker_array.markers.push_back(joint_marker);
}

void ArmVisualizer::publishBaseLinkTransform(const Eigen::Vector3d& body_pos, const Eigen::Quaterniond& body_q) {
    geometry_msgs::TransformStamped odom_to_base_transform;
    odom_to_base_transform.header.stamp = ros::Time::now();
    odom_to_base_transform.header.frame_id = "odom";
    odom_to_base_transform.child_frame_id = "base_link";
    odom_to_base_transform.transform.translation.x = body_pos(0);
    odom_to_base_transform.transform.translation.y = body_pos(1);
    odom_to_base_transform.transform.translation.z = body_pos(2);
    odom_to_base_transform.transform.rotation.x = body_q.x();
    odom_to_base_transform.transform.rotation.y = body_q.y();
    odom_to_base_transform.transform.rotation.z = body_q.z();
    odom_to_base_transform.transform.rotation.w = body_q.w();
    br_->sendTransform(odom_to_base_transform);
}

void ArmVisualizer::publishMarkers(const visualization_msgs::MarkerArray& marker_array) {
    pub_marker_.publish(marker_array);
}

void ArmVisualizer::publishSplineWaypoints(const visualization_msgs::Marker& spline_marker) {
    pub_spline_waypoints_.publish(spline_marker);
}

void ArmVisualizer::publishJointStates() {
    pub_joint_states_.publish(goal_joint_state);
}