#include <ros/ros.h>
#include <arm_controller/arm.h>
#include <arm_controller/core.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64MultiArray.h>

void callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
void debugCallback(const geometry_msgs::PointStamped::ConstPtr& msg);

// const int arm_num = 4;
// Arm arms[arm_num] =
// {
//     Arm( 1, Vector3d(1.0, 0.0,  0.0), Vector3d( 0.0, 0.06,  0.0),
//          2, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0, 0.06,  0.0),
//          3, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0, 0.06,  0.0)),
    
//     Arm( 4, Vector3d(1.0, 0.0,  0.0), Vector3d( 0.0, -0.06,  0.0),
//          5, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0, -0.06,  0.0),
//          6, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0, -0.06,  0.0)),
    
//     Arm( 7, Vector3d(1.0, 0.0,  0.0), Vector3d( 0.0,  0.06,  0.0),
//          8, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0,  0.06,  0.0),
//          9, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0,  0.06,  0.0)),
    
//     Arm(10, Vector3d(1.0, 0.0,  0.0), Vector3d( 0.0, -0.06,  0.0),
//         11, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0, -0.06,  0.0),
//         12, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0, -0.06,  0.0))
// };

std::vector<geometry_msgs::Vector3> link0_pos(4);
void initLink0Positions()
{
    link0_pos[0].x = 0.06; link0_pos[0].y = 0.06; link0_pos[0].z = 0.0;
    link0_pos[1].x = 0.06; link0_pos[1].y = -0.06; link0_pos[1].z = 0.0;
    link0_pos[2].x = -0.06; link0_pos[2].y = 0.06; link0_pos[2].z = 0.0;
    link0_pos[3].x = -0.06; link0_pos[3].y = -0.06; link0_pos[3].z = 0.0;
}

tf2_ros::TransformBroadcaster* br;
tf2_ros::StaticTransformBroadcaster* static_br;

ros::Publisher pub_marker;
ros::Publisher pub_debug;
ros::Publisher pub_target_angle;

void publishTransforms(Arm& arm, double angle1, double angle2, double angle3, const std::string& frame_prefix)
{
    auto pos = arm.calculateEndEffectorPosition(angle1, angle2, angle3);

    tf2::Quaternion q_ini;
    q_ini.setRPY(0, 0, M_PI / 2);
    tf2::Quaternion q1, q2, q3;
    q1.setRotation(tf2::Vector3(arm.getAxis1()(0), arm.getAxis1()(1), arm.getAxis1()(2)), angle1 * M_PI / 180.0);
    q2.setRotation(tf2::Vector3(arm.getAxis2()(0), arm.getAxis2()(1), arm.getAxis2()(2)), angle2 * M_PI / 180.0);
    q3.setRotation(tf2::Vector3(arm.getAxis3()(0), arm.getAxis3()(1), arm.getAxis3()(2)), angle3 * M_PI / 180.0);

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    
    // Link 1
    transformStamped.header.frame_id = frame_prefix + "0";
    transformStamped.child_frame_id = frame_prefix + "1";
    transformStamped.transform.translation.x = std::get<0>(pos)(0);
    transformStamped.transform.translation.y = std::get<0>(pos)(1);
    transformStamped.transform.translation.z = std::get<0>(pos)(2);
    tf2::Quaternion q_link1 = q1 * q_ini;
    transformStamped.transform.rotation.x = q_link1.x();
    transformStamped.transform.rotation.y = q_link1.y();
    transformStamped.transform.rotation.z = q_link1.z();
    transformStamped.transform.rotation.w = q_link1.w();
    br->sendTransform(transformStamped);

    // Link 2
    transformStamped.header.frame_id = frame_prefix + "0";
    transformStamped.child_frame_id = frame_prefix + "2";
    transformStamped.transform.translation.x = std::get<1>(pos)(0);
    transformStamped.transform.translation.y = std::get<1>(pos)(1);
    transformStamped.transform.translation.z = std::get<1>(pos)(2);
    tf2::Quaternion q_link2 = q1 * q2 * q_ini;
    transformStamped.transform.rotation.x = q_link2.x();
    transformStamped.transform.rotation.y = q_link2.y();
    transformStamped.transform.rotation.z = q_link2.z();
    transformStamped.transform.rotation.w = q_link2.w();
    br->sendTransform(transformStamped);

    // Link 3
    transformStamped.header.frame_id = frame_prefix + "0";
    transformStamped.child_frame_id = frame_prefix + "3";
    transformStamped.transform.translation.x = std::get<2>(pos)(0);
    transformStamped.transform.translation.y = std::get<2>(pos)(1);
    transformStamped.transform.translation.z = std::get<2>(pos)(2);
    tf2::Quaternion q_link3 = q1 * q2 * q3 * q_ini;
    transformStamped.transform.rotation.x = q_link3.x();
    transformStamped.transform.rotation.y = q_link3.y();
    transformStamped.transform.rotation.z = q_link3.z();
    transformStamped.transform.rotation.w = q_link3.w();
    br->sendTransform(transformStamped);
}

void createMarkers(const std::tuple<Vector3d, Vector3d, Vector3d>& pos, const std::string& base_frame, const std::string& ns_prefix, int arm_id, visualization_msgs::MarkerArray& marker_array)
{
    // ARM色を設定
    std_msgs::ColorRGBA colors[4];
    colors[0].r = 1.0; colors[0].g = 0.0; colors[0].b = 0.0; colors[0].a = 1.0; // 赤
    colors[1].r = 0.0; colors[1].g = 1.0; colors[1].b = 0.0; colors[1].a = 1.0; // 緑
    colors[2].r = 0.0; colors[2].g = 0.0; colors[2].b = 1.0; colors[2].a = 1.0; // 青
    colors[3].r = 1.0; colors[3].g = 1.0; colors[3].b = 0.0; colors[3].a = 1.0; // 黄

    geometry_msgs::Vector3 scale;
    scale.x = 0.01;
    scale.y = 0.01;
    scale.z = 0.01;

    // Link 1 マーカー
    visualization_msgs::Marker link1_marker;
    link1_marker.header.frame_id = base_frame;
    link1_marker.header.stamp = ros::Time::now();
    link1_marker.ns = ns_prefix + "_links";
    link1_marker.id = arm_id * 10 + 1;
    link1_marker.type = visualization_msgs::Marker::CYLINDER;
    link1_marker.action = visualization_msgs::Marker::ADD;
    link1_marker.pose.position.x = std::get<0>(pos)(0) / 2.0;
    link1_marker.pose.position.y = std::get<0>(pos)(1) / 2.0;
    link1_marker.pose.position.z = std::get<0>(pos)(2) / 2.0;
    
    // Link 1の向きを計算
    Eigen::Vector3d link1_vec = std::get<0>(pos);
    Eigen::Vector3d z_axis(0, 0, 1);
    Eigen::Vector3d rotation_axis = z_axis.cross(link1_vec.normalized());
    double angle = acos(z_axis.dot(link1_vec.normalized()));
    if (rotation_axis.norm() > 1e-6) {
        tf2::Quaternion q_link1_orient;
        q_link1_orient.setRotation(tf2::Vector3(rotation_axis.x(), rotation_axis.y(), rotation_axis.z()), angle);
        link1_marker.pose.orientation.x = q_link1_orient.x();
        link1_marker.pose.orientation.y = q_link1_orient.y();
        link1_marker.pose.orientation.z = q_link1_orient.z();
        link1_marker.pose.orientation.w = q_link1_orient.w();
    } else {
        link1_marker.pose.orientation.w = 1.0;
    }
    
    link1_marker.scale.x = scale.x;
    link1_marker.scale.y = scale.y;
    link1_marker.scale.z = link1_vec.norm();
    link1_marker.color = colors[arm_id % 4];
    marker_array.markers.push_back(link1_marker);

    // Link 2 マーカー
    visualization_msgs::Marker link2_marker;
    link2_marker.header.frame_id = base_frame;
    link2_marker.header.stamp = ros::Time::now();
    link2_marker.ns = ns_prefix + "_links";
    link2_marker.id = arm_id * 10 + 2;
    link2_marker.type = visualization_msgs::Marker::CYLINDER;
    link2_marker.action = visualization_msgs::Marker::ADD;
    link2_marker.pose.position.x = (std::get<0>(pos)(0) + std::get<1>(pos)(0)) / 2.0;
    link2_marker.pose.position.y = (std::get<0>(pos)(1) + std::get<1>(pos)(1)) / 2.0;
    link2_marker.pose.position.z = (std::get<0>(pos)(2) + std::get<1>(pos)(2)) / 2.0;
    
    // Link 2の向きを計算
    Eigen::Vector3d link2_vec = std::get<1>(pos) - std::get<0>(pos);
    rotation_axis = z_axis.cross(link2_vec.normalized());
    angle = acos(z_axis.dot(link2_vec.normalized()));
    if (rotation_axis.norm() > 1e-6) {
        tf2::Quaternion q_link2_orient;
        q_link2_orient.setRotation(tf2::Vector3(rotation_axis.x(), rotation_axis.y(), rotation_axis.z()), angle);
        link2_marker.pose.orientation.x = q_link2_orient.x();
        link2_marker.pose.orientation.y = q_link2_orient.y();
        link2_marker.pose.orientation.z = q_link2_orient.z();
        link2_marker.pose.orientation.w = q_link2_orient.w();
    } else {
        link2_marker.pose.orientation.w = 1.0;
    }
    
    link2_marker.scale.x = scale.x;
    link2_marker.scale.y = scale.y;
    link2_marker.scale.z = link2_vec.norm();
    link2_marker.color = colors[arm_id % 4];
    marker_array.markers.push_back(link2_marker);

    // Link 3 マーカー
    visualization_msgs::Marker link3_marker;
    link3_marker.header.frame_id = base_frame;
    link3_marker.header.stamp = ros::Time::now();
    link3_marker.ns = ns_prefix + "_links";
    link3_marker.id = arm_id * 10 + 3;
    link3_marker.type = visualization_msgs::Marker::CYLINDER;
    link3_marker.action = visualization_msgs::Marker::ADD;
    link3_marker.pose.position.x = (std::get<1>(pos)(0) + std::get<2>(pos)(0)) / 2.0;
    link3_marker.pose.position.y = (std::get<1>(pos)(1) + std::get<2>(pos)(1)) / 2.0;
    link3_marker.pose.position.z = (std::get<1>(pos)(2) + std::get<2>(pos)(2)) / 2.0;
    
    // Link 3の向きを計算
    Eigen::Vector3d link3_vec = std::get<2>(pos) - std::get<1>(pos);
    rotation_axis = z_axis.cross(link3_vec.normalized());
    angle = acos(z_axis.dot(link3_vec.normalized()));
    if (rotation_axis.norm() > 1e-6) {
        tf2::Quaternion q_link3_orient;
        q_link3_orient.setRotation(tf2::Vector3(rotation_axis.x(), rotation_axis.y(), rotation_axis.z()), angle);
        link3_marker.pose.orientation.x = q_link3_orient.x();
        link3_marker.pose.orientation.y = q_link3_orient.y();
        link3_marker.pose.orientation.z = q_link3_orient.z();
        link3_marker.pose.orientation.w = q_link3_orient.w();
    } else {
        link3_marker.pose.orientation.w = 1.0;
    }
    
    link3_marker.scale.x = scale.x;
    link3_marker.scale.y = scale.y;
    link3_marker.scale.z = link3_vec.norm();
    link3_marker.color = colors[arm_id % 4];
    marker_array.markers.push_back(link3_marker);

    // ジョイント（球）マーカー
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
    joint_marker.scale.x = scale.x * 0.5;
    joint_marker.scale.y = scale.y * 0.5;
    joint_marker.scale.z = scale.z * 0.5;
    joint_marker.color.r = 0.5;
    joint_marker.color.g = 0.5;
    joint_marker.color.b = 0.5;
    joint_marker.color.a = 1.0;
    marker_array.markers.push_back(joint_marker);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle nh;

    initLink0Positions();
    
    br = new tf2_ros::TransformBroadcaster();
    static_br = new tf2_ros::StaticTransformBroadcaster();

    for (int i = 0; i < arm_num; ++i)
    {
        geometry_msgs::TransformStamped static_transformStamped;
        static_transformStamped.header.stamp = ros::Time::now();
        static_transformStamped.header.frame_id = "base_link";
        static_transformStamped.child_frame_id = "link" + std::to_string(i) + "_0";
        static_transformStamped.transform.translation = link0_pos[i];
        static_transformStamped.transform.rotation.x = 0.0;
        static_transformStamped.transform.rotation.y = 0.0;
        static_transformStamped.transform.rotation.z = 0.0;
        static_transformStamped.transform.rotation.w = 1.0;
        static_br->sendTransform(static_transformStamped);
    }

    std::string angle_topic_name;
    nh.getParam("angle_topic_name", angle_topic_name);
    ros::Subscriber sub = nh.subscribe(angle_topic_name, 1, callback);
    pub_target_angle = nh.advertise<geometry_msgs::Vector3>("target_angle", 1);
    pub_marker = nh.advertise<visualization_msgs::MarkerArray>("arm_markers", 1);
    pub_debug = nh.advertise<visualization_msgs::MarkerArray>("arm_markers_debug", 1);

    // ros::Subscriber sub_debug = nh.subscribe("clicked_point", 1, debugCallback);

    ros::spin();
    delete br;
    delete static_br;
    return 0;
}

void callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() != arm_num * 3)
    {
        ROS_ERROR("Not match array size: %zu", msg->data.size());
        return;
    }

    visualization_msgs::MarkerArray marker_array;

    // 4つのARMをfor文で処理
    for (int arm_id = 0; arm_id < arm_num; ++arm_id)
    {
        // 各ARMの角度を取得
        double angle1 = msg->data[arm_id * 3 + 0];
        double angle2 = msg->data[arm_id * 3 + 1];
        double angle3 = msg->data[arm_id * 3 + 2];

        // 各ARMの角度を設定
        arms[arm_id].setAngle1(angle1);
        arms[arm_id].setAngle2(angle2);
        arms[arm_id].setAngle3(angle3);

        // TF変換を発行
        std::string frame_prefix = "link" + std::to_string(arm_id) + "_";
        publishTransforms(arms[arm_id], angle1, angle2, angle3, frame_prefix);

        // マーカーを作成
        auto pos = arms[arm_id].calculateEndEffectorPosition(angle1, angle2, angle3);
        std::string base_frame = "link" + std::to_string(arm_id) + "_0";
        createMarkers(pos, base_frame, "arm", arm_id, marker_array);
    }

    pub_marker.publish(marker_array);
}

// void debugCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
// {
//     Vector3d clicked_point(msg->point.x, msg->point.y, msg->point.z);
//     auto angles = arm1.calculateAngle(clicked_point);

//     auto pos = arm1.calculateEndEffectorPosition(std::get<0>(angles), 
//                                                  std::get<1>(angles), 
//                                                  std::get<2>(angles));

//     if (std::get<3>(angles))
//     {
//         geometry_msgs::Vector3 target_goal;
//         target_goal.x = std::get<0>(angles);
//         target_goal.y = std::get<1>(angles);
//         target_goal.z = std::get<2>(angles);
//         pub_target_angle.publish(target_goal);
//     }

//     tf2::Quaternion q_ini;
//     q_ini.setRPY(0, 0, M_PI / 2);
//     tf2::Quaternion q1, q2, q3;
//     q1.setRotation(tf2::Vector3(arm1.getAxis1()(0), arm1.getAxis1()(1), arm1.getAxis1()(2)), std::get<0>(angles) * M_PI / 180.0);
//     q2.setRotation(tf2::Vector3(arm1.getAxis2()(0), arm1.getAxis2()(1), arm1.getAxis2()(2)), std::get<1>(angles) * M_PI / 180.0);
//     q3.setRotation(tf2::Vector3(arm1.getAxis3()(0), arm1.getAxis3()(1), arm1.getAxis3()(2)), std::get<2>(angles) * M_PI / 180.0);

//     geometry_msgs::TransformStamped transformStamped;
//     transformStamped.header.stamp = ros::Time::now();
//     transformStamped.header.frame_id = "link0";
//     transformStamped.child_frame_id = "link1_debug";
//     transformStamped.transform.translation.x = std::get<0>(pos)(0);
//     transformStamped.transform.translation.y = std::get<0>(pos)(1);
//     transformStamped.transform.translation.z = std::get<0>(pos)(2);
//     tf2::Quaternion q_link1 = q1 * q_ini;
//     transformStamped.transform.rotation.x = q_link1.x();
//     transformStamped.transform.rotation.y = q_link1.y();
//     transformStamped.transform.rotation.z = q_link1.z();
//     transformStamped.transform.rotation.w = q_link1.w();
//     br->sendTransform(transformStamped);

//     transformStamped.header.frame_id = "link0";
//     transformStamped.child_frame_id = "link2_debug";
//     transformStamped.transform.translation.x = std::get<1>(pos)(0);
//     transformStamped.transform.translation.y = std::get<1>(pos)(1);
//     transformStamped.transform.translation.z = std::get<1>(pos)(2);
//     tf2::Quaternion q_link2 = q1 * q2 * q_ini;
//     transformStamped.transform.rotation.x = q_link2.x();
//     transformStamped.transform.rotation.y = q_link2.y();
//     transformStamped.transform.rotation.z = q_link2.z();
//     transformStamped.transform.rotation.w = q_link2.w();
//     br->sendTransform(transformStamped);

//     transformStamped.header.frame_id = "link0";
//     transformStamped.child_frame_id = "link3_debug";
//     transformStamped.transform.translation.x = std::get<2>(pos)(0);
//     transformStamped.transform.translation.y = std::get<2>(pos)(1);
//     transformStamped.transform.translation.z = std::get<2>(pos)(2);
//     tf2::Quaternion q_link3 = q1 * q2 * q3 * q_ini;
//     transformStamped.transform.rotation.x = q_link3.x();
//     transformStamped.transform.rotation.y = q_link3.y();
//     transformStamped.transform.rotation.z = q_link3.z();
//     transformStamped.transform.rotation.w = q_link3.w();
//     br->sendTransform(transformStamped);


//     visualization_msgs::MarkerArray marker_array;
//     // Link 1 マーカー
//     visualization_msgs::Marker link1_marker;
//     link1_marker.header.frame_id = "link0";
//     link1_marker.header.stamp = ros::Time::now();
//     link1_marker.ns = "arm_links";
//     link1_marker.id = 1;
//     link1_marker.type = visualization_msgs::Marker::CYLINDER;
//     link1_marker.action = visualization_msgs::Marker::ADD;
//     link1_marker.pose.position.x = std::get<0>(pos)(0) / 2.0;
//     link1_marker.pose.position.y = std::get<0>(pos)(1) / 2.0;
//     link1_marker.pose.position.z = std::get<0>(pos)(2) / 2.0;
    
//     // Link 1の向きを計算
//     Eigen::Vector3d link1_vec = std::get<0>(pos);
//     Eigen::Vector3d z_axis(0, 0, 1);
//     Eigen::Vector3d rotation_axis = z_axis.cross(link1_vec.normalized());
//     double angle = acos(z_axis.dot(link1_vec.normalized()));
//     if (rotation_axis.norm() > 1e-6) {
//         tf2::Quaternion q_link1_orient;
//         q_link1_orient.setRotation(tf2::Vector3(rotation_axis.x(), rotation_axis.y(), rotation_axis.z()), angle);
//         link1_marker.pose.orientation.x = q_link1_orient.x();
//         link1_marker.pose.orientation.y = q_link1_orient.y();
//         link1_marker.pose.orientation.z = q_link1_orient.z();
//         link1_marker.pose.orientation.w = q_link1_orient.w();
//     } else {
//         link1_marker.pose.orientation.w = 1.0;
//     }
    
//     link1_marker.scale.x = 0.1;
//     link1_marker.scale.y = 0.1;
//     link1_marker.scale.z = link1_vec.norm();
//     link1_marker.color.r = 1.0;
//     link1_marker.color.g = 0.0;
//     link1_marker.color.b = 0.0;
//     link1_marker.color.a = 1.0;
//     marker_array.markers.push_back(link1_marker);

//     // Link 2 マーカー
//     visualization_msgs::Marker link2_marker;
//     link2_marker.header.frame_id = "link0";
//     link2_marker.header.stamp = ros::Time::now();
//     link2_marker.ns = "arm_links";
//     link2_marker.id = 2;
//     link2_marker.type = visualization_msgs::Marker::CYLINDER;
//     link2_marker.action = visualization_msgs::Marker::ADD;
//     link2_marker.pose.position.x = (std::get<0>(pos)(0) + std::get<1>(pos)(0)) / 2.0;
//     link2_marker.pose.position.y = (std::get<0>(pos)(1) + std::get<1>(pos)(1)) / 2.0;
//     link2_marker.pose.position.z = (std::get<0>(pos)(2) + std::get<1>(pos)(2)) / 2.0;
    
//     // Link 2の向きを計算
//     Eigen::Vector3d link2_vec = std::get<1>(pos) - std::get<0>(pos);
//     rotation_axis = z_axis.cross(link2_vec.normalized());
//     angle = acos(z_axis.dot(link2_vec.normalized()));
//     if (rotation_axis.norm() > 1e-6) {
//         tf2::Quaternion q_link2_orient;
//         q_link2_orient.setRotation(tf2::Vector3(rotation_axis.x(), rotation_axis.y(), rotation_axis.z()), angle);
//         link2_marker.pose.orientation.x = q_link2_orient.x();
//         link2_marker.pose.orientation.y = q_link2_orient.y();
//         link2_marker.pose.orientation.z = q_link2_orient.z();
//         link2_marker.pose.orientation.w = q_link2_orient.w();
//     } else {
//         link2_marker.pose.orientation.w = 1.0;
//     }
    
//     link2_marker.scale.x = 0.1;
//     link2_marker.scale.y = 0.1;
//     link2_marker.scale.z = link2_vec.norm();
//     link2_marker.color.r = 0.0;
//     link2_marker.color.g = 1.0;
//     link2_marker.color.b = 0.0;
//     link2_marker.color.a = 1.0;
//     marker_array.markers.push_back(link2_marker);

//     // Link 3 マーカー
//     visualization_msgs::Marker link3_marker;
//     link3_marker.header.frame_id = "link0";
//     link3_marker.header.stamp = ros::Time::now();
//     link3_marker.ns = "arm_links";
//     link3_marker.id = 3;
//     link3_marker.type = visualization_msgs::Marker::CYLINDER;
//     link3_marker.action = visualization_msgs::Marker::ADD;
//     link3_marker.pose.position.x = (std::get<1>(pos)(0) + std::get<2>(pos)(0)) / 2.0;
//     link3_marker.pose.position.y = (std::get<1>(pos)(1) + std::get<2>(pos)(1)) / 2.0;
//     link3_marker.pose.position.z = (std::get<1>(pos)(2) + std::get<2>(pos)(2)) / 2.0;
    
//     // Link 3の向きを計算
//     Eigen::Vector3d link3_vec = std::get<2>(pos) - std::get<1>(pos);
//     rotation_axis = z_axis.cross(link3_vec.normalized());
//     angle = acos(z_axis.dot(link3_vec.normalized()));
//     if (rotation_axis.norm() > 1e-6) {
//         tf2::Quaternion q_link3_orient;
//         q_link3_orient.setRotation(tf2::Vector3(rotation_axis.x(), rotation_axis.y(), rotation_axis.z()), angle);
//         link3_marker.pose.orientation.x = q_link3_orient.x();
//         link3_marker.pose.orientation.y = q_link3_orient.y();
//         link3_marker.pose.orientation.z = q_link3_orient.z();
//         link3_marker.pose.orientation.w = q_link3_orient.w();
//     } else {
//         link3_marker.pose.orientation.w = 1.0;
//     }
    
//     link3_marker.scale.x = 0.1;
//     link3_marker.scale.y = 0.1;
//     link3_marker.scale.z = link3_vec.norm();
//     link3_marker.color.r = 0.0;
//     link3_marker.color.g = 0.0;
//     link3_marker.color.b = 1.0;
//     link3_marker.color.a = 1.0;
//     marker_array.markers.push_back(link3_marker);

//     // ジョイント（球）マーカー
//     visualization_msgs::Marker joint_marker;
//     joint_marker.header.frame_id = "link0";
//     joint_marker.header.stamp = ros::Time::now();
//     joint_marker.ns = "arm_joints";
//     joint_marker.id = 0;
//     joint_marker.type = visualization_msgs::Marker::SPHERE;
//     joint_marker.action = visualization_msgs::Marker::ADD;
//     joint_marker.pose.position.x = 0.0;
//     joint_marker.pose.position.y = 0.0;
//     joint_marker.pose.position.z = 0.0;
//     joint_marker.pose.orientation.w = 1.0;
//     joint_marker.scale.x = 0.2;
//     joint_marker.scale.y = 0.2;
//     joint_marker.scale.z = 0.2;
//     joint_marker.color.r = 0.5;
//     joint_marker.color.g = 0.5;
//     joint_marker.color.b = 0.5;
//     joint_marker.color.a = 1.0;
//     marker_array.markers.push_back(joint_marker);

//     pub_debug.publish(marker_array);
// }