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

tf2_ros::TransformBroadcaster* br;
tf2_ros::StaticTransformBroadcaster* static_br;

ros::Publisher pub_marker;
ros::Publisher pub_target_angle;

void publishTransforms(Arm& arm, double angle1, double angle2, double angle3, const std::string& frame_prefix);
void createMarkers(const std::tuple<Vector3d, Vector3d, Vector3d>& pos, const std::string& base_frame, const std::string& ns_prefix, int arm_id, visualization_msgs::MarkerArray& marker_array);
void publishBaseLinkTransform(const Eigen::Vector3d& body_pos, const Eigen::Quaterniond& body_q);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle nh;
    
    br = new tf2_ros::TransformBroadcaster();
    static_br = new tf2_ros::StaticTransformBroadcaster();

    for (int i = 0; i < arm_num; ++i)
    {
        geometry_msgs::TransformStamped static_transformStamped;
        static_transformStamped.header.stamp = ros::Time::now();
        static_transformStamped.header.frame_id = "base_link";
        static_transformStamped.child_frame_id = "link" + std::to_string(i) + "_0";
        static_transformStamped.transform.translation.x = default_leg_offsets[i](0);
        static_transformStamped.transform.translation.y = default_leg_offsets[i](1);
        static_transformStamped.transform.translation.z = default_leg_offsets[i](2);
        static_transformStamped.transform.rotation.x = 0.0;
        static_transformStamped.transform.rotation.y = 0.0;
        static_transformStamped.transform.rotation.z = 0.0;
        static_transformStamped.transform.rotation.w = 1.0;
        static_br->sendTransform(static_transformStamped);
    }

    std::string angle_topic_name;
    nh.getParam("angle_topic_name", angle_topic_name);
    //ros::Subscriber sub = nh.subscribe(angle_topic_name, 1, callback);
    pub_target_angle = nh.advertise<geometry_msgs::Vector3>("target_angle", 1);
    pub_marker = nh.advertise<visualization_msgs::MarkerArray>("arm_markers", 1);

    for (int i = 0; i < arm_num; ++i)
    {
        arms[i].setAngle1(ini_leg_angle[i](0));
        arms[i].setAngle2(ini_leg_angle[i](1));
        arms[i].setAngle3(ini_leg_angle[i](2));
    }

    static int cnt = 0;
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), [](const ros::TimerEvent&)
    {
        Eigen::Vector3d body_pos;
        body_pos << 0.1,
                    0.0,
                    0.2;
        
        body_pos(2) += (cnt * 0.005);
        
        Eigen::Quaterniond body_q;
        double roll = 0.0;
        double pitch = 0.0; 
        double yaw = M_PI / 4.0;
        body_q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                 Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        auto result = setLegPositionsFromBody(body_pos, body_q, default_leg_offsets);

        visualization_msgs::MarkerArray marker_array;
        for (int arm_id = 0; arm_id < arm_num; ++arm_id)
        {   
            double angle1 = std::get<0>(result[arm_id]);
            double angle2 = std::get<1>(result[arm_id]);
            double angle3 = std::get<2>(result[arm_id]);
            arms[arm_id].setAngle(angle1, angle2, angle3);
            std::string frame_prefix = "link" + std::to_string(arm_id) + "_";
            publishTransforms(arms[arm_id],
                              angle1,
                              angle2,
                              angle3,
                              frame_prefix);

            publishBaseLinkTransform(body_pos, body_q);

            auto pos = arms[arm_id].calculateEndEffectorPosition(angle1, angle2, angle3);
            std::string base_frame = "link" + std::to_string(arm_id) + "_0";
            createMarkers(pos, base_frame, "arm", arm_id, marker_array);
        }
        pub_marker.publish(marker_array);

        // cnt++;
        if (cnt >10) cnt = 0;
    });

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
        createMarkers(pos, base_frame, "arm" + std::to_string(arm_id), arm_id, marker_array);
    }

    pub_marker.publish(marker_array);
}

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

    // 位置データを配列に格納
    std::vector<Eigen::Vector3d> positions = {
        Eigen::Vector3d(0, 0, 0),  // base position
        std::get<0>(pos),          // link1 end
        std::get<1>(pos),          // link2 end
        std::get<2>(pos)           // link3 end
    };

    Eigen::Vector3d z_axis(0, 0, 1);

    // Linkマーカーをfor文で作成
    for (int i = 1; i < 4; ++i) {
        visualization_msgs::Marker link_marker;
        link_marker.header.frame_id = base_frame;
        link_marker.header.stamp = ros::Time::now();
        link_marker.ns = ns_prefix + "_" + std::to_string(i) + "_links";
        link_marker.id = arm_id * 10 + i;
        link_marker.type = visualization_msgs::Marker::CYLINDER;
        link_marker.action = visualization_msgs::Marker::ADD;
        
        // リンクの中点を計算
        Eigen::Vector3d link_vec = positions[i] - positions[i-1];
        Eigen::Vector3d mid_point = (positions[i-1] + positions[i]) / 2.0;
        
        link_marker.pose.position.x = mid_point(0);
        link_marker.pose.position.y = mid_point(1);
        link_marker.pose.position.z = mid_point(2);
        
        // リンクの向きを計算
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
    joint_marker.scale.x = scale.x * 1.0;
    joint_marker.scale.y = scale.y * 1.0;
    joint_marker.scale.z = scale.z * 1.0;
    joint_marker.color.r = 0.5;
    joint_marker.color.g = 0.5;
    joint_marker.color.b = 0.5;
    joint_marker.color.a = 1.0;
    marker_array.markers.push_back(joint_marker);
}

void publishBaseLinkTransform(const Eigen::Vector3d& body_pos, const Eigen::Quaterniond& body_q)
{
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
    br->sendTransform(odom_to_base_transform);
}