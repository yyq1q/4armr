#include <ros/ros.h>
#include <arm_controller/arm.h>
#include <arm_controller/core.h>
#include <arm_controller/spline.h>
#include <ros/helper.h>
#include <sensor_msgs/Joy.h>

#define DEBUG

Eigen::Vector3d body_pos(0.0, 0.0, 0.2);
Eigen::Quaterniond body_q;
double roll = 0.0, pitch = 0.0, yaw = 0.0;

void callbackJoy(const sensor_msgs::Joy::ConstPtr& joy_msg);
sensor_msgs::Joy current_joy_msg;

//手を降るモーション
spline3d::Spline3D spline(
    {
        Eigen::Vector3d(0.1,  0.1, -0.1),
        Eigen::Vector3d(0.1, -0.1, -0.1),
        Eigen::Vector3d(0.1,  0.1, -0.1)
    }
);
auto way_points = spline.sampleByDistance(0.01, 0.0001);
int current_waypoint_index = 0;

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle nh;

    ros::Subscriber joy_sub = nh.subscribe("joy", 1, callbackJoy);
    current_joy_msg.buttons.resize(32, 0);
    current_joy_msg.axes.resize(8, 0.0);
    
    ArmVisualizer visualizer(nh);
    visualizer.publishStaticTransforms();

    auto ini_leg_angle = setKneeOrientation("<<");

    for (int i = 0; i < arm_num; ++i)
    {
        arms[i].setGoalAngle(ini_leg_angle[i](0),
                             ini_leg_angle[i](1),
                             ini_leg_angle[i](2));

        #ifdef DEBUG
        arms[i].setCurrentAngle(ini_leg_angle[i](0),
                                ini_leg_angle[i](1),
                                ini_leg_angle[i](2));
        #endif
    }

    static int cnt = 0;
    static bool increasing = true;

    ros::Timer timer = nh.createTimer(ros::Duration(0.01), [&visualizer](const ros::TimerEvent&)
    {
        body_q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                 Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        Eigen::Vector3d body_pos_offset = Eigen::Vector3d(0.0, 0.0, 0.0);
        Eigen::Vector3d body_pos_target = body_pos;
        if (current_joy_msg.buttons[0] == 1)
        {
            body_pos_offset = Eigen::Vector3d(0.0, 0.0, 0.0005);
            body_pos_target = body_pos + body_pos_offset * cnt;
            auto result = setLegPositionsFromBody(body_pos_target, body_q, default_leg_offsets);
        }
        else if (current_joy_msg.buttons[1] == 1)
        {
            body_pos_offset = Eigen::Vector3d(-0.0005, 0.0, 0.0);
            body_pos_target = body_pos + body_pos_offset * cnt;
            auto result = setLegPositionsFromBody(body_pos_target, body_q, default_leg_offsets);
        }
        else if (current_joy_msg.buttons[3] == 1)
        {
            body_pos_offset = Eigen::Vector3d(0.0, 0.0005, 0.0);
            body_pos_target = body_pos + body_pos_offset * cnt;
            auto result = setLegPositionsFromBody(body_pos_target, body_q, default_leg_offsets);
        }
        else if (current_joy_msg.buttons[2] == 1)
        {
            auto [goal_angle1, goal_angle2, goal_angle3, success] = arms[0].calculateAngle(way_points[current_waypoint_index % way_points.size()]);
            if (success) arms[0].setGoalAngle(goal_angle1, goal_angle2, goal_angle3);
            current_waypoint_index++;
        }
        else
        {
            cnt = 0;
            increasing = true;
        }
        visualizer.publishBaseLinkTransform(body_pos_target, body_q);

        visualization_msgs::MarkerArray marker_array;

        for (int arm_id = 0; arm_id < arm_num; ++arm_id)
        {   
            auto [angle1, angle2, angle3] = arms[arm_id].getCurrentAngle();
            visualizer.publishTransforms(arms[arm_id], angle1, angle2, angle3,
                                       "current_link" + std::to_string(arm_id) + "_");

            auto[goal_angle1, goal_angle2, goal_angle3] = arms[arm_id].getGoalAngle();
            visualizer.goal_joint_state.position[arm_id * 3 + 0] = goal_angle1;
            visualizer.goal_joint_state.position[arm_id * 3 + 1] = goal_angle2;
            visualizer.goal_joint_state.position[arm_id * 3 + 2] = goal_angle3;
            visualizer.publishTransforms(arms[arm_id], goal_angle1, goal_angle2, goal_angle3,
                                       "goal_link" + std::to_string(arm_id) + "_");

            auto pos = arms[arm_id].calculateEndEffectorPosition(goal_angle1, goal_angle2, goal_angle3);
            visualizer.createMarkers(pos, "goal_link" + std::to_string(arm_id) + "_0", 
                                   "arm", arm_id, marker_array);
        }
        visualizer.publishMarkers(marker_array);

        if (increasing) {
            cnt++;
            if (cnt >= 100) increasing = false;
        } else {
            cnt--;
            if (cnt <= -100) increasing = true;
        }
        visualizer.publishJointStates();
    });

    ros::spin();
    return 0;
}

void callbackJoy(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
    current_joy_msg = *joy_msg;
}