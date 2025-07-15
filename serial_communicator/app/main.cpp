#include <serial_communicator/serial.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <boost/bind.hpp>
#include <sensor_msgs/Joy.h>

union AngleData
{
    float angles[12];  // 4つのARM × 3つの角度
    uint8_t bytes[48]; // 12 * sizeof(float)
};

union PosData
{
    uint16_t positions[12];
    uint8_t bytes[24];
};

AngleData angle_data;
AngleData angle_data2;
AngleData received_angle_data;

template <typename T, typename U>
U mapValue(T x, T in_min, T in_max, U out_min, U out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double normalizeAngle(double angle)
{
    while (angle > 180.0)
        angle -= 360.0;
    while (angle < -180.0)
        angle += 360.0;
    return angle;
}

PosData send_pos_data;
PosData send_pos_data2;

uint8_t header[2] = {0xFF, 0xFF};
uint8_t footer[2] = {0xFE, 0xFD};

const int arm_num = 4;

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg, SerialCommunication* serial)
{
    if (msg->name.size() != arm_num * 3)
    {
        ROS_ERROR("Received joint state does not contain enough data for all arms.");
        return;
    }

    for (int i = 0; i < arm_num; i++)
    {
        double x_offset = 0.0;
        double isInverse = 1.0;
        if (i % 2 == 0)
        {
            x_offset = -90.0;
            isInverse = -1.0;
        }
        else
        {
            x_offset = 90.0;
            isInverse = 1.0;
        }

        angle_data.angles[i * 3 + 0] = static_cast<float>(msg->position[i * 3 + 0] * 180 / M_PI + x_offset);
        angle_data.angles[i * 3 + 1] = static_cast<float>(msg->position[i * 3 + 1] * 180 / M_PI * isInverse);
        angle_data.angles[i * 3 + 2] = static_cast<float>(msg->position[i * 3 + 2] * 180 / M_PI * isInverse);

        send_pos_data.positions[i * 3 + 0] = static_cast<uint16_t>(mapValue(normalizeAngle(-msg->position[i * 3 + 0] * 180 / M_PI + x_offset),  -180.0, 180.0, 0, 4095));
        send_pos_data.positions[i * 3 + 1] = static_cast<uint16_t>(mapValue(normalizeAngle(msg->position[i * 3 + 1] * 180 / M_PI * (-isInverse)), -180.0, 180.0, 0, 4095));
        send_pos_data.positions[i * 3 + 2] = static_cast<uint16_t>(mapValue(normalizeAngle(msg->position[i * 3 + 2] * 180 / M_PI * isInverse), -180.0, 180.0, 0, 4095));

        // std::cout << "ARM" << i << ": "
        //           << "Angle1: " << angle_data.angles[i * 3 + 0] << ", "
        //           << "Angle2: " << angle_data.angles[i * 3 + 1] << ", "
        //           << "Angle3: " << angle_data.angles[i * 3 + 2] << ", " << std::endl;
                //   << "Position1: " << send_pos_data.positions[i * 3 + 0] << ", "
                //   << "Position2: " << send_pos_data.positions[i * 3 + 1] << ", "
                //   << "Position3: " << send_pos_data.positions[i * 3 + 2] << std::endl;
    }
}

void jointState2Callback(const sensor_msgs::JointState::ConstPtr& msg, SerialCommunication* serial)
{
    if (msg->name.size() != arm_num * 3)
    {
        ROS_ERROR("Received joint state does not contain enough data for all arms.");
        return;
    }

    for (int i = 0; i < arm_num; i++)
    {
        double x_offset = 0.0;
        double isInverse = 1.0;
        if (i % 2 == 0)
        {
            x_offset = -90.0;
            isInverse = -1.0;
        }
        else
        {
            x_offset = 90.0;
            isInverse = 1.0;
        }

        angle_data2.angles[i * 3 + 0] = static_cast<float>(msg->position[i * 3 + 0]);
        angle_data2.angles[i * 3 + 1] = static_cast<float>(msg->position[i * 3 + 1]);
        angle_data2.angles[i * 3 + 2] = static_cast<float>(msg->position[i * 3 + 2]);

        send_pos_data2.positions[i * 3 + 0] = static_cast<uint16_t>(mapValue(normalizeAngle(msg->position[i * 3 + 0]), -180.0, 180.0, 0, 4095));
        send_pos_data2.positions[i * 3 + 1] = static_cast<uint16_t>(mapValue(normalizeAngle(-msg->position[i * 3 + 1]), -180.0, 180.0, 0, 4095));
        send_pos_data2.positions[i * 3 + 2] = static_cast<uint16_t>(mapValue(normalizeAngle(msg->position[i * 3 + 2]), -180.0, 180.0, 0, 4095));
    }
}

sensor_msgs::Joy current_joy_msg;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_communicator");
    ros::NodeHandle nh;

    std::string serial_port;
    nh.getParam("port", serial_port);
    int serial_baudrate;
    nh.param<int>("baudrate", serial_baudrate, 115200);
    SerialCommunication serial(serial_port);

    std::string angle_topic_name;
    nh.getParam("angle_topic_name", angle_topic_name);
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>(angle_topic_name, 10);

    ros::Subscriber joint_state_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, 
        boost::bind(jointStateCallback, _1, &serial));
    ros::Subscriber joint_state_sub2 = nh.subscribe<sensor_msgs::JointState>("joint_states2", 1, 
        boost::bind(jointState2Callback, _1, &serial));

    current_joy_msg.buttons.resize(32, 0);
    current_joy_msg.axes.resize(32, 0.0);
    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, [&](const sensor_msgs::Joy::ConstPtr& msg)
    {
        current_joy_msg = *msg;
    });

    if (!serial.open_port(serial_baudrate))
    {
        ROS_ERROR("Failed to open serial port");
        return 1;
    }

    ros::Duration(1.0).sleep();

    serial.receive_bytes();

    // for (int i = 0; i < arm_num; ++i)
    // {
    //     std::string data_to_send = "ARM" + std::to_string(i) + ":" + "TorqueOn\n";
    //     ROS_INFO("Sending data for ARM%d: %s", i, data_to_send.c_str());
    //     if (!serial.send_data(data_to_send))
    //     {
    //         ROS_ERROR("Failed to send data for ARM%d", i);
    //     }
    // }

    // 送信データの初期化
    for (int i = 0; i < arm_num * 3; ++i)
    {
        send_pos_data.positions[i] = 2048;
        send_pos_data2.positions[i] = 2048;
    }
    send_pos_data.positions[0] = 1024;
    send_pos_data2.positions[0] = 1024;
    send_pos_data.positions[3] = 3072;
    send_pos_data2.positions[3] = 3072;
    send_pos_data.positions[6] = 1024;
    send_pos_data2.positions[6] = 1024;
    send_pos_data.positions[9] = 3072;
    send_pos_data2.positions[9] = 3072;

    ros::Time start_time = ros::Time::now();
    ros::Timer timer = nh.createTimer(ros::Duration(0.01), [&](const ros::TimerEvent&)
    {
    });

    ros::Timer timer2 = nh.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent&)
    {
        // auto received_data = serial.receive_bytes();
        // if (!received_data.empty())
        // {
        //     ROS_INFO("Received data size: %zu", received_data.size());
        //     if (received_data.size() >= 4 && 
        //         received_data[0] == header[0] &&
        //         received_data[1] == header[1] &&
        //         received_data[2] == 24 &&
        //         received_data[received_data.size() - 2] == footer[0] &&
        //         received_data[received_data.size() - 1] == footer[1])
        //     {
        //         memcpy(received_angle_data.bytes, received_data.data() + 3, sizeof(received_angle_data.bytes));
        //         std_msgs::Float64MultiArray angles_array_msg;
        //         angles_array_msg.layout.dim.resize(2);
        //         angles_array_msg.layout.dim[0].label = "arm";
        //         angles_array_msg.layout.dim[0].size = arm_num;
        //         angles_array_msg.layout.dim[0].stride = 3 * arm_num;
        //         angles_array_msg.layout.dim[1].label = "angle";
        //         angles_array_msg.layout.dim[1].size = 3;
        //         angles_array_msg.layout.dim[1].stride = 1;
        //         angles_array_msg.layout.data_offset = 0;
        //         angles_array_msg.data.resize(arm_num * 3); // 4 ARMs × 3 angles
        //         for (int i = 0; i < arm_num; ++i)
        //         {
        //             angles_array_msg.data[i * 3 + 0] = received_angle_data.angles[i * 3 + 0];
        //             angles_array_msg.data[i * 3 + 1] = received_angle_data.angles[i * 3 + 1];
        //             angles_array_msg.data[i * 3 + 2] = received_angle_data.angles[i * 3 + 2];
        //         }
        //         pub.publish(angles_array_msg);
        //     }
        // }

        // auto received_data = serial.receive_data();
        // if (!received_data.empty())
        // {
        //     std::cout << "Received data size: " << received_data.size() << std::endl;
        //     std::cout << "Received data: " << received_data << std::endl;
        // }

        std::vector<uint8_t> data_to_send;
        data_to_send.clear();
    
        data_to_send.push_back(header[0]);
        data_to_send.push_back(header[1]);
        data_to_send.push_back(24); // データ長を指定
        
        data_to_send.insert(data_to_send.end(), angle_data.bytes, angle_data.bytes + sizeof(angle_data.bytes));
        // if (current_joy_msg.buttons[0] == 1 ||
        //     current_joy_msg.buttons[1] == 1 ||
        //     current_joy_msg.buttons[2] == 1 ||
        //     current_joy_msg.buttons[3] == 1)
        // {
        //     data_to_send.insert(data_to_send.end(), send_pos_data2.bytes, send_pos_data2.bytes + sizeof(send_pos_data2.bytes));
        // }
        // else
        // {
        //     data_to_send.insert(data_to_send.end(), send_pos_data.bytes, send_pos_data.bytes + sizeof(send_pos_data.bytes));
        // }
        
        data_to_send.push_back(footer[0]);
        data_to_send.push_back(footer[1]);

        // for (int i = 0; i < data_to_send.size(); ++i)
        // {
        //     std::cout << std::hex << static_cast<int>(data_to_send[i]) << " ";
        // }
        // std::cout << std::dec << std::endl;

        if (serial.send_binary_data(data_to_send))
        {
            for (int i = 0; i < data_to_send.size(); ++i)
            {
                std::cout << std::hex << static_cast<int>(data_to_send[i]) << " ";
            }
            std::cout << std::dec << std::endl;
        }
        else
        {
            ROS_ERROR("Failed to send data to serial port.");
        }
    });

    ros::spin();
    std::string torque_off = "TorqueOff\n";
    if (serial.send_data(torque_off))
    {
    }
    serial.close_port();
    return 0;
}