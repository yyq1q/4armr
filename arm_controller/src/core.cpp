#include <arm_controller/arm.h>
#include <arm_controller/core.h>
#include <iostream>

const int arm_num = 4;
Arm arms[arm_num] =
{
    Arm( 1, Vector3d(1.0, 0.0,  0.0), Vector3d( 0.0,  0.05 ,  0.0),
         2, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0,  0.105,  0.0),
         3, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0,  0.105,  0.0)),
    
    Arm( 4, Vector3d(1.0, 0.0,  0.0), Vector3d( 0.0, -0.05 ,  0.0),
         5, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0, -0.105,  0.0),
         6, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0, -0.105,  0.0)),
    
    Arm( 7, Vector3d(1.0, 0.0,  0.0), Vector3d( 0.0,  0.05 ,  0.0),
         8, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0,  0.105,  0.0),
         9, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0,  0.105,  0.0)),
    
    Arm(10, Vector3d(1.0, 0.0,  0.0), Vector3d( 0.0, -0.05 ,  0.0),
        11, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0, -0.105,  0.0),
        12, Vector3d(0.0, 0.0, -1.0), Vector3d( 0.0, -0.105,  0.0))
};


std::vector<std::tuple<double, double, double, bool>> setLegPositionsFromBody(
    const Vector3d& body_position, 
    const Quaterniond& body_orientation,
    const Vector3d leg_offsets[arm_num])
{
    std::vector<std::tuple<double, double, double, bool>> target_positions;
    // 各足先の目標位置を計算
    for(int i = 0; i < arm_num; i++)
    {
        // bodyからの相対位置をクォタニオンで回転
        Vector3d rotated_offset = body_orientation * leg_offsets[i];
        
        // bodyの位置からのオフセットを加算して絶対位置を計算
        Vector3d target_position = -body_position;
        
        target_positions.push_back(arms[i].calculateAngle(target_position));
        std::cout << body_position << std::endl;
        std::cout << target_position.transpose() << std::endl;
    }
    return target_positions;
}

// const Vector3d default_leg_offsets[arm_num] =
// {
//     Vector3d( 0.0675,  0.093, 0.0),  // arm0: 左前足
//     Vector3d( 0.0675, -0.093, 0.0),  // arm1: 右前足
//     Vector3d(-0.0675,  0.093, 0.0),  // arm2: 左後足
//     Vector3d(-0.0675, -0.093, 0.0)   // arm3: 右前足
// };

//debug用
const Vector3d default_leg_offsets[arm_num] =
{
    Vector3d( 0.1,  0.1, 0.0),  // arm0: 左前足
    Vector3d( 0.1, -0.1, 0.0),  // arm1: 右前足
    Vector3d(-0.1,  0.1, 0.0),  // arm2: 左後足
    Vector3d(-0.1, -0.1, 0.0)   // arm3: 右前足
};

const Vector3d ini_leg_angle[arm_num] =
{
    Vector3d( 90.0,  0.0, 0.0),  // arm0: 左前足
    Vector3d( 90.0,  0.0, 0.0),  // arm1: 右前足
    Vector3d( 90.0,  0.0, 0.0),  // arm2: 左後足
    Vector3d( 90.0,  0.0, 0.0)   // arm3: 右前足
};