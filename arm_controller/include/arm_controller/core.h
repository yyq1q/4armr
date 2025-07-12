#pragma once
#include "arm.h"

extern const int arm_num;
extern Arm arms[];
extern const Vector3d default_leg_offsets[];
extern const Vector3d ini_leg_angle[];

// 関数の宣言
std::vector<std::tuple<double, double, double, bool>> setLegPositionsFromBody(
    const Vector3d& body_position, 
    const Quaterniond& body_orientation,
    const Vector3d leg_offsets[]);