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

std::vector<Vector3d> setKneeOrientation(const std::string& kneeOrientation);

struct WalkingGait
{
    double step_length;      // 歩幅
    double step_height;      // 足上げ高さ
    double step_duration;    // 1歩の時間
    double duty_factor;      // デューティ比（接地時間/歩行周期）
    
    WalkingGait(double length = 0.05, double height = 0.02, double duration = 1.0, double duty = 0.6);
};

// 歩行用の関数宣言
std::vector<Eigen::Vector3d> generateForwardWalkingFootPositions(
    double time, 
    const WalkingGait& gait,
    const Eigen::Vector3d leg_offsets[]);

std::vector<std::tuple<double, double, double, bool>> setLegPositionsForWalking(
    double time,
    const Eigen::Vector3d& body_position,
    const Eigen::Quaterniond& body_orientation,
    const WalkingGait& gait,
    const Eigen::Vector3d leg_offsets[]);

std::vector<Eigen::Vector3d> generateAdaptiveWalkingFootPositions(
    double time,
    const Eigen::Vector3d& velocity,
    const WalkingGait& gait,
    const Eigen::Vector3d leg_offsets[]);


std::vector<std::tuple<double, double, double, bool>> setLegPositionsWithFixedFeet(
    const Vector3d& body_position, 
    const Quaterniond& body_orientation,
    const Vector3d fixed_foot_positions[],
    const Vector3d leg_offsets[]);

// ボディ移動クラスの宣言
class BodyMovement
{
public:
    Vector3d initial_body_position;
    Quaterniond initial_body_orientation;
    Vector3d fixed_foot_positions[4];  // arm_numの代わりに直接4を使用
    bool feet_are_fixed;
    
    BodyMovement();
    void fixCurrentFootPositions(const Vector3d& current_body_pos, 
                                const Quaterniond& current_body_orient,
                                const Vector3d leg_offsets[]);
    std::vector<std::tuple<double, double, double, bool>> moveBodyWithFixedFeet(
        const Vector3d& target_body_position,
        const Quaterniond& target_body_orientation,
        const Vector3d leg_offsets[]);
    void releaseFeet();
};