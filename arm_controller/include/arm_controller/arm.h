#pragma once

#include <Eigen/Dense>

using namespace Eigen;

class Arm
{
public:
    Arm(uint8_t id1, Vector3d axis1, Vector3d link1,
        uint8_t id2, Vector3d axis2, Vector3d link2,
        uint8_t id3, Vector3d axis3, Vector3d link3);
    bool init();
    void setGoalAngle(double angle1, double angle2, double angle3);
    void setCurrentAngle(double angle1, double angle2, double angle3);
    std::tuple<double, double, double> getGoalAngle();
    std::tuple<double, double, double> getCurrentAngle();
    Matrix3d getRotationMatrix(Vector3d axis, double angle);
    std::tuple<Vector3d, Vector3d, Vector3d> calculateEndEffectorPosition(double angle1, double angle2, double angle3);
    Vector3d getEndEffectorPosition();
    std::tuple<double, double, double, bool> calculateAngle(Vector3d position);
    Matrix3d createSkewSymmetric(const Vector3d& v);
    Matrix3d calculateRotDerivative(double angle1, Matrix3d nTn, Matrix3d skew);
    Vector3d getAxis1() const { return axis1; }
    Vector3d getAxis2() const { return axis2; }
    Vector3d getAxis3() const { return axis3; }
    Vector3d getLink1() const { return link1; }
    Vector3d getLink2() const { return link2; }
    Vector3d getLink3() const { return link3; }
    double setAngle1(double agl1){ angle1 = agl1; return angle1; }
    double setAngle2(double agl2){ angle2 = agl2; return angle2; }
    double setAngle3(double agl3){ angle3 = agl3; return angle3; }
    double getGoalAngle(uint8_t index){ return goalAngle[index%3]; }

private:
    uint8_t id1, id2, id3;
    Vector3d axis1, axis2, axis3;
    Matrix3d rot1, rot2, rot3;
    Vector3d link1, link2, link3;
    Vector3d endEffector;
    double angle1, angle2, angle3;
    double goalAngle[3];
    double currentAngle[3];
    Matrix3d nTn_1, nTn_2, nTn_3; // 回転軸の転値とそのままをかけたもの
    Matrix3d skew1, skew2, skew3; // 回転軸のスキュー行列
};