#pragma once

#include "dynamixel_ctrl.h"
#include <ArduinoEigen.h>
#include <ArduinoEigenDense.h>
#include <ArduinoEigenSparse.h>

using namespace Eigen;

class Arm
{
public:
    Arm(uint8_t id1, Vector3d axis1, Vector3d link1,
        uint8_t id2, Vector3d axis2, Vector3d link2,
        uint8_t id3, Vector3d axis3, Vector3d link3);
    bool init();
    void setAngle(double angle1, double angle2, double angle3);
    bool setPos(double pos1, double pos2, double pos3);
    std::tuple<double, double, double> getAngle();
    Matrix3d getRotationMatrix(Vector3d axis, double angle);
    Vector3d calculateEndEffectorPosition();
    Vector3d getEndEffectorPosition();
    std::tuple<double, double, double> calculateAngle(Vector3d position);
    Matrix3d createSkewSymmetric(const Vector3d& v);
    Matrix3d calculateRotDerivative(double angle1, Matrix3d nTn, Matrix3d skew);
    bool torqueOff();
    bool torqueOn();

private:
    uint8_t id1, id2, id3;
    Vector3d axis1, axis2, axis3;
    Matrix3d rot1, rot2, rot3;
    Vector3d link1, link2, link3;
    Vector3d endEffector;
    double angle1, angle2, angle3;
    Matrix3d nTn_1, nTn_2, nTn_3; // 回転軸の転値とそのままをかけたもの
    Matrix3d skew1, skew2, skew3; // 回転軸のスキュー行列
};