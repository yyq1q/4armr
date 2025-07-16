#include "arm.h"
#include "helper.h"

Arm::Arm(uint8_t id1, Vector3d axis1, Vector3d link1,
         uint8_t id2, Vector3d axis2, Vector3d link2,
         uint8_t id3, Vector3d axis3, Vector3d link3)
    : id1(id1), axis1(axis1.normalized()),
      id2(id2), axis2(axis2.normalized()),
      id3(id3), axis3(axis3.normalized()),
      rot1(Matrix3d::Identity()),
      rot2(Matrix3d::Identity()),
      rot3(Matrix3d::Identity()),
      link1(link1),
      link2(link2),
      link3(link3)
{
    endEffector = Vector3d::Zero();
    angle1 = 0.0;
    angle2 = 0.0;
    angle3 = 0.0;

    nTn_1 = axis1 * axis1.transpose();
    nTn_2 = axis2 * axis2.transpose();
    nTn_3 = axis3 * axis3.transpose();

    skew1 = createSkewSymmetric(axis1);
    skew2 = createSkewSymmetric(axis2);
    skew3 = createSkewSymmetric(axis3);
}

bool Arm::init()
{
    return DynamixelCtrl::setOperationMode(id1, OP_POSITION) &&
           DynamixelCtrl::setOperationMode(id2, OP_POSITION) &&
           DynamixelCtrl::setOperationMode(id3, OP_POSITION);
}

void Arm::setAngle(double angle1, double angle2, double angle3)
{
    auto normalizeAngle = [](double angle) -> double
    {
        angle = fmod(angle, 360.0);
        if (angle > 180.0)
        {
            angle -= 360.0;
        }
        else if (angle < -180.0)
        {
            angle += 360.0;
        }
        return angle;
    };

    dxl.setGoalPosition(id1, dmap(normalizeAngle(angle1), -180, 180, 0, 4095));
    dxl.setGoalPosition(id2, dmap(normalizeAngle(angle2), -180, 180, 0, 4095));
    dxl.setGoalPosition(id3, dmap(normalizeAngle(angle3), -180, 180, 0, 4095));
}

bool Arm::setPos(double pos1, double pos2, double pos3)
{
    return dxl.setGoalPosition(id1, pos1) &&
           dxl.setGoalPosition(id2, pos2) &&
           dxl.setGoalPosition(id3, pos3);
}

std::tuple<double, double, double> Arm::getAngle()
{
    return std::make_tuple(dmap(dxl.getPresentPosition(id1), 0, 4095, -180.0, 180.0),
                           dmap(dxl.getPresentPosition(id2), 0, 4095, -180.0, 180.0),
                           dmap(dxl.getPresentPosition(id3), 0, 4095, -180.0, 180.0));
}

Matrix3d Arm::getRotationMatrix(Vector3d axis, double angle)
{
    // 軸ベクトルを正規化
    Vector3d normalizedAxis = axis.normalized();
    
    // AngleAxisを使用して回転行列を生成
    return AngleAxisd(angle * M_PI / 180.0, normalizedAxis).toRotationMatrix();
}

Vector3d Arm::calculateEndEffectorPosition()
{
    getAngle();
    rot1 = getRotationMatrix(axis1, angle1);
    rot2 = getRotationMatrix(axis2, angle2);
    rot3 = getRotationMatrix(axis3, angle3);

    endEffector = rot1 * link1 + rot1 * rot2 * link2 + rot1 * rot2 * rot3 * link3;
    return endEffector;
}

Vector3d Arm::getEndEffectorPosition()
{
    return endEffector;
}

std::tuple<double, double, double> Arm::calculateAngle(Vector3d targetPosition)
{
    // 収束条件
    const double tolerance = 1e-6;
    const int maxIterations = 100;
    
    // 現在の角度を初期値として使用
    getAngle();
    double theta1 = this->angle1;
    double theta2 = this->angle2;
    double theta3 = this->angle3;
    
    for (int i = 0; i < maxIterations; i++) {
        // 現在の角度で回転行列を更新
        rot1 = getRotationMatrix(axis1, theta1);
        rot2 = getRotationMatrix(axis2, theta2);
        rot3 = getRotationMatrix(axis3, theta3);
        
        // 現在のエンドエフェクタ位置を計算
        Vector3d currentPosition = rot1 * link1 + rot1 * rot2 * link2 + rot1 * rot2 * rot3 * link3;
        
        // 目標位置との誤差を計算
        Vector3d error = targetPosition - currentPosition;
        
        // 収束判定
        if (error.norm() < tolerance) {
            break;
        }
        
        // ヤコビアンの計算
        auto dR_dtheta1 = calculateRotDerivative(theta1, this->nTn_1, this->skew1);
        auto dR_dtheta2 = calculateRotDerivative(theta2, this->nTn_2, this->skew2);
        auto dR_dtheta3 = calculateRotDerivative(theta3, this->nTn_3, this->skew3);

        auto df_dtheta1 = dR_dtheta1 * link1 + dR_dtheta1 * rot2 * link2 + dR_dtheta1 * rot2 * rot3 * link3;
        auto df_dtheta2 = rot1 * dR_dtheta2 * link2 + rot1 * dR_dtheta2 * rot3 * link3;
        auto df_dtheta3 = rot1 * rot2 * dR_dtheta3 * link3;

        Matrix3d J;
        J.col(0) = df_dtheta1;
        J.col(1) = df_dtheta2;
        J.col(2) = df_dtheta3;
        
        // ヤコビアンの逆行列を計算（特異点のチェックも含む）
        if (J.determinant() < 1e-10) {
            // 特異点の場合は疑似逆行列を使用
            auto J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
            Vector3d deltaTheta = J_pinv * error;
            
            theta1 += deltaTheta(0) * 180.0 / M_PI;
            theta2 += deltaTheta(1) * 180.0 / M_PI;
            theta3 += deltaTheta(2) * 180.0 / M_PI;
        } else {
            auto J_inv = J.inverse();
            Vector3d deltaTheta = J_inv * error;
            
            theta1 += deltaTheta(0) * 180.0 / M_PI;
            theta2 += deltaTheta(1) * 180.0 / M_PI;
            theta3 += deltaTheta(2) * 180.0 / M_PI;
        }
    }
    
    // 計算された角度を更新
    this->angle1 = theta1;
    this->angle2 = theta2;
    this->angle3 = theta3;
    
    return std::make_tuple(theta1, theta2, theta3);
}

Matrix3d Arm::createSkewSymmetric(const Vector3d& v)
{
    Matrix3d skew;
    skew <<      0, -v.z(),  v.y(),
             v.z(),      0, -v.x(),
            -v.y(),  v.x(),      0;
    return skew;
}

Matrix3d Arm::calculateRotDerivative(double angle, Matrix3d nTn, Matrix3d skew)
{   
    double rad = angle * M_PI / 180.0;
    Matrix3d rotDerivative = sin(rad) * (nTn - Matrix3d::Identity())
                           + cos(rad) * skew;
    return rotDerivative;
}

bool Arm::torqueOff()
{
    return dxl.torqueOff(id1) &&
           dxl.torqueOff(id2) &&
           dxl.torqueOff(id3);
}

bool Arm::torqueOn()
{
    return dxl.torqueOn(id1) &&
           dxl.torqueOn(id2) &&
           dxl.torqueOn(id3);
}