#include "spatial_transformation.h"

void hrx(Eigen::Matrix4f &h, float theta) {
    h << 1.0, 0.0, 0.0, 0.0, 0.0, cosf(theta), -sinf(theta), 0.0, 0.0, sinf(theta),
        cosf(theta), 0.0, 0.0, 0.0, 0.0, 1.0;
}

void hry(Eigen::Matrix4f &h, float theta) {
    h << cosf(theta), 0.0, sinf(theta), 0.0, 0.0, 1.0, 0.0, 0.0, -sinf(theta), 0.0,
        cosf(theta), 0.0, 0.0, 0.0, 0.0, 1.0;
}

void hrz(Eigen::Matrix4f &h, float theta) {
    h << cosf(theta), -sinf(theta), 0.0, 0.0, sinf(theta), cosf(theta), 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
}

void ht(Eigen::Matrix4f &h, float x, float y, float z) {
    h << 1.0, 0.0, 0.0, x, 0.0, 1.0, 0.0, y, 0.0, 0.0, 1.0, z, 0.0, 0.0, 0.0, 1.0;
}

void rx(Eigen::Matrix3f &r, float theta) {
    r << 1.0, 0.0, 0.0, 0.0, cosf(theta), -sinf(theta), 0.0, sinf(theta),
        cosf(theta);
}

void ry(Eigen::Matrix3f &r, float theta) {
    r << cosf(theta), 0.0, sinf(theta), 0.0, 1.0, 0.0, -sinf(theta), 0.0,
        cosf(theta);
}

void rz(Eigen::Matrix3f &r, float theta) {
    r << cosf(theta), -sinf(theta), 0.0, sinf(theta), cosf(theta), 0.0, 0.0, 0.0,
        1.0;
}

void conv_rotmat_to_quat(Eigen::Vector4f q, Eigen::Matrix3f r) {
    float q0 = sqrtf(1 + r(0) + r(4) + r(8)) / 2;
    float q1 = (r(7) - r(5)) / (4 * q0);
    float q2 = (r(2) - r(6)) / (4 * q0);
    float q3 = (r(3) - r(1)) / (4 * q0);
    q << q0, q1, q2, q3;
}

void conv_quat_to_rotmat(Eigen::Matrix3f r, Eigen::Vector4f q) {
    r << 1 - 2 * pow(q(2), 2) - 2 * pow(q(3), 2),
        2 * q(1) * q(2) - 2 * q(0) * q(3), 2 * q(1) * q(3) + 2 * q(0) * q(2),
        2 * q(1) * q(2) + 2 * q(0) * q(3), 1 - 2 * pow(q(1), 2) - 2 * pow(q(3), 2),
        2 * q(2) * q(3) - 2 * q(0) * q(1), 2 * q(1) * q(3) - 2 * q(0) * q(2),
        2 * q(2) * q(3) + 2 * q(0) * q(1), 1 - 2 * pow(q(1), 2) - 2 * pow(q(2), 2);
}