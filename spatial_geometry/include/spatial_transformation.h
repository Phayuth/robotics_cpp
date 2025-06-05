#ifndef SPATIAL_TRANSFORMATION_H
#define SPATIAL_TRANSFORMATION_H

#include <Eigen/Dense>

void hrx(Eigen::Matrix4f &h, float theta);
void hry(Eigen::Matrix4f &h, float theta);
void hrz(Eigen::Matrix4f &h, float theta);
void ht(Eigen::Matrix4f &h, float x, float y, float z);

void rx(Eigen::Matrix3f &r, float theta);
void ry(Eigen::Matrix3f &r, float theta);
void rz(Eigen::Matrix3f &r, float theta);

void conv_rotmat_to_quat(Eigen::Vector4f q, Eigen::Matrix3f r);
void conv_quat_to_rotmat(Eigen::Matrix3f r, Eigen::Vector4f q);

#endif