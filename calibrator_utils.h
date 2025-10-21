#pragma once

#include <Eigen/Dense>
#include <vector>
#include <torch/torch.h>

Eigen::MatrixXf batch_matvec_mul(const std::vector<Eigen::Matrix3f>& rot, const Eigen::MatrixXf& acc);
Eigen::MatrixXf apply_matrix_to_batch(const Eigen::Matrix3f& M, const Eigen::MatrixXf& V);
std::vector<Eigen::Matrix3f> left_multiply_batch(const Eigen::Matrix3f& M, const std::vector<Eigen::Matrix3f>& batch);
Eigen::Matrix3f quat_to_rotmat_XYZ(const Eigen::Vector4f& q_raw);
Eigen::Matrix3f quat_to_rotmat(const Eigen::Vector4f& q_raw);
std::vector<Eigen::Matrix3f> quat_batch_to_rotmat(const Eigen::MatrixXf& quats);
void round_matrix(Eigen::Matrix3f& mat, int precision = 4);
