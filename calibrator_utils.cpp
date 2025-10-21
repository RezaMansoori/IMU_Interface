#include "calibrator_utils.h"
#include <iostream>
#include <cmath>  // برای std::pow و std::round

Eigen::MatrixXf batch_matvec_mul(const std::vector<Eigen::Matrix3f>& rot, const Eigen::MatrixXf& acc) {
    const int N = acc.rows();
    Eigen::MatrixXf result(N, 3);

    for (int i = 0; i < N; ++i) {
        result.row(i) = (rot[i] * acc.row(i).transpose()).transpose();
    }

    return result;
}

Eigen::MatrixXf apply_matrix_to_batch(const Eigen::Matrix3f& M, const Eigen::MatrixXf& V) {
    const int N = V.rows();
    Eigen::MatrixXf result(N, 3);

    for (int i = 0; i < N; ++i) {
        Eigen::Vector3f vec = V.row(i).transpose(); // 1x3 -> 3x1
        Eigen::Vector3f prod = M * vec; // 3x3 * 3x1 -> 3x1
        result.row(i) = prod.transpose(); // 3x1 -> 1x3
    }
    return result;
}

std::vector<Eigen::Matrix3f> left_multiply_batch(const Eigen::Matrix3f& M, const std::vector<Eigen::Matrix3f>& batch) {
    std::vector<Eigen::Matrix3f> result(batch.size());
    for (size_t i = 0; i < batch.size(); ++i)
        result[i] = M * batch[i];
    return result;
}

Eigen::Matrix3f quat_to_rotmat(const Eigen::Vector4f& q_raw) {
    Eigen::Vector4f q = q_raw.normalized(); // Normalize quaternion
    float w = q[0], x = q[1], y = q[2], z = q[3];

    Eigen::Matrix3f R;
    R << 1 - 2 * (y * y + z * z),     2 * (x * y - z * w),     2 * (x * z + y * w),
        2 * (x * y + z * w),     1 - 2 * (x * x + z * z),     2 * (y * z - x * w),
        2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y);
    return R;
}

Eigen::Matrix3f quat_to_rotmat_XYZ(const Eigen::Vector4f& q_raw) {
    Eigen::Vector4f q = q_raw.normalized(); // Normalize quaternion
    float w = q[0], x = q[1], y = q[2], z = q[3];

    Eigen::Matrix3f R;
    R << 1 - 2 * (y * y + z * z),     2 * (x * y - z * w),     2 * (x * z + y * w),
        2 * (x * y + z * w),     1 - 2 * (x * x + z * z),     2 * (y * z - x * w),
        2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y);
    return R;
}

std::vector<Eigen::Matrix3f> quat_batch_to_rotmat(const Eigen::MatrixXf& quats) {
    std::vector<Eigen::Matrix3f> result;
    for (int i = 0; i < quats.rows(); ++i) {
        result.push_back(quat_to_rotmat(quats.row(i)));
    }
    return result;
}

void round_matrix(Eigen::Matrix3f& mat, int precision) {
    float factor = std::pow(10, precision);
    mat = (mat * factor).array().round() / factor;
}
