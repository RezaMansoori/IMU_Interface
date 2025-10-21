#pragma once

#undef slots
#undef signals
#undef emit
#include <torch/torch.h>
#define slots Q_SLOTS
#define signals Q_SIGNALS
#define emit Q_EMIT


#include <Eigen/Dense>
#include <vector>
#include <map>
#include <string>
#include "calibratorBase.h"

Eigen::Matrix3f quat_to_rotmat(const Eigen::Vector4f& q);
Eigen::Matrix3f quat_to_rotmat_XYZ(const Eigen::Vector4f& q);
std::vector<Eigen::Matrix3f> quat_batch_to_rotmat(const Eigen::MatrixXf& quats);
#ifndef MATMUL_EIGEN_BATCH_H
#define MATMUL_EIGEN_BATCH_H

namespace matops {

inline std::vector<Eigen::Matrix3f> matmul_batch(
    const std::vector<Eigen::Matrix3f>& A,
    const std::vector<Eigen::Matrix3f>& B)
{
    const int N = A.size();
    std::vector<Eigen::Matrix3f> C(N);

    for (int i = 0; i < N; ++i) {
        C[i].setZero();
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                for (int k = 0; k < 3; ++k) {
                    C[i](r, c) += A[i](r, k) * B[i](k, c);
                }
            }
        }
    }

    return C;
}

}

#endif // MATMUL_EIGEN_BATCH_H

class Calibrator : public CalibratorBase {
public:

    Calibrator(const std::map<std::string, int>& selected_imus,
               const Eigen::MatrixXf& coord_data,
               const Eigen::MatrixXf& tpose_data);

    void calibrate(const Eigen::MatrixXf& acc,
                   const std::vector<Eigen::Matrix3f>& rot,
                   Eigen::MatrixXf& acc_cal,
                   std::vector<Eigen::Matrix3f>& ori_cal);
    // static Eigen::Matrix3f quat_to_rotmat(const Eigen::Vector4f& q);
    static std::vector<Eigen::Matrix3f> quat_batch_to_rotmats(const torch::Tensor& quat_batch);
    static Eigen::MatrixXf loadTposeCSV(const std::string& path);
    static Eigen::MatrixXf loadCoordCSV(const std::string& path);
private:
    Eigen::Matrix3f RMI;
    std::vector<Eigen::Matrix3f> RSB;
    Eigen::MatrixXf acc_offsets;

    std::vector<Eigen::Matrix3f> RIS;
    std::vector<Eigen::Vector3f> tpose_acc;
};
