#include "modelCalibrator.h"
#include <iostream>
// #include <torch/torch.h>
using namespace Eigen;
#include <fstream>
#include <numeric>
#include <QMessageBox>
#include "calibrator_utils.h"

Calibrator::Calibrator(const std::map<std::string, int>& selected_imus,
                       const Eigen::MatrixXf& coord_data,
                       const Eigen::MatrixXf& tpose_data,
                       std::vector<Eigen::Vector4f> avg_quatt) {
    // === RMI ===
    try {
        int total_rows = coord_data.rows();
        int start_row = std::max(0, total_rows - 1000);
        int num_rows = total_rows - start_row;
        int valid_rows = 0;
        const float zero_tol = 1e-6f;
        for (int i = 0; i < 1000; ++i) {
            auto row = coord_data.row(i);
            bool all_zero = row.isZero(zero_tol);
            bool has_nan  = (row.array().isNaN()).any();
            if (!all_zero && !has_nan) ++valid_rows;
        }
        Eigen::MatrixXf pelvis_quats = coord_data.block(start_row, 0, valid_rows, coord_data.cols());
        int N = pelvis_quats.rows();
        // Average quaternions first (not rotation matrices!), then convert to rotation matrix
        Eigen::Vector4f sum_quat = Eigen::Vector4f::Zero();
        for (int i = 0; i < N; ++i) {
            sum_quat += pelvis_quats.row(i).transpose();
        }
        Eigen::Vector4f avg_quat = sum_quat / static_cast<float>(N);
        avg_quat.normalize(); // Normalize the averaged quaternion
        Eigen::Matrix3f avg_rot = quat_to_rotmat(avg_quat);
        RMI = avg_rot.transpose();
        
        // Debug: Check if RMI is a valid rotation matrix
        float det_RMI = RMI.determinant();

        std::vector<std::string> joints = {"pelvis", "left_knee", "right_knee", "head", "left_hand", "right_hand"};
        RIS.reserve(joints.size());
        tpose_acc.reserve(joints.size());

        for (const auto& joint : joints) {
            int id = selected_imus.at(joint);
            int start_row = id * 1000;
            int end_row = start_row + 1000;
            int valid_rows = 0;
            const float zero_tol = 1e-6f;

            for (int i = start_row; i < end_row; ++i) {
                auto row = tpose_data.row(i);
                bool all_zero = row.isZero(zero_tol);
                bool has_nan  = (row.array().isNaN()).any();
                if (!all_zero && !has_nan) ++valid_rows;
            }
            Eigen::MatrixXf acc(valid_rows, 3);
            Eigen::MatrixXf quat(valid_rows, 4);

            int out = 0;
            for (int i = start_row; i < end_row; ++i) {
                auto row = tpose_data.row(i);
                bool all_zero = row.isZero(zero_tol);
                bool has_nan  = (row.array().isNaN()).any();
                if (!all_zero && !has_nan) {
                    acc.row(out)  = row.segment<3>(0).transpose() * 9.81f; // ستون 0..2
                    quat.row(out) = row.segment<4>(3).transpose();        // ستون 3..6
                    ++out;
                }
            }
            std::vector<Eigen::Matrix3f> R = quat_batch_to_rotmat(quat);
            // Average rotation matrix
            Eigen::Matrix3f R_avg = Eigen::Matrix3f::Zero();
            int indx = 0;
            for (const auto& Ri : R) {
                R_avg += Ri;
                indx++;
            }
            R_avg /= R.size();
            // Debug: Check if R_avg is a valid rotation matrix
            float det_R_avg = R_avg.determinant();
            qDebug() << "RIS[" << RIS.size() << "] (R_avg) determinant:" << det_R_avg << "(should be ~1.0)";
            RIS.push_back(R_avg);
            RIS.push_back(quat_to_rotmat(avg_quatt[id]));
            Eigen::Vector3f mean_acc = Eigen::Vector3f::Zero();
            // Average transformed acceleration
            for (int i = 0; i < 1000; ++i)
                mean_acc += R[i] * acc.row(i).transpose();
            mean_acc /= 1000.0;
            tpose_acc.push_back(mean_acc);
        }
        // === self.RSB = self.RMI.matmul(self.RIS.mean(dim=1)).transpose(1, 2).matmul(eye(3, dtype=float)) ===
        RSB.clear();
        RSB.reserve(RIS.size());

        for (size_t i = 0; i < RIS.size(); ++i) {
            Eigen::Matrix3f rsb = (RMI * RIS[i]).transpose();
            RSB.push_back(rsb);
        }
        // === self.acc_offsets = einsum("ij,nj->ni", self.RMI, self.tpose_acc) ===
        acc_offsets.resize(tpose_acc.size(), 3);
        for (size_t i = 0; i < tpose_acc.size(); ++i) {
            acc_offsets.row(i) = (RMI * tpose_acc[i]).transpose();
        }
    } catch (const c10::Error &e) {
        qWarning() << "c10: " << e.what();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        QMessageBox::warning(nullptr, "Error", QString::fromStdString(e.what()));
    }
}

void Calibrator::calibrate(const Eigen::MatrixXf& acc,
                           const std::vector<Eigen::Matrix3f>& rot,
                           Eigen::MatrixXf& acc_cal,
                           std::vector<Eigen::Matrix3f>& ori_cal) {
    const int N = acc.rows();
    acc_cal.resize(N, 3);
    ori_cal.resize(N);
    // === einsum("nij,nj->ni")
    acc_cal = batch_matvec_mul(rot, acc);
    // === einsum("ij,nj->ni")
    acc_cal = apply_matrix_to_batch(RMI, acc_cal);

    for (int i = 0; i < N; ++i) {
        ori_cal[i] = RMI * rot[i];
    }
    
    // Apply RSB transformation: ori_cal[i] = ori_cal[i] * RSB[i]
    // RSB has 6 elements (one per IMU), so we use the size of RSB
    for (size_t i = 0; i < RSB.size() && i < ori_cal.size(); ++i) {
        ori_cal[i] = ori_cal[i] * RSB[i];
    }
    
    // === acc_cal - acc_offsets ===
    acc_cal = acc_cal - acc_offsets;

    // Debug output
    // qDebug() << "acc_cal:";
    // for (int row = 0; row < acc_cal.rows(); ++row) {
    //     qDebug() << QString("Row %1: %2, %3, %4")
    //     .arg(row)
    //         .arg(acc_cal(row, 0), 0, 'f', 6)
    //         .arg(acc_cal(row, 1), 0, 'f', 6)
    //         .arg(acc_cal(row, 2), 0, 'f', 6);
    // }
    // qDebug() << "ori_cal:";
    // for (size_t i = 0; i < ori_cal.size(); ++i) {
    //     qDebug() << QString("ori_cal[%1]:").arg(i);
    //     for (int row = 0; row < 3; ++row) {
    //         qDebug() << QString("  Row %1: %2, %3, %4")
    //         .arg(row)
    //             .arg(ori_cal[i](row, 0), 0, 'f', 6)
    //             .arg(ori_cal[i](row, 1), 0, 'f', 6)
    //             .arg(ori_cal[i](row, 2), 0, 'f', 6);
    //     }
    // }
}

// Matrix3f Calibrator::quat_to_rotmat(const Vector4f& q) {
//     Quaterniond quat(q(3), q(0), q(1), q(2));
//     return quat.normalized().toRotationMatrix();
// }

std::vector<Eigen::Matrix3f> Calibrator::quat_batch_to_rotmats(const torch::Tensor& quat_batch) {
    auto qb = quat_batch.contiguous();
    float* data = qb.data_ptr<float>();
    int N = qb.size(0);

    std::vector<Eigen::Matrix3f> out;
    out.reserve(N);
    for (int i = 0; i < N; ++i) {
        Eigen::Vector4f q;
        q << data[4*i + 0],  // x
            data[4*i + 1],  // y
            data[4*i + 2],  // z
            data[4*i + 3];  // w
        out.push_back(quat_to_rotmat(q));
    }
    return out;
}
