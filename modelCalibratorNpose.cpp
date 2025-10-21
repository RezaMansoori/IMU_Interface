#include "modelCalibratorNpose.h"
#include "calibrator_utils.h"
#include <iostream>
#include <fstream>
#include <numeric>
#include <QMessageBox>
/*
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
        Eigen::Vector3f vec = V.row(i).transpose();
        Eigen::Vector3f prod = M * vec;
        result.row(i) = prod.transpose();
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
    Eigen::Vector4f q = q_raw.normalized();
    float w = q[3], x = q[0], y = q[1], z = q[2];

    Eigen::Matrix3f R;
    R << 1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
         2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
         2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y);
    return R;
}

std::vector<Eigen::Matrix3f> quat_batch_to_rotmat(const Eigen::MatrixXf& quats) {
    std::vector<Eigen::Matrix3f> result;
    for (int i = 0; i < quats.rows(); ++i) {
        result.push_back(quat_to_rotmat(quats.row(i)));
    }
    return result;
}
*/
NposeCalibrator::NposeCalibrator(const std::map<std::string, int>& selected_imus,
                                 const Eigen::MatrixXf& coord_data,
                                 const Eigen::MatrixXf& npose_data) {
    try {
        // === RMI ===
        Eigen::MatrixXf pelvis_quats = coord_data.bottomRows(800);
        Eigen::Vector4f avg_quat = pelvis_quats.colwise().mean();
        RMI = quat_to_rotmat(avg_quat).transpose();

        std::vector<std::string> joints = {"pelvis", "left_knee", "right_knee", "head", "left_hand", "right_hand"};
        RIS.reserve(joints.size());
        npose_acc.reserve(joints.size());

        for (const auto& joint : joints) {
            int id = selected_imus.at(joint);
            Eigen::MatrixXf acc = npose_data.block(id * 800, 0, 800, 3) * 9.81;
            Eigen::MatrixXf quat = npose_data.block(id * 800, 3, 800, 4);
            std::vector<Eigen::Matrix3f> R = quat_batch_to_rotmat(quat);

            Eigen::Matrix3f R_avg = Eigen::Matrix3f::Zero();
            for (const auto& Ri : R) R_avg += Ri;
            R_avg /= R.size();
            RIS.push_back(R_avg);

            Eigen::Vector3f mean_acc = Eigen::Vector3f::Zero();
            for (int i = 0; i < 800; ++i)
                mean_acc += R[i] * acc.row(i).transpose();
            mean_acc /= 800.0;
            npose_acc.push_back(mean_acc);
        }

        // === npose matrices ===
        std::vector<Eigen::Matrix3f> npose(6, Eigen::Matrix3f::Identity());
        npose[4] << 0, 1, 0,
                   -1, 0, 0,
                    0, 0, 1;
        npose[5] << 0, -1, 0,
                    1, 0, 0,
                    0, 0, 1;

        // === RSB[i] = RMI * RIS[i].mean().T * npose[i] ===
        RSB.clear();
        RSB.reserve(RIS.size());
        for (size_t i = 0; i < RIS.size(); ++i) {
            Eigen::Matrix3f rsb = (RMI * RIS[i]).transpose() * npose[i];
            RSB.push_back(rsb);
        }

        // === acc_offsets = RMI * npose_acc ===
        acc_offsets.resize(npose_acc.size(), 3);
        for (size_t i = 0; i < npose_acc.size(); ++i) {
            acc_offsets.row(i) = (RMI * npose_acc[i]).transpose();
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        QMessageBox::warning(nullptr, "Error", QString::fromStdString(e.what()));
    }
}
/*
void round_matrix(Eigen::Matrix3f& mat, int precision = 4) {
    float factor = std::pow(10, precision);
    mat = (mat * factor).array().round() / factor;
}
*/
void NposeCalibrator::calibrate(const Eigen::MatrixXf& acc,
                               const std::vector<Eigen::Matrix3f>& rot,
                               Eigen::MatrixXf& acc_cal,
                               std::vector<Eigen::Matrix3f>& ori_cal) {
    const int N = acc.rows();
    acc_cal.resize(N, 3);
    ori_cal.resize(N);

    acc_cal = batch_matvec_mul(rot, acc);
    acc_cal = apply_matrix_to_batch(RMI, acc_cal);

    for (int i = 0; i < N; ++i) {
        ori_cal[i] = RMI * rot[i];
    }

    torch::Tensor result = torch::zeros({6, 3, 3}, torch::kFloat32);
    for (int i = 0; i < 6; ++i) {
        round_matrix(ori_cal[i], 4);
        round_matrix(RSB[i], 4);
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                float sum = 0.0;
                for (int l = 0; l < 3; ++l) {
                    sum += ori_cal[i](j, l) * RSB[i](l, k);
                }
                result[i][j][k] = sum;
            }
        }
    }

    for (int i = 0; i < 6; ++i) {
        Eigen::Matrix3f tmp_result;
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                tmp_result(j, k) = result[i][j][k].item<float>();
            }
        }
        ori_cal[i] = tmp_result;
    }

    acc_cal = acc_cal - acc_offsets;
}

std::vector<Eigen::Matrix3f> NposeCalibrator::quat_batch_to_rotmats(const torch::Tensor& quat_batch) {
    auto qb = quat_batch.contiguous();
    float* data = qb.data_ptr<float>();
    int N = qb.size(0);

    std::vector<Eigen::Matrix3f> out;
    out.reserve(N);
    for (int i = 0; i < N; ++i) {
        Eigen::Vector4f q;
        q << data[4*i + 1], // x
             data[4*i + 2], // y
             data[4*i + 3], // z
             data[4*i + 0]; // w
        out.push_back(quat_to_rotmat(q));
    }
    return out;
}
