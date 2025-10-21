#include "modelCalibrator.h"
#include <iostream>
// #include <torch/torch.h>
using namespace Eigen;
#include <fstream>
#include <numeric>
#include <QMessageBox>
#include "calibrator_utils.h"
#include <QDebug>

// #include <vector>
// #include <Eigen/Dense>
// Equivalent to einsum("nij,nj->ni")
// einsum("nij,nj->ni")
/*
Eigen::MatrixXf batch_matvec_mul(const std::vector<Eigen::Matrix3f>& rot, const Eigen::MatrixXf& acc) {
    const int N = acc.rows();
    Eigen::MatrixXf result(N, 3);

    for (int i = 0; i < N; ++i) {
        result.row(i) = (rot[i] * acc.row(i).transpose()).transpose();
    }

    return result;
}

// einsum("ij,nj->ni")
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

// einsum("ij,njk->nik")
std::vector<Eigen::Matrix3f> left_multiply_batch(const Eigen::Matrix3f& M, const std::vector<Eigen::Matrix3f>& batch) {
    std::vector<Eigen::Matrix3f> result(batch.size());
    for (size_t i = 0; i < batch.size(); ++i)
        result[i] = M * batch[i];
    return result;
}

// Quaternion to rotation matrix
Eigen::Matrix3f quat_to_rotmat(const Eigen::Vector4f& q_raw) {
    Eigen::Vector4f q = q_raw.normalized(); // Normalize quaternion
    float w = q[3], x = q[0], y = q[1], z = q[2];

    Eigen::Matrix3f R;
    R << 1 - 2 * (y * y + z * z),     2 * (x * y - z * w),     2 * (x * z + y * w),
        2 * (x * y + z * w),     1 - 2 * (x * x + z * z),     2 * (y * z - x * w),
        2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y);
    return R;
}

// Batch quaternion to rotation
std::vector<Eigen::Matrix3f> quat_batch_to_rotmat(const Eigen::MatrixXf& quats) {
    std::vector<Eigen::Matrix3f> result;
    for (int i = 0; i < quats.rows(); ++i) {
        result.push_back(quat_to_rotmat(quats.row(i)));
    }
    return result;
}
*/
Calibrator::Calibrator(const std::map<std::string, int>& selected_imus,
                       const Eigen::MatrixXf& coord_data,
                       const Eigen::MatrixXf& tpose_data) {
    // === RMI ===
    try {
        int total_rows = coord_data.rows();
        int start_row = std::max(0, total_rows - 1000);
        int num_rows = total_rows - start_row;
        Eigen::MatrixXf pelvis_quats = coord_data.block(start_row, 0, num_rows, coord_data.cols());
        // Eigen::MatrixXf pelvis_quats = coord_data.bottomRows(1000);
        Eigen::Vector4f avg_quat = pelvis_quats.colwise().mean();
        RMI = quat_to_rotmat(avg_quat).transpose();
        qDebug() << "RMI Matrix:";
        for (int i = 0; i < 3; ++i) {
            qDebug() << "Row" << i << ":" << RMI(i, 0) << RMI(i, 1) << RMI(i, 2);
        }

        // RMI << -1, 0, 0,
        //             0, 0, 1,
        //             0, 1, 0;
        RMI = RMI.transpose();
        qDebug() << "RMI Matrix:";
        for (int i = 0; i < 3; ++i) {
            qDebug() << "Row" << i << ":" << RMI(i, 0) << RMI(i, 1) << RMI(i, 2);
        }
        std::vector<std::string> joints = {"pelvis", "left_knee", "right_knee", "head", "left_hand", "right_hand"};
        RIS.reserve(joints.size());
        tpose_acc.reserve(joints.size());
        for (const auto& joint : joints) {
            int id = selected_imus.at(joint);
            qDebug() << "ID for joint '" << joint << "':" << id;
            int start_row = id * 1000;
            int end_row = start_row + 1000;
            int valid_rows = 0;
            const float zero_tol = 1e-6f;

            for (int i = start_row; i < end_row; ++i) {
                auto row = tpose_data.row(i);
                bool all_zero = row.isZero(zero_tol);
                bool has_nan  = (row.array().isNaN()).any();
                if (!all_zero && !has_nan) ++valid_rows;

                // if (tpose_data.row(i).isZero()) break;
                // valid_rows++;
            }
            std::cout << "start row: " << start_row << "\n" << "end row" << end_row << std::endl;
            std::cout << "Valid Rows: " << valid_rows << std::endl;
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

            // Eigen::MatrixXf acc = tpose_data.block(start_row, 0, valid_rows, 3) * 9.81;
            // Eigen::MatrixXf quat = tpose_data.block(start_row, 3, valid_rows, 4);

        }

        for (const auto& joint : joints) {
            int id = selected_imus.at(joint);
            qDebug() << "ID for joint '" << joint << "':" << id;
            int start_row = id * 1000;
            int end_row = start_row + 1000;
            int valid_rows = 0;
            const float zero_tol = 1e-6f;

            for (int i = start_row; i < end_row; ++i) {
                auto row = tpose_data.row(i);
                bool all_zero = row.isZero(zero_tol);
                bool has_nan  = (row.array().isNaN()).any();
                if (!all_zero && !has_nan) ++valid_rows;

            //     if (tpose_data.row(i).isZero()) break;
            //     valid_rows++;
            }
            std::cout << "start row: " << start_row << "\n" << "end row" << end_row << std::endl;
            std::cout << "Valid Rows: " << valid_rows << std::endl;
            // Eigen::MatrixXf acc = tpose_data.block(start_row, 0, valid_rows, 3) * 9.81;
            // Eigen::MatrixXf quat = tpose_data.block(start_row, 3, valid_rows, 4);
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

            qDebug() << "Quaternions for joint ID" << id << "(1000 samples):";
            for (int i = 0; i < valid_rows; ++i) {
                qDebug() << "Quat[" << i << "]:" << quat(i, 0) << quat(i, 1) << quat(i, 2) << quat(i, 3);
            }

            std::vector<Eigen::Matrix3f> R = quat_batch_to_rotmat(quat);

            // Average rotation matrix
            Eigen::Matrix3f R_avg = Eigen::Matrix3f::Zero();
            int indx = 0;
            for (const auto& Ri : R) {
                qDebug() << "Ri[" << indx << "]:";
                for (int row = 0; row < 3; ++row) {
                    qDebug() << Ri(row, 0) << Ri(row, 1) << Ri(row, 2);
                }
                R_avg += Ri;
                indx++;
            }
            /*
            for (const auto& Ri : R) {
                R_avg += Ri;
                qDebug() << Ri;
            }
            */
            R_avg /= R.size();
            qDebug() << "R.size" << R.size();
            RIS.push_back(R_avg);

            // Average transformed acceleration
            Eigen::Vector3f mean_acc = Eigen::Vector3f::Zero();
            for (int i = 0; i < valid_rows; ++i)
                mean_acc += R[i] * acc.row(i).transpose();
            mean_acc /= valid_rows;
            tpose_acc.push_back(mean_acc);
        }

        // === RSB[i] = RMI * RIS[i].T ===
        RSB.clear();
        RSB.reserve(RIS.size());

        for (size_t i = 0; i < RIS.size(); ++i) {
            qDebug() << "RIS[" << i << "]:";
            for (int row = 0; row < 3; ++row) {
                qDebug() << RIS[i](row, 0) << RIS[i](row, 1) << RIS[i](row, 2);
            }

            Eigen::Matrix3f rsb = (RMI * RIS[i]).transpose();
            qDebug() << "RSB[" << i << "]:";
            for (int row = 0; row < 3; ++row) {
                qDebug() << rsb(row, 0) << rsb(row, 1) << rsb(row, 2);
            }
            RSB.push_back(rsb);
        }
        /*
        for (size_t i = 0; i < RIS.size(); ++i) {
                Eigen::Matrix3f rsb = (RMI * RIS[i]).transpose();
            RSB.push_back(rsb);

        }
        */
        // === acc_offsets = RMI * tpose_acc ===
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
/*
void round_matrix(Eigen::Matrix3f& mat, int precision = 4) {
    float factor = std::pow(10, precision);  // Scaling factor for rounding
    mat = (mat * factor).array().round() / factor;  // Round each element of the matrix
}
*/
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
    std::vector<Eigen::Matrix3f> ori_result(6); // To hold the result

    torch::Tensor result = torch::zeros({6, 3, 3}, torch::kFloat32);
    for (int i = 0; i < 6; ++i) {
        round_matrix(ori_cal[i], 4);
        round_matrix(RSB[i], 4);
    }
    // for (int i = 0; i < N; ++i) {
    //     std::cout << "pri cal[" << i << "]:\n" << ori_cal[i] << std::endl;
    // }
    // for (int i = 0; i < N; ++i) {
    //     std::cout << "rsb[" << i << "]:\n" << RSB[i] << std::endl;
    // }


    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                float sum = 0.0;
                for (int l = 0; l < 3; ++l) {
                    sum += ori_cal[i](j, l) * RSB[i](l, k);  // Use operator() to access matrix elements
                }
                result[i][j][k] = sum;
            }
        }
    }
    // Now, ori_result contains the result of the matrix multiplications
    // for (int i = 0; i < 6; ++i) {
    //     // ori_result[i] = ori_cal[i] * RSB[i];
    //     std::cout << "result" << result[i];
    // }

    for (int i = 0; i < 6; ++i) {
        // Convert the result tensor to Eigen::Matrix3f and assign it to ori_cal
        Eigen::Matrix3f tmp_result;
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                tmp_result(j, k) = result[i][j][k].item<float>();  // Extract value from tensor and assign to Eigen::Matrix3f
            }
        }
        ori_cal[i] = tmp_result;  // Assign the computed Eigen::Matrix3f to ori_cal
    }
    // === acc_cal - acc_offsets ===
    acc_cal = acc_cal - acc_offsets;
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
        q << data[4*i + 1],  // x
            data[4*i + 2],  // y
            data[4*i + 3],  // z
            data[4*i + 0];  // w
        out.push_back(quat_to_rotmat(q));
    }
    return out;
}
