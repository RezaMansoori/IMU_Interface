#pragma once

#include <Eigen/Dense>
#include <vector>

class CalibratorBase {
public:
    virtual ~CalibratorBase() = default;

    virtual void calibrate(const Eigen::MatrixXf& acc,
                           const std::vector<Eigen::Matrix3f>& rot,
                           Eigen::MatrixXf& acc_cal,
                           std::vector<Eigen::Matrix3f>& ori_cal) = 0;
};