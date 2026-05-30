#pragma once
#include <iostream>
#include <QDebug>
#include <Eigen/Dense>
#include <vector>
#include <stdexcept>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace lbk {

// ---------- Helpers ----------
inline Eigen::Matrix4f makeT(const Eigen::Matrix3f& R, const Eigen::Vector3f& p) {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.topLeftCorner<3,3>() = R;
    T.topRightCorner<3,1>() = p;
    return T;
}

// Intrinsic XYZ Euler (degrees) from rotation matrix (like MATLAB rotm2eul(R,'XYZ'))
inline Eigen::Vector3f eulerXYZdeg(const Eigen::Matrix3f& R) {
    const double r11 = R(0,0), r12 = R(0,1), r13 = R(0,2);
    const double r21 = R(1,0), r22 = R(1,1), r23 = R(1,2);
    const double r31 = R(2,0), r32 = R(2,1), r33 = R(2,2);

    const double sy = std::sqrt(r11*r11 + r21*r21);
    const bool singular = sy < 1e-8;

    double x, y, z; // rotations about X,Y,Z respectively
    if (!singular) {
        x = std::atan2(r32, r33);
        y = std::atan2(-r31, sy);
        z = std::atan2(r21, r11);
    } else {
        x = std::atan2(-r23, r22);
        y = std::atan2(-r31, sy);
        z = 0.0;
    }
    const double rad2deg = 180.0 / M_PI;
    return Eigen::Vector3f(x*rad2deg, y*rad2deg, z*rad2deg);
}

struct UpperFrameOutputs {
    // ================== World (N) transforms ==================
    Eigen::Matrix4f T_root_N;            // waist/root

    // upper

    Eigen::Matrix3f Ru_01_0, Ru_02_01;

    Eigen::Matrix3f Ru_r1_01; // shoulder wrt s01
    Eigen::Matrix3f Ru_r2_r1; // elbow wrt shoulder
    Eigen::Matrix3f Ru_r3_r2; // wrist wrt elbow
    Eigen::Matrix3f Ru_r4_r3; // hand wrt wrist

    // Left arm
    Eigen::Matrix3f Ru_l1_01; // shoulder wrt s01
    Eigen::Matrix3f Ru_l2_l1; // elbow wrt shoulder
    Eigen::Matrix3f Ru_l3_l2; // wrist wrt elbow
    Eigen::Matrix3f Ru_l4_l3; // hand wrt wrist


    // lower
    Eigen::Matrix3f Rl_r2_r1, Rl_r3_r2, Rl_r4_r3, Rl_l2_l1, Rl_l3_l2, Rl_l4_l3, Rl_r1_0, Rl_l1_0;
};
// void printMatrix3ff(const Eigen::Matrix3f& M, const QString& name)
// {
//     qDebug().noquote() << name;
//     for (int i = 0; i < 3; ++i)
//     {
//         qDebug().noquote() << QString("%1 %2 %3")
//         .arg(M(i,0), 0, 'f', 4)
//             .arg(M(i,1), 0, 'f', 4)
//             .arg(M(i,2), 0, 'f', 4);
//     }
// }
// namespace lbk {

// inline void printMatrix3ff(const Eigen::Matrix3f& M, const QString& name)
// {
//     qDebug().noquote() << name;
//     for (int i = 0; i < 3; ++i)
//     {
//         qDebug().noquote() << QString("%1 %2 %3")
//         .arg(M(i,0), 0, 'f', 4)
//             .arg(M(i,1), 0, 'f', 4)
//             .arg(M(i,2), 0, 'f', 4);
//     }
// }

// } // namespace lbk

// ---------- Main class ----------
class BodyKinematics {
public:
    explicit BodyKinematics()
    {
        Ru_s0_0_.setZero();    // pelvis/waist sensor -> body at T-pose
        // upper
        Ru_s01_01_.setZero();  // back sensor -> body
        Ru_s02_02_.setZero();  // head sensor -> body

        Ru_sr1_r1_.setZero(); Ru_sr2_r2_.setZero(); Ru_sr3_r3_.setZero();
        Ru_sl1_l1_.setZero(); Ru_sl2_l2_.setZero(); Ru_sl3_l3_.setZero();

        // lower
        Rl_sr1_r1_.setZero(); Rl_sr2_r2_.setZero(); Rl_sr3_r3_.setZero();
        Rl_sl1_l1_.setZero(); Rl_sl2_l2_.setZero(); Rl_sl3_l3_.setZero();

        foot_offset_.setIdentity();
    }

    // One-time calibration using T-pose
    // R_coord: coordinate alignment matrix (MATLAB R_coord)
    void calibrate(
        const Eigen::Matrix3f& R_coord,
        const Eigen::Matrix3f& R_Tpose_pelvis,
        const Eigen::Matrix3f& R_Tpose_back,
        const Eigen::Matrix3f& R_Tpose_head,
        const Eigen::Matrix3f& R_Tpose_armL,
        const Eigen::Matrix3f& R_Tpose_armR,
        const Eigen::Matrix3f& R_Tpose_forearmL,
        const Eigen::Matrix3f& R_Tpose_forearmR,
        const Eigen::Matrix3f& R_Tpose_handL,
        const Eigen::Matrix3f& R_Tpose_handR,

        const Eigen::Matrix3f& R_Tpose_thighL,
        const Eigen::Matrix3f& R_Tpose_thighR,
        const Eigen::Matrix3f& R_Tpose_shankL,
        const Eigen::Matrix3f& R_Tpose_shankR,
        bool use_foot_from_pelvis = true,
        const Eigen::Matrix3f& foot_offset = (Eigen::Matrix3f() <<
                                                  1,0,0,  0,0,1,  0,-1,0).finished(),
        const Eigen::Matrix3f* R_Tpose_footL = nullptr,
        const Eigen::Matrix3f* R_Tpose_footR = nullptr
        ) {
        qDebug() << "starting calibration";
        // upper
        Eigen::Matrix3f Rp = R_Tpose_pelvis;
        auto printMatrix = [](const Eigen::Matrix3f& M, const QString& name) {
            qDebug() << name;
            for (int i = 0; i < 3; ++i) {
                qDebug() << QString("%1 %2 %3")
                .arg(M(i,0),0,'f',4)
                    .arg(M(i,1),0,'f',4)
                    .arg(M(i,2),0,'f',4);
            }
        };
        // Body relative to sensor at T-pose: R_BS = R_Tpose^T * R_coord
        const Eigen::Matrix3f R_BS_pelvis   = R_Tpose_pelvis.transpose()   * R_coord;
        const Eigen::Matrix3f R_BS_back     = R_Tpose_back.transpose()     * R_coord;
        const Eigen::Matrix3f R_BS_head     = R_Tpose_head.transpose()     * R_coord;

        const Eigen::Matrix3f R_BS_armL     = R_Tpose_armL.transpose()     * R_coord;
        const Eigen::Matrix3f R_BS_armR     = R_Tpose_armR.transpose()     * R_coord;

        const Eigen::Matrix3f R_BS_forearmL = R_Tpose_forearmL.transpose() * R_coord;
        const Eigen::Matrix3f R_BS_forearmR = R_Tpose_forearmR.transpose() * R_coord;

        const Eigen::Matrix3f R_BS_handL    = R_Tpose_handL.transpose()    * R_coord;
        const Eigen::Matrix3f R_BS_handR    = R_Tpose_handR.transpose()    * R_coord;
        // qDebug() << "starting calibration2";
        // printMatrix(R_BS_pelvis,   "R_BS_pelvis:");
        // printMatrix(R_BS_back,     "R_BS_back:");
        // printMatrix(R_BS_head,     "R_BS_head:");

        // printMatrix(R_BS_armL,     "R_BS_armL:");
        // printMatrix(R_BS_armR,     "R_BS_armR:");

        // printMatrix(R_BS_forearmL, "R_BS_forearmL:");
        // printMatrix(R_BS_forearmR, "R_BS_forearmR:");

        // printMatrix(R_BS_handL,    "R_BS_handL:");
        // printMatrix(R_BS_handR,    "R_BS_handR:");

        // Sensor -> body (transpose of R_BS)
        Ru_s0_0_   = R_BS_pelvis.transpose();
        Ru_s01_01_ = R_BS_back.transpose();
        Ru_s02_02_ = R_BS_head.transpose();

        // right
        Ru_sr1_r1_ = R_BS_armR.transpose();
        Ru_sr2_r2_ = R_BS_forearmR.transpose();
        Ru_sr3_r3_ = R_BS_handR.transpose();

        // left
        Ru_sl1_l1_ = R_BS_armL.transpose();
        Ru_sl2_l2_ = R_BS_forearmL.transpose();
        Ru_sl3_l3_ = R_BS_handL.transpose();

        // lower
        Eigen::Matrix3f RtL = R_Tpose_thighL;
        Eigen::Matrix3f RtR = R_Tpose_thighR;
        Eigen::Matrix3f RsL = R_Tpose_shankL;
        Eigen::Matrix3f RsR = R_Tpose_shankR;

        Eigen::Matrix3f RfL, RfR;
        if (use_foot_from_pelvis) {
            RfL = Rp * foot_offset;
            RfR = Rp * foot_offset;
            foot_offset_ = foot_offset;
        } else {
            if (!R_Tpose_footL || !R_Tpose_footR)
                throw std::invalid_argument("Provide R_Tpose_footL/R or set use_foot_from_pelvis=true.");
            RfL = *R_Tpose_footL;
            RfR = *R_Tpose_footR;
        }
        qDebug() << "starting calibration4";

        // Body relative to sensor at T-pose: R_BS = R_Tpose^T * R_coord
        // Eigen::Matrix3f R_BS_pelvis = Rp.transpose() * R_coord;
        Eigen::Matrix3f R_BS_thighL = RtL.transpose() * R_coord;
        Eigen::Matrix3f R_BS_thighR = RtR.transpose() * R_coord;
        Eigen::Matrix3f R_BS_shankL = RsL.transpose() * R_coord;
        Eigen::Matrix3f R_BS_shankR = RsR.transpose() * R_coord;
        Eigen::Matrix3f R_BS_footL  = RfL.transpose() * R_coord;
        Eigen::Matrix3f R_BS_footR  = RfR.transpose() * R_coord;

        // Sensor -> body (transpose of R_BS)
        Rl_s0_0_   = R_BS_pelvis.transpose();

        Rl_sr1_r1_ = R_BS_thighR.transpose();
        Rl_sr2_r2_ = R_BS_shankR.transpose();
        Rl_sr3_r3_ = R_BS_footR.transpose();

        Rl_sl1_l1_ = R_BS_thighL.transpose();
        Rl_sl2_l2_ = R_BS_shankL.transpose();
        Rl_sl3_l3_ = R_BS_footL.transpose();
        qDebug() << "calibration finished";
    }

    // Compute one frame
    // Inputs are world->sensor rotations for each IMU (matching the MATLAB loop variables)
    UpperFrameOutputs computeFramee(
        const Eigen::Matrix3f& R_pelvis,    // waist (s0)
        const Eigen::Matrix3f& R_back,      // s01
        const Eigen::Matrix3f& R_head,      // s02
        const Eigen::Matrix3f& R_armL,      // sl1
        const Eigen::Matrix3f& R_armR,      // sr1
        const Eigen::Matrix3f& R_forearmL,  // sl2
        const Eigen::Matrix3f& R_forearmR,  // sr2
        const Eigen::Matrix3f& R_handL,     // sl3
        const Eigen::Matrix3f& R_handR,     // sr3

        const Eigen::Matrix3f& R_thighL,
        const Eigen::Matrix3f& R_thighR,
        const Eigen::Matrix3f& R_shankL,
        const Eigen::Matrix3f& R_shankR,
        const Eigen::Matrix3f* R_footL = nullptr,
        const Eigen::Matrix3f* R_footR = nullptr,
        bool use_foot_offset_if_none = true
        ) const {

        auto printMat = [](const Eigen::Matrix3f& M, const QString& name) {
            qDebug().noquote() << name;
            for (int i = 0; i < 3; ++i)
                qDebug().noquote() << QString("%1 %2 %3")
                                          .arg(M(i,0),0,'f',4)
                                          .arg(M(i,1),0,'f',4)
                                          .arg(M(i,2),0,'f',4);
        };
        // qDebug() << "printing rotationd input";
        // // --- print all input rotations ---
        // printMat(R_pelvis,    "R_pelvis");
        // printMat(R_back,      "R_back");
        // printMat(R_head,      "R_head");
        // printMat(R_armL,      "R_armL");
        // printMat(R_armR,      "R_armR");
        // printMat(R_forearmL,  "R_forearmL");
        // printMat(R_forearmR,  "R_forearmR");
        // printMat(R_handL,     "R_handL");
        // printMat(R_handR,     "R_handR");
        // printMat(R_thighL,    "R_thighL");
        // printMat(R_thighR,    "R_thighR");
        // printMat(R_shankL,    "R_shankL");
        // printMat(R_shankR,    "R_shankR");

        ensureCalibrated_();
        // upper
        // -------- part 1: name them like MATLAB --------
        const Eigen::Matrix3f Ru_s0_N  = R_pelvis;
        const Eigen::Matrix3f Ru_s01_N = R_back;
        const Eigen::Matrix3f Ru_s02_N = R_head;

        const Eigen::Matrix3f Ru_sr1_N = R_armR;
        const Eigen::Matrix3f Ru_sr2_N = R_forearmR;
        const Eigen::Matrix3f Ru_sr3_N = R_handR;

        const Eigen::Matrix3f Ru_sl1_N = R_armL;
        const Eigen::Matrix3f Ru_sl2_N = R_forearmL;
        const Eigen::Matrix3f Ru_sl3_N = R_handL;

        // -------- part 2: sensors relative --------
        const Eigen::Matrix3f Ru_s01_s0  = Ru_s0_N.transpose()  * Ru_s01_N;
        const Eigen::Matrix3f Ru_s02_s01 = Ru_s01_N.transpose() * Ru_s02_N;

        // right
        const Eigen::Matrix3f Ru_sr1_s01 = Ru_s01_N.transpose() * Ru_sr1_N;
        const Eigen::Matrix3f Ru_sr2_sr1 = Ru_sr1_N.transpose() * Ru_sr2_N;
        const Eigen::Matrix3f Ru_sr3_sr2 = Ru_sr2_N.transpose() * Ru_sr3_N;

        // left
        const Eigen::Matrix3f Ru_sl1_s01 = Ru_s01_N.transpose() * Ru_sl1_N;
        const Eigen::Matrix3f Ru_sl2_sl1 = Ru_sl1_N.transpose() * Ru_sl2_N;
        const Eigen::Matrix3f Ru_sl3_sl2 = Ru_sl2_N.transpose() * Ru_sl3_N;

        // -------- part 3: body relative (using calibration R_SB) --------
        const Eigen::Matrix3f Ru_01_0    = Ru_s0_0_   * Ru_s01_s0  * Ru_s01_01_.transpose();
        const Eigen::Matrix3f Ru_02_01   = Ru_s01_01_ * Ru_s02_s01 * Ru_s02_02_.transpose();

        // right arm chain
        const Eigen::Matrix3f Ru_r1_01 = Ru_s01_01_ * Ru_sr1_s01 * Ru_sr1_r1_.transpose();
        const Eigen::Matrix3f Ru_r2_r1 = Ru_sr1_r1_ * Ru_sr2_sr1 * Ru_sr2_r2_.transpose();
        const Eigen::Matrix3f Ru_r3_r2 = Ru_sr2_r2_ * Ru_sr3_sr2 * Ru_sr3_r3_.transpose();
        const Eigen::Matrix3f Ru_r4_r3 = Eigen::Matrix3f::Identity();
        UpperFrameOutputs out;

        // left arm chain
        const Eigen::Matrix3f Ru_l1_01 = Ru_s01_01_ * Ru_sl1_s01 * Ru_sl1_l1_.transpose();
        const Eigen::Matrix3f Ru_l2_l1 = Ru_sl1_l1_ * Ru_sl2_sl1 * Ru_sl2_l2_.transpose();
        const Eigen::Matrix3f Ru_l3_l2 = Ru_sl2_l2_ * Ru_sl3_sl2 * Ru_sl3_l3_.transpose();
        const Eigen::Matrix3f Ru_l4_l3 = Eigen::Matrix3f::Identity();

        // -------- part 4: build local transforms (0-frame) --------

        out.Ru_01_0  = Ru_01_0;
        out.Ru_02_01 = Ru_02_01;

        // right
        out.Ru_r1_01 = Ru_r1_01;
        out.Ru_r2_r1 = Ru_r2_r1;
        out.Ru_r3_r2 = Ru_r3_r2;
        out.Ru_r4_r3 = Ru_r4_r3;

        // left
        out.Ru_l1_01 = Ru_l1_01;
        out.Ru_l2_l1 = Ru_l2_l1;
        out.Ru_l3_l2 = Ru_l3_l2;
        out.Ru_l4_l3 = Ru_l4_l3;

        const Eigen::Matrix3f R_0_N = Ru_s0_N * Ru_s0_0_.transpose();
        const Eigen::Matrix4f T_0_N = makeT(R_0_N, Eigen::Vector3f::Zero());

        out.T_root_N = T_0_N;

        // lower
        // Use pelvis + foot_offset if feet not provided
        Eigen::Matrix3f RfL, RfR;
        if (!R_footL || !R_footR) {
            if (!use_foot_offset_if_none)
                throw std::invalid_argument("Foot rotations not provided and use_foot_offset_if_none=false.");
            RfL = R_pelvis * (Eigen::Matrix3f() << 1,0,0, 0,0,1, 0,-1,0).finished();
            RfR = R_pelvis * (Eigen::Matrix3f() << 1,0,0, 0,0,1, 0,-1,0).finished();
        } else {
            RfL = *R_footL; RfR = *R_footR;
        }

        // Sensors relative
        // Right chain
        Eigen::Matrix3f Rl_sr1_s0  = R_pelvis.transpose() * R_thighR;
        Eigen::Matrix3f Rl_sr2_sr1 = R_thighR.transpose()  * R_shankR;
        Eigen::Matrix3f Rl_sr3_sr2 = R_shankR.transpose()  * RfR;
        // Left chain
        Eigen::Matrix3f Rl_sl1_s0  = R_pelvis.transpose() * R_thighL;
        Eigen::Matrix3f Rl_sl2_sl1 = R_thighL.transpose() * R_shankL;
        Eigen::Matrix3f Rl_sl3_sl2 = R_shankL.transpose() * RfL;

        // Body relative (using calibration)
        // Right
        Eigen::Matrix3f Rl_r1_0  = Rl_s0_0_   * Rl_sr1_s0  * Rl_sr1_r1_.transpose();
        Eigen::Matrix3f Rl_r2_r1 = Rl_sr1_r1_ * Rl_sr2_sr1 * Rl_sr2_r2_.transpose();
        Eigen::Matrix3f Rl_r3_r2 = Rl_sr2_r2_ * Rl_sr3_sr2 * Rl_sr3_r3_.transpose();
        Eigen::Matrix3f Rl_r4_r3 = Eigen::Matrix3f::Identity();
        // Left
        Eigen::Matrix3f Rl_l1_0  = Rl_s0_0_   * Rl_sl1_s0  * Rl_sl1_l1_.transpose();
        Eigen::Matrix3f Rl_l2_l1 = Rl_sl1_l1_ * Rl_sl2_sl1 * Rl_sl2_l2_.transpose();
        Eigen::Matrix3f Rl_l3_l2 = Rl_sl2_l2_ * Rl_sl3_sl2 * Rl_sl3_l3_.transpose();
        Eigen::Matrix3f Rl_l4_l3 = Eigen::Matrix3f::Identity();

        // FrameOutputs out;

        out.Rl_r1_0 = Rl_r1_0;
        out.Rl_r2_r1 = Rl_r2_r1;
        out.Rl_r3_r2 = Rl_r3_r2;
        out.Rl_r4_r3 = Rl_r4_r3;

        // left
        out.Rl_l1_0 = Rl_l1_0;
        out.Rl_l2_l1 = Rl_l2_l1;
        out.Rl_l3_l2 = Rl_l3_l2;
        out.Rl_l4_l3 = Rl_l4_l3;

        return out;
    }

    // // Convenience wrapper: many frames
    // std::vector<UpperFrameOutputs> computeSequence(
    //     const std::vector<Eigen::Matrix3f>& R_pelvis,
    //     const std::vector<Eigen::Matrix3f>& R_back,
    //     const std::vector<Eigen::Matrix3f>& R_head,
    //     const std::vector<Eigen::Matrix3f>& R_armL,
    //     const std::vector<Eigen::Matrix3f>& R_armR,
    //     const std::vector<Eigen::Matrix3f>& R_forearmL,
    //     const std::vector<Eigen::Matrix3f>& R_forearmR,
    //     const std::vector<Eigen::Matrix3f>& R_handL,
    //     const std::vector<Eigen::Matrix3f>& R_handR
    //     ) const {
    //     const std::size_t N = std::min({
    //         R_pelvis.size(), R_back.size(), R_head.size(),
    //         R_armL.size(), R_armR.size(), R_forearmL.size(),
    //         R_forearmR.size(), R_handL.size(), R_handR.size()
    //     });
    //     std::vector<UpperFrameOutputs> out;
    //     out.reserve(N);
    //     for (std::size_t i = 0; i < N; ++i) {
    //         out.emplace_back(
    //             computeFramee(
    //                 R_pelvis[i], R_back[i], R_head[i],
    //                 R_armL[i], R_armR[i],
    //                 R_forearmL[i], R_forearmR[i],
    //                 R_handL[i], R_handR[i]
    //                 )
    //             );
    //     }
    //     return out;
    // }

private:
    void ensureCalibrated_() const {
        if (Ru_s0_0_.isZero(0)  || Ru_s01_01_.isZero(0) || Ru_s02_02_.isZero(0) ||
            Ru_sr1_r1_.isZero(0)|| Ru_sr2_r2_.isZero(0)|| Ru_sr3_r3_.isZero(0) ||
            Ru_sl1_l1_.isZero(0)|| Ru_sl2_l2_.isZero(0)|| Ru_sl3_l3_.isZero(0) ||

            Rl_sr1_r1_.isZero(0) || Rl_sr2_r2_.isZero(0) || Rl_sr3_r3_.isZero(0) ||
            Rl_sl1_l1_.isZero(0) || Rl_sl2_l2_.isZero(0) || Rl_sl3_l3_.isZero(0)) {
            throw std::runtime_error("UpperBodyKinematics: not calibrated. Call calibrate(...) first.");
        }
    }

    // Sensor->body from calibration
    Eigen::Matrix3f Ru_s0_0_;    // pelvis/waist
    Eigen::Matrix3f Ru_s01_01_;  // back
    Eigen::Matrix3f Ru_s02_02_;  // head

    // Right chain
    Eigen::Matrix3f Ru_sr1_r1_, Ru_sr2_r2_, Ru_sr3_r3_;
    // Left chain
    Eigen::Matrix3f Ru_sl1_l1_, Ru_sl2_l2_, Ru_sl3_l3_;

    Eigen::Matrix3f foot_offset_;

    // Sensor->body from calibration
    Eigen::Matrix3f Rl_s0_0_;
    Eigen::Matrix3f Rl_sr1_r1_, Rl_sr2_r2_, Rl_sr3_r3_;
    Eigen::Matrix3f Rl_sl1_l1_, Rl_sl2_l2_, Rl_sl3_l3_;
};

} // namespace lbk
