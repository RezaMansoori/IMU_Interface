#include "ProcessingWorker.h"
#include <QDebug>

void printPose3D(const std::vector<std::vector<Eigen::Matrix3f>>& pose3D, int maxFrames = 5) {
    int T = std::min((int)pose3D.size(), maxFrames);
    int imuCount = pose3D[0].size();

    for (int t = 0; t < T; ++t) {
        std::cout << "=== Frame " << t << " ===\n";
        for (int j = 0; j < imuCount; ++j) {
            std::cout << "IMU " << j << " rotation:\n" << pose3D[t][j] << "\n\n";
        }
    }
}
Eigen::Matrix3f quatToRot(float w, float x, float y, float z) {
    Eigen::Quaternionf q(w, x, y, z);  // Note: Eigen uses (w, x, y, z) order
    q.normalize();
    return q.toRotationMatrix();
}
void printAvgQuats(const std::vector<Eigen::Vector4f>& avg_quat)
{
    for (size_t i = 0; i < avg_quat.size(); ++i)
    {
        const auto& q = avg_quat[i];
        // qDebug().nospace()
        //     << "avg_quat[" << i << "]: ("
        //     << q(0) << ", "
        //     << q(1) << ", "
        //     << q(2) << ", "
        //     << q(3) << ")";
    }
}

void ProcessingWorker::calibrateKin(
    const std::map<std::string, int>& selected_imus,
    /*const Eigen::MatrixXf pose_dataKin*/
    std::vector<Eigen::Vector4f> avg_quat)
{
    imuCount = selected_imus.size();
    printAvgQuats(avg_quat);
    qDebug() << "printAvgQuats called, size =" << avg_quat.size();

    this->selected_imusKin = selected_imus;
    // Eigen::Matrix3f R_coord = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f R_coord;
    R_coord << -1, 0, 0,
        0, 0, 1,
        0, 1, 0;

    // مقداردهی پیش‌فرض (در صورت نبودن در selected_imusKin)
    Eigen::Matrix3f R_pelvis   = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f R_back     = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f R_head     = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f R_armL     = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f R_armR     = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f R_forearmL = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f R_forearmR = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f R_handL    = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f R_handR    = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f R_thighL   = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f R_thighR   = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f R_shankL   = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f R_shankR   = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f R_footL   = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f R_footR   = Eigen::Matrix3f::Identity();

    // helper برای استخراج ماتریس 3x3 از pose_dataKin
    // auto extractRotation = [&](const std::string& name) -> Eigen::Matrix3f {
    //     auto it = selected_imusKin.find(name);
    //     if (it == selected_imusKin.end()) return Eigen::Matrix3f::Identity();

    //     int idx = it->second;
    //     // if (idx >= avgRot.size()) return Eigen::Matrix3f::Identity();

    //     // return avgRot[idx];  // use the averaged 3x3 rotation
    //     //eshtebahe
    //     return avg_quats;
    // };
    auto getRot = [&](const std::string& name) -> Eigen::Matrix3f {
        auto it = selected_imus.find(name);
        if (it == selected_imus.end()) {
            qDebug() << "⚠️ IMU not found:" << QString::fromStdString(name);
            return Eigen::Matrix3f::Identity();
        }

        int idx = it->second;
        if (idx < 0 || idx >= avg_quat.size()) {
            qDebug() << "⚠️ Invalid IMU index for" << QString::fromStdString(name)
                     << ":" << idx << " (size =" << avg_quat.size() << ")";
            return Eigen::Matrix3f::Identity();
        }

        const auto& q = avg_quat[idx];
        // if (q.norm() < 1e-6f) {
        //     qDebug() << "⚠️ Zero quaternion for" << QString::fromStdString(name);
        //     return Eigen::Matrix3f::Identity();
        // }
        qDebug() << name;
        qDebug().noquote() << QString("Quat for %1: [%2, %3, %4, %5]")
                                  .arg(QString::fromStdString(name))
                                  .arg(q(0), 0, 'f', 4)
                                  .arg(q(1), 0, 'f', 4)
                                  .arg(q(2), 0, 'f', 4)
                                  .arg(q(3), 0, 'f', 4);

        return quatToRot(q(0), q(1), q(2), q(3));
    };

    // helper to test if a key exists (C++17 style)
    auto hasIMU = [&](const std::string& name) -> bool {
        return selected_imus.find(name) != selected_imus.end();
    };

    // only set rotation if IMU exists
    if (hasIMU("pelvis"))   R_pelvis   = getRot("pelvis");
    if (hasIMU("back"))     R_back     = getRot("back");
    if (hasIMU("head"))     R_head     = getRot("head");
    if (hasIMU("arml"))     R_armL     = getRot("arml");
    if (hasIMU("armr"))     R_armR     = getRot("armr");
    if (hasIMU("forearml")) R_forearmL = getRot("forearml");
    if (hasIMU("forearmr")) R_forearmR = getRot("forearmr");
    if (hasIMU("handl"))    R_handL    = getRot("handl");
    if (hasIMU("handr"))    R_handR    = getRot("handr");
    if (hasIMU("thighl"))   R_thighL   = getRot("thighl");
    if (hasIMU("thighr"))   R_thighR   = getRot("thighr");
    if (hasIMU("shankl"))   R_shankL   = getRot("shankl");
    if (hasIMU("shankr"))   R_shankR   = getRot("shankr");
    if (hasIMU("footl"))   R_footL   = getRot("footl");
    if (hasIMU("footr"))   R_footR   = getRot("footr");

    // qDebug() << "✅ Calibration rotations computed successfully.";
    qDebug() << "=== Calibration Matrix Shapes ===";
    auto printMatrix = [](const Eigen::Matrix3f& M, const QString& name) {
        qDebug() << name;
        for (int i = 0; i < 3; ++i) {
            qDebug() << QString("%1 %2 %3")
                            .arg(M(i,0),0,'f',4)
                            .arg(M(i,1),0,'f',4)
                            .arg(M(i,2),0,'f',4);
        }
    };

    // printMatrix(R_coord, "R_coord");
    // printMatrix(R_pelvis, "R_pelvis");
    // printMatrix(R_back, "R_back");
    // printMatrix(R_head, "R_head");
    // printMatrix(R_armL, "R_armL");
    // printMatrix(R_armR, "R_armR");
    // printMatrix(R_forearmL, "R_forearmL");
    // printMatrix(R_forearmR, "R_forearmR");
    // printMatrix(R_handL, "R_handL");
    // printMatrix(R_handR, "R_handR");
    // printMatrix(R_thighL, "R_thighL");
    // printMatrix(R_thighR, "R_thighR");
    // printMatrix(R_shankL, "R_shankL");
    // printMatrix(R_shankR, "R_shankR");
    // printMatrix(R_footL, "R_footL");
    // printMatrix(R_footR, "R_footR");


    Eigen::Matrix3f foot_offset;
    foot_offset <<
        1, 0, 0,
        0, 0, 1,
        0, -1, 0;

    bodyKinematics_->calibrate(
        R_coord,
        R_pelvis,
        R_back,
        R_head,
        R_armL,
        R_armR,
        R_forearmL,
        R_forearmR,
        R_handL,
        R_handR,
        R_thighL,
        R_thighR,
        R_shankL,
        R_shankR,
        false// use_foot_from_pelvis
        , foot_offset
        , &R_footL
        , &R_footR
        );
    // bodyKinematics_->calibrate(
    //     R_coord,
    //     R_pelvis,
    //     R_back,
    //     R_head,
    //     R_armL,
    //     R_armR,
    //     R_forearmL,
    //     R_forearmR,
    //     R_handL,
    //     R_handR,
    //     R_thighL,
    //     R_thighR,
    //     R_shankL,
    //     R_shankR,
    //     true  // use_foot_from_pelvis
    //     );
}

ProcessingWorker::ProcessingWorker(std::shared_ptr<torch::jit::script::Module> model,
                                   SMPLModel* smplModel,
                                   std::shared_ptr<CausalAvgImpl> causal_avg,
                                   QObject* parent)
    : QObject(parent),
    model_(model),
    smplModel_(smplModel),
    causal_avg_(causal_avg) {
    bodyKinematics_ = new lbk::BodyKinematics();
    // std::string npz_path = "C:/Users/ASUS/Desktop/newLogin/IMU_Interface/data/filtered_model.npz";
    // std::string fk_path = "C:/Users/ASUS/Desktop/newLogin/IMU_Interface/data/traced_smpl.pt";
    this->smplModel = smplModel;
    this->causal_avg = causal_avg;
    causal_avg = std::make_shared<CausalAvgImpl>(3);
    torch::Tensor avg_state;
}

ProcessingWorker::~ProcessingWorker() {
    QMutexLocker locker(&mutex_);
    running_ = true;
    avg_state_ = c10::nullopt;
    lastImuData_.clear();
}

void ProcessingWorker::setCalibrator(std::unique_ptr<CalibratorBase> cal) {
    calibrator_ = std::move(cal);
}
void ProcessingWorker::start() {
    QMutexLocker locker(&mutex_);
    running_ = true;
    avg_state_ = c10::nullopt;
    lastImuData_.clear();
}
void ProcessingWorker::stop() {
    QMutexLocker locker(&mutex_);
    running_ = false;
}
void printMatrix3f(const Eigen::Matrix3f& M, const QString& name)
{
    qDebug().noquote() << name;
    for (int i = 0; i < 3; ++i)
    {
        qDebug().noquote() << QString("%1 %2 %3")
        .arg(M(i,0), 0, 'f', 4)
            .arg(M(i,1), 0, 'f', 4)
            .arg(M(i,2), 0, 'f', 4);
    }
}

static void debugTensor(const torch::Tensor& t, const QString& name, int max_elems = 50) {
    auto sz = t.sizes();
    QString shape = "[";
    for (size_t i = 0; i < sz.size(); ++i) {
        shape += QString::number((qlonglong)sz[i]);
        if (i + 1 < sz.size()) shape += ", ";
    }
    shape += "]";

    torch::Tensor cpuTensor = t.contiguous().to(torch::kCPU);
    std::ostringstream oss;
    oss << name.toStdString() << " shape=" << shape.toStdString() << " values=[";
    int64_t numel = cpuTensor.numel();
    const float* data = cpuTensor.data_ptr<float>();
    for (int64_t i = 0; i < std::min<int64_t>(numel, max_elems); ++i) {
        oss << data[i];
        if (i + 1 < std::min<int64_t>(numel, max_elems)) oss << ", ";
    }
    if (numel > max_elems) oss << ", ...";
    oss << "]";

    qDebug().noquote() << QString::fromStdString(oss.str());
}

static void debugEigenMatrix(const Eigen::MatrixXf& mat, const QString& name) {
    std::ostringstream oss;
    oss << name.toStdString() << " shape=[" << mat.rows() << "," << mat.cols() << "] values=[";
    for (int i = 0; i < mat.rows(); ++i) {
        for (int j = 0; j < mat.cols(); ++j) {
            oss << mat(i,j);
            if (!(i == mat.rows()-1 && j == mat.cols()-1)) oss << ", ";
        }
    }
    oss << "]";
    qDebug().noquote() << QString::fromStdString(oss.str());
}

static void debugEigenRotList(const std::vector<Eigen::Matrix3f>& rots, const QString& name, int max_items = 6) {
    std::ostringstream oss;
    oss << name.toStdString() << " count=" << rots.size() << " [";
    for (size_t idx = 0; idx < std::min<size_t>(rots.size(), max_items); ++idx) {
        oss << "M" << idx << "={";
        for (int i=0; i<3; ++i) {
            for (int j=0; j<3; ++j) {
                oss << rots[idx](i,j);
                if (!(i==2 && j==2)) oss << ",";
            }
        }
        oss << "}";
        if (idx + 1 < std::min<size_t>(rots.size(), max_items)) oss << ", ";
    }
    if (rots.size() > size_t(max_items)) oss << ", ...";
    oss << "]";
    qDebug().noquote() << QString::fromStdString(oss.str());
}

void ProcessingWorker::processIMUData(const IMU &data) {
    QMutexLocker locker(&mutex_);
    if (!running_){
        qDebug() << "processIMUData: Returned early due to running_ = false";  // NEW: Debug early return
        return;
    }

    const auto slotStartUs = std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::steady_clock::now().time_since_epoch())
                                .count();
    const qint64 queueLatencyUs = data.hostEmitTimeUs > 0 ? (slotStartUs - data.hostEmitTimeUs) : -1;
    qDebug().noquote() << QString("processIMUData: imu=%1 queueLatencyUs=%2 buffered=%3")
                              .arg(data.id)
                              .arg(queueLatencyUs)
                              .arg(lastImuData_.size() + 1);

    lastImuData_[data.id] = data;

    if (lastImuData_.size() < 6) return;

    const auto batchStartUs = std::chrono::duration_cast<std::chrono::microseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count();


    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> acc(6, 3);
    std::vector<Eigen::Matrix3f> rot(6);
    int idx = 0;
    for (const auto &p : lastImuData_) {
        const IMU &imu = p.second;
        acc.row(idx) = Eigen::RowVector3f(imu.acc[0], imu.acc[1], imu.acc[2]);
        Eigen::Vector4f q(imu.quat[0], imu.quat[1], imu.quat[2], imu.quat[3]);
        if (p.first == 12 || p.first == 13)
        {
            rot[idx] = quat_to_rotmat_XYZ(q);
        }
        else{
            rot[idx] = quat_to_rotmat(q);
        }
        idx++;
    }
    lastImuData_.clear();

    // کالیبراسیون
    // Eigen::MatrixXf acc_cal;
    // std::vector<Eigen::Matrix3f> ori_cal;
    // if (!calibrator_) {
    //     qWarning() << "No calibrator set in ProcessingWorker!";
    //     return;
    // }
    // calibrator_->calibrate(acc, rot, acc_cal, ori_cal);

    auto options = torch::TensorOptions().dtype(torch::kFloat32);

    // acc_cal → tensor
    torch::Tensor calibAcc = torch::from_blob(
                                 const_cast<float*>(acc.data()),
                                 {acc.rows(), acc.cols()},
                                 options).clone();

    // ori_cal → tensor
    std::vector<torch::Tensor> rot_tensors;
    rot_tensors.reserve(rot.size());
    for (auto &M : rot) {
        rot_tensors.push_back(
            torch::from_blob(const_cast<float*>(M.data()), {3, 3}, options).clone().transpose(0, 1).contiguous()
            );
    }
    torch::Tensor calibRot = torch::stack(rot_tensors, 0); // [6,3,3]

    try {
        torch::NoGradGuard no_grad;

        // اجرای مدل
        auto out = model_->forward({calibAcc, calibRot}).toTuple();
        torch::Tensor pose = out->elements()[0].toTensor();   // [1,24,3,3]
        torch::Tensor trans;
        if (out->elements().size() > 1)
            trans = out->elements()[1].toTensor(); // [1,3]
        else
            trans = torch::zeros({1,3}, options);
        // pose[0][18] = pose[0][18].transpose(0,1).clone();
        // pose[0][19] = pose[0][19].transpose(0,1).clone();
        // ماتریس جابجایی X <-> Z
        /*
        torch::Tensor P = torch::zeros({3,3});
        P[0][2] = 1.0f;
        P[1][1] = 1.0f;
        P[2][0] = 1.0f;

        torch::Tensor M = pose[0][18];
        // ماتریس جدید که مؤلفه‌های ورودی را swap می‌کند
        torch::Tensor M_new = torch::matmul(M,P);
        pose[0][18].copy_(M_new);
        M = pose[0][19];
        // ماتریس جدید که مؤلفه‌های ورودی را swap می‌کند
        M_new = torch::matmul(M,P);
        pose[0][19].copy_(M_new);
        */
        // برای ثابت نگه داشتن دست ها در حالت تی پوز
        pose[0][13] = torch::eye(3, torch::TensorOptions().dtype(torch::kFloat32));
        pose[0][14] = torch::eye(3, torch::TensorOptions().dtype(torch::kFloat32));
        pose[0][16] = torch::eye(3, torch::TensorOptions().dtype(torch::kFloat32));
        pose[0][17] = torch::eye(3, torch::TensorOptions().dtype(torch::kFloat32));
        pose[0][18] = torch::eye(3, torch::TensorOptions().dtype(torch::kFloat32));
        pose[0][19] = torch::eye(3, torch::TensorOptions().dtype(torch::kFloat32));


        std::vector<torch::Tensor> p_pose_list = {pose};
        std::vector<torch::Tensor> p_trans_list = {trans};
        auto [verts, faces, joints] = smplModel_->view_motion(p_pose_list, p_trans_list);

        // بعد از محاسبه verts, faces, joints:
        //qDebug() << "Joints shape:" << joints.sizes();  // برای دیباگ shape (در console ببین)

        torch::Tensor joints_view = joints.squeeze(0);  // اگر [1, 24, 3] -> [24, 3]
        if (joints_view.dim() != 2 || joints_view.size(1) != 3) {
            qWarning() << "Unexpected joints shape, skipping jointsUpdated";
            return;  // جلوگیری از crash اگر shape غلط
        }

        auto joints_cpu = joints_view.contiguous().to(torch::kCPU);
        const float* joints_data = joints_cpu.data_ptr<float>();
        int numJoints = joints_cpu.size(0);  // حالا 24

        std::vector<QVector3D> currentJoints;
        for (int i = 0; i < numJoints; ++i) {
            if (i >= 6) break;  // محدود به 6 joint اول (یا بر اساس selected_imus اگر در worker داری)
            currentJoints.push_back(QVector3D(joints_data[i*3], joints_data[i*3+1], joints_data[i*3+2]));
        }

        if (currentJoints.size() < 6) {
            qWarning() << "Only" << currentJoints.size() << "joints available, padding with zero";
            while (currentJoints.size() < 6) {
                currentJoints.push_back(QVector3D(0,0,0));  // pad اگر کمتر
            }
        }

        emit jointsUpdated(currentJoints);

        c10::optional<torch::Tensor> opt_state = avg_state_;
        auto [pooled_verts, new_state] = causal_avg_->forward(verts.unsqueeze(0), opt_state);
        avg_state_ = new_state;

        verts = pooled_verts.contiguous().to(torch::kCPU);
        std::vector<float> vertices(verts.data_ptr<float>(),
                                    verts.data_ptr<float>() + verts.numel());

        faces = faces.contiguous().to(torch::kCPU);
        std::vector<int> face_indices;
        if (faces.scalar_type() == torch::kInt64) {
            std::vector<int64_t> tmp(faces.data_ptr<int64_t>(),
                                     faces.data_ptr<int64_t>() + faces.numel());
            face_indices.assign(tmp.begin(), tmp.end());
        } else {
            face_indices.assign(faces.data_ptr<int>(),
                                faces.data_ptr<int>() + faces.numel());
        }

        emit meshUpdated(vertices, face_indices);

        std::vector<std::array<double, 3>> allEulerAngles(24);
        std::array<double, 3> translation;
        try {
            for (int jointIndex = 0; jointIndex < 24; ++jointIndex) {
                torch::Tensor R = pose[0][jointIndex].to(torch::kDouble).contiguous();
                const double (*R_array)[3] = reinterpret_cast<const double (*)[3]>(R.data_ptr<double>());
                auto euler = rotm2eul_XYZ(R_array);
                allEulerAngles[jointIndex] = euler; // Store rx, ry, rz for this joint
            }
            trans = trans.to(torch::kDouble).contiguous();
            const double* trans_data = trans.data_ptr<double>();
            translation = {trans_data[0], trans_data[1], trans_data[2]};

            emit allEulerAnglesUpdated(allEulerAngles,translation); // Emit all angles
        }
        catch (const std::exception &e) {
            qWarning() << "ProcessingWorker exception:" << e.what();
        }
        catch (...) {
            qWarning() << "Error computing Euler angles for all joints";
        }

        try {
            int jointIndex = currentJointIndex;
            torch::Tensor R = pose[0][jointIndex].to(torch::kDouble);
            R = R.contiguous();
            const double (*R_array)[3] = reinterpret_cast<const double (*)[3]>(R.data_ptr<double>());
            auto euler = rotm2eul_XYZ(R_array);
            emit eulerDataUpdated(euler[0], euler[1], euler[2]);
        } catch (...) {
        }

    } catch (const std::exception &e) {
        qWarning() << "ProcessingWorker exception:" << e.what();
    } catch (...) {
        qWarning() << "ProcessingWorker unknown exception";
    }
}

#include <torch/torch.h>

// Compute projection of the Euclidean mean onto SO(3)
torch::Tensor projected_mean_so3(const torch::Tensor& R) {
    // Mean over the 3rd dimension from the end (dim = -3)
    // Python: M = R.mean(dim=-3)
    torch::Tensor M = R.mean(-3);

    // SVD: M = U * S * Vh
    torch::Tensor U, S, Vh;
    std::tie(U, S, Vh) = torch::linalg_svd(M, /*full_matrices=*/false);

    // First estimate: closest orthogonal matrix
    torch::Tensor Rhat = torch::matmul(U, Vh);

    // Compute det(Rhat)
    torch::Tensor det = torch::det(Rhat);

    // Build diagonal correction matrix (to enforce det = +1)
    // fix shape = (..., 3)
    auto fix = torch::ones_like(S);
    // fix the last element per rotation: -1 if det < 0 else +1
    fix.index_put_({torch::indexing::Slice(), 2}, torch::where(det < 0, -1.0, 1.0));

    // Convert fix into a diagonal matrix (..., 3, 3)
    torch::Tensor diag_fix = torch::diag_embed(fix);

    // Final rotation mean: U * diag_fix * Vh
    torch::Tensor R_mean = torch::matmul(U, torch::matmul(diag_fix, Vh));

    return R_mean;
}

struct Joints24Out {
    // (1,24,3,3) rotations
    std::array<Eigen::Matrix3f, 24> rotations;

    // (1,3) translation (pelvis root)
    Eigen::Vector3f pelvis_translation;
};
inline Joints24Out buildJoints24Out(const lbk::UpperFrameOutputs& out)
{
    Joints24Out result;

    // --- Root rotation & translation ---
    const Eigen::Matrix3f R_root = out.T_root_N.topLeftCorner<3,3>();
    const Eigen::Vector3f Pelvis = out.T_root_N.topRightCorner<3,1>();
    result.pelvis_translation = Pelvis;  // (1,3)
    result.rotations[0] = R_root;        // pelvis
    ///identityyy
    Eigen::Matrix3f R_up = Eigen::Matrix3f::Identity() ;

    // --- Upper body (needs the merged FrameOutputs from your class) ---
    // If you didn’t call calibrateUpper()/set upper frames, these are identity by default.

    const Eigen::Matrix3f R_back     = out.Ru_01_0;
    const Eigen::Matrix3f R_headBase = out.Ru_02_01;

    const Eigen::Matrix3f R_lShoulder = out.Ru_l1_01;
    const Eigen::Matrix3f R_lElbow    = out.Ru_l2_l1;
    const Eigen::Matrix3f R_lWrist    = out.Ru_l3_l2;
    const Eigen::Matrix3f R_lHand     = out.Ru_l4_l3;

    const Eigen::Matrix3f R_rShoulder = out.Ru_r1_01;
    const Eigen::Matrix3f R_rElbow    = out.Ru_r2_r1;
    const Eigen::Matrix3f R_rWrist    = out.Ru_r3_r2;
    const Eigen::Matrix3f R_rHand     = out.Ru_r4_r3;

    const Eigen::Matrix3f R_spine1 = R_back;
    const Eigen::Matrix3f R_spine2 = R_back;
    const Eigen::Matrix3f R_spine3 = R_back;

    // const Eigen::Matrix3f R_spine1 = R_up;
    // const Eigen::Matrix3f R_spine2 = R_up;
    // const Eigen::Matrix3f R_spine3 = R_up;
    // Neck and head
    const Eigen::Matrix3f R_neck = R_headBase;
    const Eigen::Matrix3f R_head = R_headBase;

    const Eigen::Matrix3f R_lClav = R_back;
    const Eigen::Matrix3f R_rClav = R_back;

    // Shoulders / elbows / wrists / hands with per-joint offsets
    const Eigen::Matrix3f R_lSh   = R_lShoulder;
    const Eigen::Matrix3f R_rSh   = R_rShoulder;
    const Eigen::Matrix3f R_lEl   = R_lElbow;
    const Eigen::Matrix3f R_rEl   = R_rElbow;
    // const Eigen::Matrix3f R_lWr   = R_lWrist;
    // const Eigen::Matrix3f R_rWr   = R_rWrist;
    const Eigen::Matrix3f R_lWr   = R_lWrist;
    const Eigen::Matrix3f R_rWr   = R_rWrist;
    const Eigen::Matrix3f R_lHd   = R_lHand;
    const Eigen::Matrix3f R_rHd   = R_rHand;

    // lower
    Eigen::Matrix3f R_l1 = out.Rl_l1_0;
    Eigen::Matrix3f R_l2 = out.Rl_l2_l1;
    Eigen::Matrix3f R_l3 = out.Rl_l3_l2;
    Eigen::Matrix3f R_l4 = out.Rl_l4_l3;

    Eigen::Matrix3f R_r1 = out.Rl_r1_0;
    Eigen::Matrix3f R_r2 = out.Rl_r2_r1;
    Eigen::Matrix3f R_r3 = out.Rl_r3_r2;
    Eigen::Matrix3f R_r4 = out.Rl_r4_r3;

    //     // Upper body uses pelvis or identity
    //     Eigen::Matrix3f R_up = Eigen::Matrix3f::Identity();

    //     // === Fill rotations in SMPL-like order ===
    //     result.rotations[ 0] = R_root; // pelvis
    result.rotations[ 1] = R_l1;   result.rotations[ 2] = R_r1;
    result.rotations[ 4] = R_l2;   result.rotations[ 5] = R_r2;
    result.rotations[ 7] = R_l3;   result.rotations[ 8] = R_r3;
    result.rotations[10] = R_l4;   result.rotations[11] = R_r4;
    // result.rotations[10] = R_up;   result.rotations[11] = R_up;
    //     // Upper body (fixed hands, use R_up)
    //     result.rotations[12] = R_up;  // neck
    //     result.rotations[13] = R_up;  // L clavicle
    //     result.rotations[14] = R_up;  // R clavicle
    //     result.rotations[15] = R_up;  // head




    //////////////////////////////////////////////////////
    // Now place them into the SMPL-24 order you already use:
    //  0 pelvis
    //  1 L_hip,   2 R_hip
    //  3 spine1,  4 L_knee, 5 R_knee
    //  6 spine2,  7 L_ankle,8 R_ankle
    //  9 spine3, 10 L_foot, 11 R_foot
    // 12 neck,   13 L_clav, 14 R_clav
    // 15 head,   16 L_sh,   17 R_sh
    // 18 L_el,   19 R_el,   20 L_wr, 21 R_wr
    // 22 L_hand, 23 R_hand
    result.rotations[3]  = R_spine1;
    result.rotations[6]  = R_spine2;
    result.rotations[9]  = R_spine3;

    result.rotations[12] = R_neck;
    result.rotations[13] = R_back;
    result.rotations[14] = R_back;

    result.rotations[15] = R_head;
    result.rotations[16] = R_lSh;
    result.rotations[17] = R_rSh;

    result.rotations[18] = R_lEl;
    result.rotations[19] = R_rEl;
    result.rotations[20] = R_lWr;
    result.rotations[21] = R_rWr;
    // result.rotations[22] = R_lEl;
    // result.rotations[23] = R_rEl;

    result.rotations[22] = R_lHd;
    result.rotations[23] = R_rHd;
    // result.rotations[22] = R_up;
    // result.rotations[23] = R_up;

    return result;
}


void ProcessingWorker::processIMUDataKin(const IMU &data) {
    QMutexLocker locker(&mutex_);
    if (!running_) {
        qDebug() << "processIMUDataKin: returned early, running_ = false";
        return;
    }
    const auto slotStartUs = std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now().time_since_epoch())
                                 .count();
    const qint64 queueLatencyUs = data.hostEmitTimeUs > 0 ? (slotStartUs - data.hostEmitTimeUs) : -1;
    qDebug().noquote() << QString("processIMUData: imu=%1 queueLatencyUs=%2 buffered=%3")
                              .arg(data.id)
                              .arg(queueLatencyUs)
                              .arg(lastImuData_.size() + 1);

    // Store the latest IMU data
    lastImuData_[data.id] = data;
    const auto batchStartUs = std::chrono::duration_cast<std::chrono::microseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count();



    // qDebug() << "Quat:"
    //          << data.quat[0]
    //          << data.quat[1]
    //          << data.quat[2]
    //          << data.quat[3];
    // Wait until we have enough IMUs
    if (lastImuData_.size() < imuCount) return;
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    std::map<std::string, Eigen::Matrix3f> imuR;
    imuR["pelvis"]    = I;
    imuR["back"]      = I;
    imuR["head"]      = I;
    imuR["arml"]      = I;
    imuR["armr"]      = I;
    imuR["forearml"]  = I;
    imuR["forearmr"]  = I;
    imuR["handl"]     = I;
    imuR["handr"]     = I;
    imuR["thighl"]    = I;
    imuR["thighr"]    = I;
    imuR["shankl"]    = I;
    imuR["shankr"]    = I;
    imuR["footl"]     = I;
    imuR["footr"]     = I;

    // qDebug() << "computing frame";
    // ----------------- Convert IMU quaternions to rotation matrices -----------------

    for (const auto& [name, id] : selected_imusKin) {
        if (lastImuData_.find(id) == lastImuData_.end()) continue;
        const auto& imu = lastImuData_.at(id);
        Eigen::Vector4f q(imu.quat[0], imu.quat[1], imu.quat[2], imu.quat[3]);

        imuR[name] = quatToRot(q(0), q(1), q(2), q(3));
        // qDebug() << "imu rot :";
        // qDebug() << imuR[name];
        // printMatrix3f(imuR[name], "Rotation Matrix R:");
    }
    lastImuData_.clear(); // clear buffer for next frame
    Eigen::Matrix3f R_up = Eigen::Matrix3f::Identity();
    // ----------------- Compute kinematics -----------------
    lbk::UpperFrameOutputs out = bodyKinematics_->computeFramee(
        imuR["pelvis"],    // pelvis
        imuR["back"],      // back
        // imuR["pelvis"],

        // R_up,              // head / reference

        imuR["head"],
        // imuR["pelvis"],

        imuR["arml"],      // left upper arm
        imuR["armr"],      // right upper arm
        imuR["forearml"],  // left forearm
        imuR["forearmr"],  // right forearm
        // imuR["forearml"],  // left forearm
        // imuR["forearmr"],  // right forearm
        imuR["handl"],     // left hand
        imuR["handr"],     // right hand
        imuR["thighl"],    // left thigh
        imuR["thighr"],    // right thigh
        imuR["shankl"],    // left shank
        imuR["shankr"],     // right shank
        // &imuR["shankl"],    // left shank
        // &imuR["shankr"]     // right shank
        &imuR["footl"],
        &imuR["footr"]
        );

    // Build 24-joint rotations
    Joints24Out j24 = buildJoints24Out(out);
    std::vector<float> rot_data;
    rot_data.reserve(24 * 3 * 3);
    for (int j = 0; j < 24; ++j) {
        const Eigen::Matrix3f& R = j24.rotations[j]; // درست: double
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                rot_data.push_back(static_cast<float>(R(r,c))); // cast فقط اینجا
    }

    torch::Tensor pose_t = torch::from_blob(
                               rot_data.data(),
                               {1, 24, 3, 3},
                               torch::TensorOptions().dtype(torch::kFloat32).device(torch::kCPU)
                               ).clone();

    torch::Tensor trans_t = torch::zeros({1,3}, torch::kFloat32);
    const Eigen::Vector3f ta = j24.pelvis_translation;
    trans_t[0][0] = static_cast<float>(ta.x());
    trans_t[0][1] = static_cast<float>(ta.y());
    trans_t[0][2] = static_cast<float>(ta.z());

    std::vector<torch::Tensor> p_pose_list;
    std::vector<torch::Tensor> p_trans_list;
    p_pose_list.push_back(pose_t);
    p_trans_list.push_back(trans_t);

    auto [verts, faces, joints] = smplModel->view_motion(p_pose_list, p_trans_list);
    auto [pooled_verts, new_state] = causal_avg->forward(verts.unsqueeze(0), avg_state);
    verts = pooled_verts;
    avg_state = new_state;
    verts = verts.contiguous().to(torch::kCPU);
    std::vector<float> vertices(verts.data_ptr<float>(), verts.data_ptr<float>() + verts.numel());

    faces = faces.contiguous().to(torch::kCPU);
    std::vector<int> face_indices;
    if (faces.scalar_type() == torch::kInt64) {
        const auto* p = faces.data_ptr<int64_t>();
        face_indices.assign(p, p + faces.numel());
    } else {
        const auto* p = faces.data_ptr<int>();
        face_indices.assign(p, p + faces.numel());
    }

    try {
        // محاسبه زوایای اویلر برای همه ۲۴ مفصل
        std::vector<std::array<double, 3>> allEulerAngles(24);
        std::array<double, 3> translation;

        for (int j = 0; j < 24; ++j) {
            torch::Tensor R = pose_t[0][j].to(torch::kDouble).contiguous();
            const double (*R_array)[3] = reinterpret_cast<const double (*)[3]>(R.data_ptr<double>());
            auto euler = rotm2eul_XYZ(R_array);
            allEulerAngles[j] = euler;
        }

        // دریافت ترجمه (translation)
        torch::Tensor trans_double = trans_t.to(torch::kDouble).contiguous();
        const double* trans_data = trans_double.data_ptr<double>();
        translation = {trans_data[0], trans_data[1], trans_data[2]};

        // ارسال سیگنال با همه زوایا
        emit allEulerAnglesUpdated(allEulerAngles, translation);

    } catch (const std::exception& e) {
        qWarning() << "Error computing Euler angles for all joints:" << e.what();
    } catch (...) {
        qWarning() << "Unknown error computing Euler angles";
    }

    emit meshUpdated(vertices, face_indices);

    try {
        int jointIndex = currentJointIndex;
        torch::Tensor R = pose_t[0][jointIndex].to(torch::kDouble);
        R = R.contiguous();
        const double (*R_array)[3] = reinterpret_cast<const double (*)[3]>(R.data_ptr<double>());
        // Define your joint sets
        const std::set<int> YZXJoints = {1, 20, 21};  // example indices
        const std::set<int> XYZJoints = {4, 5, 6};
        const std::set<int> ZYXJoints = {16, 17};
        const std::set<int> XZYJoints = {1, 2, 4, 5, 7, 8, 15};
        const std::set<int> YXZJoints = {18, 19};

        // Using map for cleaner code
        std::map<std::set<int>, std::function<std::array<double,3>(const double[3][3])>> jointMap = {
            {YZXJoints, rotm2eul_YZX},
            {XYZJoints, rotm2eul_XYZ},
            {ZYXJoints, rotm2eul_ZYX},
            {XZYJoints, rotm2eul_XZY},
            {YXZJoints, rotm2eul_YXZ},
        };

        // Usage
        auto euler = std::array<double, 3>{};
        // Using find() instead of contains()
        for (const auto& [jointSet, func] : jointMap) {
            if (jointSet.find(jointIndex) != jointSet.end()) {
                euler = func(R_array);
                break;
            }
        }
        // auto euler = rotm2eul_XYZ(R_array);
        emit eulerDataUpdated(euler[0], euler[1], euler[2]);
    } catch (...) {
    }
}


void ProcessingWorker::setJointIndex(int index) {
    QMutexLocker locker(&mutex_);
    currentJointIndex = index;
}

// در جایی که مش برای اولین بار دریافت می‌شود، originalVerts_ را ذخیره کنید
void ProcessingWorker::storeOriginalMesh(const std::vector<float>& verts,
                                         const std::vector<int>& faces) {
    QMutexLocker locker(&mutex_);
    originalVerts_ = verts;
    faces_ = faces;
    hasOriginalMesh_ = true;
}

// تابع تغییر شکل موجی سطح
void ProcessingWorker::deformGround(float time, float amplitude) {
    if (!hasOriginalMesh_) return;

    QMutexLocker locker(&mutex_);
    std::vector<float> deformedVerts = originalVerts_;  // شروع از حالت اولیه

    // اعمال تغییرات روی رأس‌ها
    for (size_t i = 0; i < deformedVerts.size() / 3; ++i) {
        float x = deformedVerts[3*i];
        float z = deformedVerts[3*i+2];

        // حرکت شبیه موج سینوسی
        float y_offset = sin(x * 0.5f + time) * cos(z * 0.5f) * amplitude;
        deformedVerts[3*i+1] = originalVerts_[3*i+1] + y_offset;
    }

    // ارسال مش تغییر یافته به GPU
    emit meshUpdated(deformedVerts, faces_);
}

// تابع جابه‌جایی کل زمین
void ProcessingWorker::translateGround(float dx, float dy, float dz) {
    if (!hasOriginalMesh_) return;

    QMutexLocker locker(&mutex_);
    std::vector<float> translatedVerts = originalVerts_;

    for (size_t i = 0; i < translatedVerts.size() / 3; ++i) {
        translatedVerts[3*i]   += dx;
        translatedVerts[3*i+1] += dy;
        translatedVerts[3*i+2] += dz;
    }

    emit meshUpdated(translatedVerts, faces_);
}
