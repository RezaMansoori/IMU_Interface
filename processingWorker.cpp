#include "ProcessingWorker.h"
#include <QDebug>

ProcessingWorker::ProcessingWorker(std::shared_ptr<torch::jit::script::Module> model,
                                   SMPLModel* smplModel,
                                   std::shared_ptr<CausalAvgImpl> causal_avg,
                                   QObject* parent)
    : QObject(parent),
    model_(model),
    smplModel_(smplModel),
    causal_avg_(causal_avg) {}

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

    lastImuData_[data.id] = data;

    if (lastImuData_.size() < 6) return;

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
    Eigen::MatrixXf acc_cal;
    std::vector<Eigen::Matrix3f> ori_cal;
    if (!calibrator_) {
        qWarning() << "No calibrator set in ProcessingWorker!";
        return;
    }
    calibrator_->calibrate(acc, rot, acc_cal, ori_cal);

    auto options = torch::TensorOptions().dtype(torch::kFloat32);

    // acc_cal → tensor
    torch::Tensor calibAcc = torch::from_blob(
                                 const_cast<float*>(acc_cal.data()),
                                 {acc_cal.rows(), acc_cal.cols()},
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
        // pose[0][13] = torch::eye(3, torch::TensorOptions().dtype(torch::kFloat32));
        // pose[0][14] = torch::eye(3, torch::TensorOptions().dtype(torch::kFloat32));
        // pose[0][16] = torch::eye(3, torch::TensorOptions().dtype(torch::kFloat32));
        // pose[0][17] = torch::eye(3, torch::TensorOptions().dtype(torch::kFloat32));
        // pose[0][18] = torch::eye(3, torch::TensorOptions().dtype(torch::kFloat32));
        // pose[0][19] = torch::eye(3, torch::TensorOptions().dtype(torch::kFloat32));


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

void ProcessingWorker::setJointIndex(int index) {
    QMutexLocker locker(&mutex_);
    currentJointIndex = index;
}
