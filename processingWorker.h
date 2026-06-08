#pragma once
// #include <torch/script.h>

#include "smpl_model.h"
#include "utils.h"
#include "modelCalibrator.h"
#include "imu.h"
#include "calibratorBase.h"
#include "fullKinematics.hpp"
#include <QObject>
#include <QMutex>
#include <map>
#include <vector>
#include <memory>
// namespace torch {
// // class Tensor;
//     namespace jit {
//         class Module;
//     }
// }

class ProcessingWorker : public QObject {
    Q_OBJECT

public:
    void storeOriginalMesh(const std::vector<float>& verts,
                           const std::vector<int>& faces);

    SMPLModel* smplModel = nullptr;
    std::shared_ptr<CausalAvgImpl> causal_avg;
    c10::optional<torch::Tensor> avg_state;
    std::map<std::string, int> selected_imusKin;
    void calibrateKin(
        const std::map<std::string, int>& selected_imusKin,
        /*const Eigen::MatrixXf pose_data*/
        std::vector<Eigen::Vector4f> avg_quat);

    void setBodyKinematics(lbk::BodyKinematics* kin) { bodyKinematics_ = kin; }
    void setUseKinematics(bool enable) { useKinematics_ = enable; }
    ProcessingWorker(std::shared_ptr<torch::jit::script::Module> model,
                     SMPLModel* smplModel,
                     std::shared_ptr<CausalAvgImpl> causal_avg,
                     QObject* parent = nullptr);
    ~ProcessingWorker();
    bool use_kinematics_ = false;
    int imuCount;
    void setCalibrator(std::unique_ptr<CalibratorBase> cal);
    // void setUse_kinematics(bool enable);
public slots:
    void processIMUData(const IMU &data);
    void processIMUDataKin(const IMU &data);
    void stop();
    void setJointIndex(int index);
    void start();

    // توابع حرکت دادن سطح زمین
    void translateGround(float dx, float dy, float dz);
    void deformGround(float time, float amplitude);
signals:
    void meshUpdated(const std::vector<float>& verts,
                     const std::vector<int>& faces);
    void updateChart(torch::Tensor pose_t);
    void eulerDataUpdated(double rx, double ry, double rz);
    void allEulerAnglesUpdated(const std::vector<std::array<double, 3>>& eulerAngles,
                               const std::array<double, 3>& translation);
    void jointsUpdated(const std::vector<QVector3D>& joints);

private:
    // متغیرهای جدید برای نگهداری داده‌های اصلی مش
    std::vector<float> originalVerts_;  // ذخیره حالت اولیه
    std::vector<int> faces_;            // ذخیره وجه‌ها
    bool hasOriginalMesh_ = false;


    bool useKinematics_ = false;
    lbk::BodyKinematics* bodyKinematics_ = nullptr;
    std::shared_ptr<torch::jit::script::Module> model_;
    SMPLModel* smplModel_;
    std::shared_ptr<CausalAvgImpl> causal_avg_;
    std::unique_ptr<CalibratorBase> calibrator_;

    std::map<int, IMU> lastImuData_;
    bool running_ = true;
    QMutex mutex_;
    c10::optional<torch::Tensor> avg_state_;
    int currentJointIndex = 7;

};
