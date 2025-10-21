#pragma once
// #include <torch/script.h>

#include "smpl_model.h"
#include "utils.h"
#include "modelCalibrator.h"
#include "imu.h"
#include "calibratorBase.h"

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
    ProcessingWorker(std::shared_ptr<torch::jit::script::Module> model,
                     SMPLModel* smplModel,
                     std::shared_ptr<CausalAvgImpl> causal_avg,
                     QObject* parent = nullptr);
    ~ProcessingWorker();

    void setCalibrator(std::unique_ptr<CalibratorBase> cal);

public slots:
    void processIMUData(const IMU &data);
    void stop();
    void setJointIndex(int index);
    void start();
signals:
    void meshUpdated(const std::vector<float>& verts,
                     const std::vector<int>& faces);
    void eulerDataUpdated(double rx, double ry, double rz);
    void allEulerAnglesUpdated(const std::vector<std::array<double, 3>>& eulerAngles,
                               const std::array<double, 3>& translation);
    void jointsUpdated(const std::vector<QVector3D>& joints);

private:
    std::shared_ptr<torch::jit::script::Module> model_;
    SMPLModel* smplModel_;
    std::shared_ptr<CausalAvgImpl> causal_avg_;
    std::unique_ptr<CalibratorBase> calibrator_;

    std::map<int, IMU> lastImuData_;
    bool running_ = true;
    QMutex mutex_;
    c10::optional<torch::Tensor> avg_state_;
    int currentJointIndex = 0;
};
