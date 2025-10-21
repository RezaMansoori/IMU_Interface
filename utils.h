#ifndef UTILS_H
#define UTILS_H

#undef slots
#undef signals
#undef emit
#include <torch/torch.h>
#include <torch/script.h>
#define slots Q_SLOTS
#define signals Q_SIGNALS
#define emit Q_EMIT

#include <fstream>
#include <Eigen/Dense>
#include <QVector3D>
#include "cnpy.h"

Eigen::MatrixXf loadMatrix(const std::string& path);
std::vector<QVector3D> computeNormals(const std::vector<float>& vertices,
                                      const std::vector<int>& faces);
std::array<double, 3> rotm2eul_XYZ(const double R[3][3]);
torch::Tensor npzToTensor(const cnpy::NpyArray& arr, torch::ScalarType dtype);
Eigen::MatrixXd loadEulerCsv(const std::string& path);
torch::Tensor eulerToRotMatrix(double roll, double pitch, double yaw);
Eigen::Vector3f eulerFromRotMatrix(const Eigen::Matrix3f& R);
Eigen::Vector4f rotmatToQuat(const Eigen::Matrix3f& R);
bool containsId(const std::map<std::string, int>& m, int id);
#endif // UTILS_H
