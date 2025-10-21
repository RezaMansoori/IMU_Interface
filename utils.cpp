#include "utils.h"


Eigen::MatrixXf loadMatrix(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("Failed to open file: " + path);

    int rows = 0, cols = 0;
    in.read(reinterpret_cast<char*>(&rows), sizeof(int));
    in.read(reinterpret_cast<char*>(&cols), sizeof(int));

    Eigen::MatrixXf mat(rows, cols);
    in.read(reinterpret_cast<char*>(mat.data()), rows * cols * sizeof(float));
    return mat;
}

std::vector<QVector3D> computeNormals(const std::vector<float>& vertices,
                                      const std::vector<int>& faces) {
    size_t numVerts = vertices.size() / 3;
    std::vector<QVector3D> normals(numVerts, QVector3D(0,0,0));

    for (size_t i = 0; i < faces.size(); i += 3) {
        int i0 = faces[i];
        int i1 = faces[i+1];
        int i2 = faces[i+2];

        QVector3D v0(vertices[3*i0], vertices[3*i0+1], vertices[3*i0+2]);
        QVector3D v1(vertices[3*i1], vertices[3*i1+1], vertices[3*i1+2]);
        QVector3D v2(vertices[3*i2], vertices[3*i2+1], vertices[3*i2+2]);

        QVector3D normal = QVector3D::normal(v1 - v0, v2 - v0);

        normals[i0] += normal;
        normals[i1] += normal;
        normals[i2] += normal;
    }

    // normalize all normals
    for (auto& n : normals) {
        n.normalize();
    }

    return normals;
}

std::array<double, 3> rotm2eul_XYZ(const double R[3][3]) {
    double x, y, z;

    if (std::abs(R[0][2]) < 1.0 - 1e-12) {
        y = std::asin(R[0][2]);
        x = std::atan2(-R[1][2], R[2][2]);
        z = std::atan2(-R[0][1], R[0][0]);
    } else {
        y = (R[0][2] > 0 ? M_PI/2 : -M_PI/2);
        x = std::atan2(R[1][0], R[1][1]);
        z = 0;
    }

    return { x * 180.0 / M_PI,
            y * 180.0 / M_PI,
            z * 180.0 / M_PI };
}

torch::Tensor npzToTensor(const cnpy::NpyArray& arr, torch::ScalarType dtype) {
    std::vector<int64_t> sizes(arr.shape.begin(), arr.shape.end());
    auto options = torch::TensorOptions().dtype(dtype).device(torch::kCPU);
    torch::Tensor t;
    std::cout << "Word size: " << arr.word_size << std::endl;
    return torch::from_blob(const_cast<float*>(arr.data<float>()), sizes, torch::kFloat32).clone();
}

Eigen::MatrixXd loadEulerCsv(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open CSV: " + path);
    }
    std::vector<std::vector<double>> data;
    std::string line;
    std::getline(file, line);  // Skip header
    while (std::getline(file, line)) {
        std::vector<double> row;
        std::stringstream ss(line);
        std::string token;
        while (std::getline(ss, token, ',')) {
            row.push_back(std::stod(token));
        }
        if (row.size() == 221) {
            row.erase(row.begin());
        }
        if (row.size() != 220) {
            throw std::runtime_error("Invalid row size in CSV (expected 220 angles after removing index)");
        }
        data.push_back(row);
    }
    Eigen::MatrixXd mat(data.size(), 220);
    for (size_t i = 0; i < data.size(); ++i) {
        mat.row(i) = Eigen::Map<Eigen::RowVectorXd>(data[i].data(), 220);
    }
    return mat;
}

torch::Tensor eulerToRotMatrix(double roll, double pitch, double yaw) {
    const double to_radians = M_PI / 180.0;
    roll *= to_radians;
    pitch *= to_radians;
    yaw *= to_radians;

    double cr = cos(roll), sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw), sy = sin(yaw);
    return torch::tensor({
        {cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy},
        {cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy},
        {-sp,     sr * cp,                cr * cp}
    });
}

Eigen::Vector3f eulerFromRotMatrix(const Eigen::Matrix3f& R) {
    float sy = std::sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
    bool singular = sy < 1e-6;
    float roll, pitch, yaw;
    if (!singular) {
        roll = std::atan2(R(2,1), R(2,2));
        pitch = std::atan2(-R(2,0), sy);
        yaw = std::atan2(R(1,0), R(0,0));
    } else {
        roll = std::atan2(-R(1,2), R(1,1));
        pitch = std::atan2(-R(2,0), sy);
        yaw = 0;
    }
    return {roll, pitch, yaw}; // roll (x), pitch (y), yaw (z)
}

Eigen::Vector4f rotmatToQuat(const Eigen::Matrix3f& R) {
    Eigen::Quaternionf q(R);
    q.normalize();
    return {q.x(), q.y(), q.z(), q.w()};
}

bool containsId(const std::map<std::string, int>& m, int id) {
    for (const auto& [name, imuId] : m) {
        if (imuId == id) {
            return true;
        }
    }
    return false;
}




