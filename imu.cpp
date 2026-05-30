// #include "imu.h"
// #include <QtCore/QDataStream>
// #include <QtCore/QTextStream>
// #include <QtGui/QQuaternion>

// IMU::IMU() { memset(this, 0, sizeof(IMU)); }

// void IMU::toArray(float* out) const {
//     memcpy(out, &acc, 3 * sizeof(float));
//     memcpy(out + 3, &gyro, 3 * sizeof(float));
//     memcpy(out + 6, &quat, 4 * sizeof(float));
//     memcpy(out + 10, &lacc, 3 * sizeof(float));
// }

// QString IMU::csvHeader() noexcept {
//     return "ID,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,QuatW,QuatX,QuatY,QuatZ,LAccX,LAccY,LAccZ,Time,Battery";
// }

// QDataStream& operator<<(QDataStream& ds, const IMU& imu) {
//     ds << imu.id;
//     for (int i = 0; i < 3; i++) ds << imu.acc[i];
//     for (int i = 0; i < 3; i++) ds << imu.gyro[i];
//     for (int i = 0; i < 4; i++) ds << imu.quat[i];
//     for (int i = 0; i < 3; i++) ds << imu.lacc[i];
//     ds << imu.time << imu.battery;
//     return ds;
// }

// QDataStream& operator>>(QDataStream& ds, IMU& imu) {
//     ds >> imu.id;
//     for (int i = 0; i < 3; i++) ds >> imu.acc[i];
//     for (int i = 0; i < 3; i++) ds >> imu.gyro[i];
//     for (int i = 0; i < 4; i++) ds >> imu.quat[i];
//     for (int i = 0; i < 3; i++) ds >> imu.lacc[i];
//     ds >> imu.time >> imu.battery;
//     return ds;
// }

// QTextStream& operator<<(QTextStream& ts, const IMU& imu) {
//     ts << imu.id << ',';
//     for (int i = 0; i < 3; i++) ts << imu.acc[i] << ',';
//     for (int i = 0; i < 3; i++) ts << imu.gyro[i] << ',';
//     ts.setRealNumberPrecision(5);
//     for (int i = 0; i < 4; i++) ts << imu.quat[i] << ',';
//     ts.setRealNumberPrecision(4);
//     for (int i = 0; i < 3; i++) ts << imu.lacc[i] << ',';
//     ts << imu.time << ',' << imu.battery;
//     return ts;
// }

#include "imu.h"
#include <QtCore/QDataStream>
#include <QtCore/QTextStream>
#include <QtGui/QQuaternion>

IMU::IMU() { memset(this, 0, sizeof(IMU)); }

void IMU::toArray(float* out) const {
    memcpy(out, &acc, 3 * sizeof(float));
    memcpy(out + 3, &gyro, 3 * sizeof(float));
    memcpy(out + 6, &quat, 4 * sizeof(float));
    memcpy(out + 10, &lacc, 3 * sizeof(float));
}

QString IMU::csvHeader() noexcept {
    return "ID,AccX,AccY,AccZ,GyroX,GyroY,GyroZ,QuatW,QuatX,QuatY,QuatZ,LAccX,LAccY,LAccZ,Time,Battery";
}

QDataStream& operator<<(QDataStream& ds, const IMU& imu) {
    ds << imu.id;
    for (int i = 0; i < 3; i++) ds << imu.acc[i];
    for (int i = 0; i < 3; i++) ds << imu.gyro[i];
    for (int i = 0; i < 4; i++) ds << imu.quat[i];
    for (int i = 0; i < 3; i++) ds << imu.lacc[i];
    ds << imu.time << imu.battery;
    return ds;
}

QDataStream& operator>>(QDataStream& ds, IMU& imu) {
    ds >> imu.id;
    for (int i = 0; i < 3; i++) ds >> imu.acc[i];
    for (int i = 0; i < 3; i++) ds >> imu.gyro[i];
    for (int i = 0; i < 4; i++) ds >> imu.quat[i];
    for (int i = 0; i < 3; i++) ds >> imu.lacc[i];
    ds >> imu.time >> imu.battery;
    return ds;
}

QTextStream& operator<<(QTextStream& ts, const IMU& imu) {
    ts << imu.id << ',';
    for (int i = 0; i < 3; i++) ts << imu.acc[i] << ',';
    for (int i = 0; i < 3; i++) ts << imu.gyro[i] << ',';
    ts.setRealNumberPrecision(5);
    for (int i = 0; i < 4; i++) ts << imu.quat[i] << ',';
    ts.setRealNumberPrecision(4);
    for (int i = 0; i < 3; i++) ts << imu.lacc[i] << ',';
    ts << imu.time << ',' << imu.battery;
    return ts;
}
