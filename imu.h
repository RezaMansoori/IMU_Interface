#ifndef IMU_H
#define IMU_H

#include <QtGui/QQuaternion>
#include <QtMath>

class QTextStream;
class QDataStream;

class IMU {
public:
    IMU();

    void toArray(float* out) const;
    static QString csvHeader() noexcept;

    friend QDataStream& operator<<(QDataStream& ds, const IMU& imu);
    friend QDataStream& operator>>(QDataStream& ds, IMU& imu);
    friend QTextStream& operator<<(QTextStream& ts, const IMU& imu);

    inline QVector3D eulars() const {
        return QQuaternion(quat[0], quat[1], quat[2], quat[3]).toEulerAngles();
    }

    qint8 id;
    float acc[3];
    float gyro[3];
    float quat[4];
    float lacc[3];
    qint64 time;
    qint64 hostEmitTimeUs;
    qint8 battery;
};

#endif // IMU_H
