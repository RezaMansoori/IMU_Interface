#ifndef CALIBRATIONPAGE_H
#define CALIBRATIONPAGE_H

#include <QWidget>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <QMessageBox>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QHBoxLayout>
#include <QFrame>
#include "imu.h"
#include "usbreceiver.h"
#include <Eigen/Dense>

class CalibrationGLWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public:
    explicit CalibrationGLWidget(QWidget *parent = nullptr, bool isDarkTheme = true)
        : QOpenGLWidget(parent), rotationX(30.0f), rotationY(45.0f), zoom(-0.5f), lastPos(0, 0), isDarkTheme(isDarkTheme) {}
    void setPoints(const std::vector<Eigen::Vector3d> &newPoints) {
        points = newPoints;
        update();
    }

protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    std::vector<Eigen::Vector3d> points;
    float rotationX;
    float rotationY;
    float zoom;
    QPoint lastPos;
    bool isDarkTheme;
};

class CalibrationPage : public QWidget {
    Q_OBJECT
public:
    explicit CalibrationPage(QWidget *parent = nullptr, int numImus = 18, bool isDarkTheme = true, USBReceiver *usbReceiver = nullptr, QComboBox *externalCombo = nullptr);
    void applyTheme();

public slots:
    void selectIMU();
    void selectIMUFromCombo(int index);
    void startAccelGyroCalibration();
    void startMagCalibration();
    void saveCalibration();
    void handleIMUData(const IMU &data);
    void handleUSBConnection(bool connected);

signals:
    void recordingStarted(const QString &fileName);
    void recordingStopped(const QString &fileName);
    void calibrationCompleted(int imuId); // <--- سیگنال جدید

private:
    void performEllipsoidFit();
    void initToolbar();

    int numImus;
    int selectedIMU;
    bool isAccelGyroCalibrating;
    bool isMagCalibrating;
    Eigen::Vector3d lastMagData;
    Eigen::Vector3d accelBias;
    Eigen::Vector3d gyroBias;
    Eigen::Vector3d magCenter;
    Eigen::Vector3d magRadii;
    std::vector<Eigen::Vector3d> accSamples;
    std::vector<Eigen::Vector3d> gyroSamples;
    std::vector<Eigen::Vector3d> magDataPoints;
    QComboBox *imuSelector;
    QPushButton *btnSelectIMU;
    QPushButton *btnAccelGyro;
    QPushButton *btnMagCalib;
    QPushButton *btnSave;
    QLabel *statusLabel;
    QLabel *pointsLabel;
    CalibrationGLWidget *glWidget;
    QHBoxLayout *toolbar;
    QLabel *guideLabel; // لیبل جدید برای راهنمای کالیبراسیون
    bool isDarkTheme;
    bool isUSBConnected;
    USBReceiver *usbReceiver;
    QLabel *quatWLabel;
    QLabel *northDeviationLabel;
};

#endif // CALIBRATIONPAGE_H
