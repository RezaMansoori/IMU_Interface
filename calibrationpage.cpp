#include "calibrationpage.h"
#include <QVBoxLayout>
#include <QOpenGLFunctions>
#include <QMatrix4x4>
#include <QVector3D>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QMessageBox>
#include <cmath>

void CalibrationGLWidget::initializeGL() {
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    glClearColor(isDarkTheme ? 0.2f : 1.0f, isDarkTheme ? 0.2f : 1.0f, isDarkTheme ? 0.2f : 1.0f, 1.0f); // Dark gray for dark theme, white for light theme
}

void CalibrationGLWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    QMatrix4x4 projection;
    projection.perspective(45.0f, static_cast<float>(w) / h, 0.1f, 100.0f);
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(projection.constData());
}

void CalibrationGLWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    QMatrix4x4 modelView;
    modelView.translate(0, 0, zoom); // اعمال زوم
    modelView.rotate(rotationX, 1, 0, 0); // چرخش حول محور X
    modelView.rotate(rotationY, 0, 1, 0); // چرخش حول محور Y
    modelView.translate(0, 1.0f, 0); // جابه‌جایی عمودی به بالا (1.0 واحد)
    glLoadMatrixf(modelView.constData());

    glPointSize(5.0f);
    glBegin(GL_POINTS);
    glColor3f(isDarkTheme ? 1.0f : 0.0f, isDarkTheme ? 1.0f : 0.0f, isDarkTheme ? 1.0f : 0.0f); // White points for dark theme, black for light theme
    for (const auto &point : points) {
        glVertex3d(point[0], point[1], point[2]);
    }
    glEnd();
}

void CalibrationGLWidget::mousePressEvent(QMouseEvent *event) {
    lastPos = event->pos(); // ذخیره موقعیت کلیک ماوس
}

void CalibrationGLWidget::mouseMoveEvent(QMouseEvent *event) {
    if (event->buttons() & Qt::LeftButton) {
        int dx = event->position().x() - lastPos.x();
        int dy = event->position().y() - lastPos.y();

        rotationX += dy * 0.5f; // به‌روزرسانی چرخش
        rotationY += dx * 0.5f;

        lastPos = event->pos();
        update(); // بازسازی صحنه
    }
}

void CalibrationGLWidget::wheelEvent(QWheelEvent *event) {
    zoom += event->angleDelta().y() * 0.001f; // تنظیم زوم
    if (zoom > -0.1f) zoom = -0.1f; // حداقل فاصله
    if (zoom < -10.0f) zoom = -10.0f; // حداکثر فاصله
    update(); // بازسازی صحنه
}

CalibrationPage::CalibrationPage(QWidget *parent, int numImus, bool isDarkTheme, USBReceiver *usbReceiver, QComboBox *externalCombo)
    : QWidget(parent),
    numImus(numImus),
    selectedIMU(-1),
    isAccelGyroCalibrating(false),
    isMagCalibrating(false),
    lastMagData(Eigen::Vector3d::Zero()),
    accelBias(Eigen::Vector3d::Zero()),
    gyroBias(Eigen::Vector3d::Zero()),
    magCenter(Eigen::Vector3d::Zero()),
    magRadii(Eigen::Vector3d::Zero()),
    imuSelector(externalCombo),
    btnSelectIMU(nullptr),
    btnAccelGyro(nullptr),
    btnMagCalib(nullptr),
    btnSave(nullptr),
    statusLabel(nullptr),
    pointsLabel(nullptr),
    glWidget(nullptr),
    toolbar(nullptr),
    isDarkTheme(isDarkTheme),
    isUSBConnected(false),
    usbReceiver(usbReceiver)
{
    QVBoxLayout *layout = new QVBoxLayout(this);

    // Initialize toolbar
    initToolbar();
    QFrame *toolbarFrame = new QFrame();
    toolbarFrame->setLayout(toolbar);
    toolbarFrame->setFixedHeight(60);
    toolbarFrame->setStyleSheet(isDarkTheme ? "background: #232526; border-radius: 18px;" : "background: #e0e0e0; border: 1px solid #333; border-radius: 18px;");
    layout->addWidget(toolbarFrame);
    // Add shared IMU selector to layout
    //layout->addWidget(new QLabel("Select IMU:"));
    quatWLabel = new QLabel("Quat W: 0.000", this);
    northDeviationLabel = new QLabel("North Deviation: 0.00°", this);
    layout->addWidget(northDeviationLabel);
    northDeviationLabel->setStyleSheet(isDarkTheme ? "color: #43cea2; font: 16px;" : "color: black; font: 16px;");
    layout->addWidget(quatWLabel);
    layout->addWidget(imuSelector);

    // Initialize IMU selector
    if (!imuSelector) {
        imuSelector = new QComboBox();
        for (int i = 1; i <= numImus; ++i) {
            imuSelector->addItem(QString("IMU %1").arg(i));
        }
        layout->addWidget(imuSelector);
    }

    btnSelectIMU = new QPushButton("Select IMU");
    connect(btnSelectIMU, &QPushButton::clicked, this, &CalibrationPage::selectIMU);
    layout->addWidget(btnSelectIMU);

    btnAccelGyro = new QPushButton("Calibrate Accelerometer and Gyroscope");
    btnAccelGyro->setEnabled(false);
    connect(btnAccelGyro, &QPushButton::clicked, this, &CalibrationPage::startAccelGyroCalibration);
    layout->addWidget(btnAccelGyro);

    btnMagCalib = new QPushButton("Calibrate Magnometer");
    btnMagCalib->setEnabled(false);
    connect(btnMagCalib, &QPushButton::clicked, this, &CalibrationPage::startMagCalibration);
    layout->addWidget(btnMagCalib);

    btnSave = new QPushButton("Save");
    btnSave->setEnabled(false);
    connect(btnSave, &QPushButton::clicked, this, &CalibrationPage::saveCalibration);
    layout->addWidget(btnSave);

    statusLabel = new QLabel("Please select an IMU.");
    layout->addWidget(statusLabel);

    pointsLabel = new QLabel("Collected Points: 0");
    layout->addWidget(pointsLabel);

    glWidget = new CalibrationGLWidget(this, isDarkTheme);
    glWidget->setMinimumSize(400, 400); // اندازه حداقل برای ویجت گرافیکی
    layout->addWidget(glWidget);

    layout->addStretch();

    // Connect IMU selector signal
    connect(imuSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &CalibrationPage::selectIMUFromCombo);

    // Connect USBReceiver signals
    if (usbReceiver) {
        connect(usbReceiver, &USBReceiver::data, this, &CalibrationPage::handleIMUData);
        connect(usbReceiver, &USBReceiver::connectionStatus, this, &CalibrationPage::handleUSBConnection);
        connect(usbReceiver->findChild<RecorderService*>(), &RecorderService::recordingStarted, this, &CalibrationPage::recordingStarted);
        connect(usbReceiver->findChild<RecorderService*>(), &RecorderService::recordingStopped, this, &CalibrationPage::recordingStopped);
        isUSBConnected = usbReceiver->isRunning();
    }

    // Apply theme
    applyTheme();
}

void CalibrationPage::applyTheme()
{
    QString widgetStyle = isDarkTheme ?
                              "QWidget { background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #232526, stop:1 #414345); }" :
                              "QWidget { background: white; }";
    QString buttonStyle = isDarkTheme ?
                              "QPushButton { background: #3e3e4e; color: #43cea2; border-radius: 12px; min-height: 38px; font: 16px 'Segoe UI'; }"
                              "QPushButton:pressed { background: #4e4e5e; }"
                              "QPushButton:hover { background: #43cea2; color: #232526; }" :
                              "QPushButton { background: #e0e0e0; color: black; border: 1px solid #333; border-radius: 12px; min-height: 38px; font: 16px 'Segoe UI'; }"
                              "QPushButton:hover { background: #333; color: white; }"
                              "QPushButton:pressed { background: #ccc; }";
    QString labelStyle = isDarkTheme ?
                             "QLabel { color: #f8f8f8; font: 16px 'Segoe UI'; }" :
                             "QLabel { color: black; font: 16px 'Segoe UI'; }";
    QString comboBoxStyle = isDarkTheme ?
                                "QComboBox { background: #3e3e4e; color: #43cea2; border-radius: 8px; font: 15px 'Segoe UI'; }"
                                "QComboBox QAbstractItemView { background: #232526; color: #43cea2; selection-background-color: #185a9d; }" :
                                "QComboBox { background: #e0e0e0; color: black; border: 1px solid #333; border-radius: 8px; font: 15px 'Segoe UI'; }"
                                "QComboBox QAbstractItemView { background: #ffffff; color: black; selection-background-color: #c0c0c0; }";
    QString toolbarButtonStyle = isDarkTheme ?
                                     "QPushButton { background: #232526; color: #43cea2; border-radius: 12px; font: 18px 'Segoe UI'; min-width: 140px; margin: 6px; }"
                                     "QPushButton:hover { background: #43cea2; color: #232526; }" :
                                     "QPushButton { background: #e0e0e0; color: black; border: 1px solid #333; border-radius: 12px; font: 18px 'Segoe UI'; min-width: 140px; margin: 6px; }"
                                     "QPushButton:hover { background: #333; color: white; }"
                                     "QPushButton:pressed { background: #ccc; }";

    setStyleSheet(widgetStyle);

    // Apply to buttons
    for (QPushButton *btn : {btnSelectIMU, btnAccelGyro, btnMagCalib, btnSave}) {
        btn->setStyleSheet(buttonStyle);
    }

    // Apply to labels
    statusLabel->setStyleSheet(labelStyle);
    pointsLabel->setStyleSheet(labelStyle);
    for (QWidget *widget : findChildren<QLabel*>()) {
        if (widget != statusLabel && widget != pointsLabel) {
            widget->setStyleSheet(labelStyle);
        }
    }
    quatWLabel->setStyleSheet(labelStyle);

    // Apply to combo box
    if (imuSelector) {
        imuSelector->setStyleSheet(comboBoxStyle);
    }

    // Update GL widget
    glWidget->update();
}

void CalibrationPage::initToolbar()
{
    toolbar = new QHBoxLayout();
    guideLabel = new QLabel("To calibrate, select IMU and press the corresponding buttons");
    toolbar->addWidget(guideLabel);
    toolbar->addStretch();
}

void CalibrationPage::selectIMUFromCombo(int index) {
    selectedIMU = index + 1;
    statusLabel->setText(QString("Selected IMU: %1").arg(selectedIMU));
    btnAccelGyro->setEnabled(true);
    btnMagCalib->setEnabled(false);
    btnSave->setEnabled(false);
    isAccelGyroCalibrating = false;
    isMagCalibrating = false;
    accSamples.clear();
    gyroSamples.clear();
    magDataPoints.clear();
    lastMagData = Eigen::Vector3d::Zero();
    glWidget->setPoints(magDataPoints); // پاک کردن نقاط گرافیکی
    pointsLabel->setText("Collected Points: 0");
}

void CalibrationPage::selectIMU() {
    // Deprecated function, kept for backward compatibility
    selectIMUFromCombo(imuSelector->currentIndex());
}

void CalibrationPage::startAccelGyroCalibration() {
    if (selectedIMU == -1) return;
    statusLabel->setText("Keep the IMU steady with the Y-axis pointing up.");
    isAccelGyroCalibrating = true;
    accSamples.clear();
    gyroSamples.clear();
}

void CalibrationPage::startMagCalibration() {
    if (selectedIMU == -1) return;
    statusLabel->setText("Rotate the IMU in all directions to collect quaternion data.");
    isMagCalibrating = true;
    magDataPoints.clear();
    lastMagData = Eigen::Vector3d::Zero();
    glWidget->setPoints(magDataPoints); // پاک کردن نقاط گرافیکی
    pointsLabel->setText("Collected Points: 0");
    btnSave->setEnabled(true);
}

void CalibrationPage::handleIMUData(const IMU &data) {
    if (data.id != selectedIMU) return;
    quatWLabel->setText(QString("Quat W: %1").arg(data.quat[0], 0, 'f', 3));
    float northDeviation = qAbs(data.quat[0] - 1.0f) * 100.0f;
    northDeviationLabel->setText(QString("North Deviation: %1°").arg(northDeviation, 0, 'f', 2));
    if (isAccelGyroCalibrating) {
        if (accSamples.size() < 100) {
            accSamples.push_back(Eigen::Vector3d(data.acc[0], data.acc[1], data.acc[2]));
            gyroSamples.push_back(Eigen::Vector3d(data.gyro[0], data.gyro[1], data.gyro[2]));
            if (accSamples.size() == 100) {
                Eigen::Vector3d accSum = Eigen::Vector3d::Zero();
                Eigen::Vector3d gyroSum = Eigen::Vector3d::Zero();
                for (const auto &sample : accSamples) accSum += sample;
                for (const auto &sample : gyroSamples) gyroSum += sample;
                accelBias = accSum / 100.0 - Eigen::Vector3d(0, 9.81, 0); // فرض محور Y رو به بالا
                gyroBias = gyroSum / 100.0;
                statusLabel->setText("Accelerometer and Gyroscope calibration completed.");
                isAccelGyroCalibrating = false;
                btnMagCalib->setEnabled(true);
            }
        }
    } else if (isMagCalibrating) {
        // ساخت کواترنیون از داده‌ها
        Eigen::Quaterniond q(data.quat[3], data.quat[0], data.quat[1], data.quat[2]);
        q.normalize(); // نرمال‌سازی کواترنیون
        Eigen::Matrix3d R = q.toRotationMatrix(); // ماتریس چرخش
        Eigen::Vector3d v = R * Eigen::Vector3d(1, 0, 0); // چرخاندن بردار ثابت
        Eigen::Vector3d currentMagData = v;

        if (magDataPoints.empty() || (currentMagData - lastMagData).norm() > 0.1) {
            if (magDataPoints.size() < 1800) {
                magDataPoints.push_back(currentMagData);
                pointsLabel->setText(QString("Collected Points: %1").arg(magDataPoints.size()));
                glWidget->setPoints(magDataPoints);
                lastMagData = currentMagData;
                if (magDataPoints.size() == 1800) {
                    statusLabel->setText("Maximum points collected. Press Save.");
                    isMagCalibrating = false;
                }
            }
        }
    }
}

void CalibrationPage::saveCalibration() {
    if (magDataPoints.size() < 100) {
        QMessageBox warningBox(this);
        warningBox.setWindowTitle("Insufficient Data");
        warningBox.setText("Please collect more points.");
        // Apply stylesheet to ensure readable text, specifically targeting QLabel
        QString messageBoxStyle = isDarkTheme ?
                                      "QMessageBox { background: #232526; } QMessageBox QLabel { color: #43cea2; }" :
                                      "QMessageBox { background: white; } QMessageBox QLabel { color: black; }";
        warningBox.setStyleSheet(messageBoxStyle);
        warningBox.exec();
        return;
    }
    performEllipsoidFit();
    QString message = QString("Accelerometer Bias: [%1, %2, %3]\n"
                              "Gyroscope Bias: [%4, %5, %6]\n"
                              "Quaternion Center: [%7, %8, %9]\n"
                              "Quaternion Radii: [%10, %11, %12]")
                          .arg(accelBias[0]).arg(accelBias[1]).arg(accelBias[2])
                          .arg(gyroBias[0]).arg(gyroBias[1]).arg(gyroBias[2])
                          .arg(magCenter[0]).arg(magCenter[1]).arg(magCenter[2])
                          .arg(magRadii[0]).arg(magRadii[1]).arg(magRadii[2]);
    QMessageBox infoBox(this);
    infoBox.setWindowTitle("Calibration Coefficients");
    infoBox.setText(message);
    // Apply stylesheet to ensure readable text, specifically targeting QLabel
    QString messageBoxStyle = isDarkTheme ?
                                  "QMessageBox { background: #232526; } QMessageBox QLabel { color: #43cea2; }" :
                                  "QMessageBox { background: white; } QMessageBox QLabel { color: black; }";
    infoBox.setStyleSheet(messageBoxStyle);
    infoBox.exec();
    emit calibrationCompleted(selectedIMU);
    btnMagCalib->setEnabled(false);
    btnSave->setEnabled(false);
}

void CalibrationPage::performEllipsoidFit() {
    int N = magDataPoints.size();
    Eigen::MatrixXd D(N, 9);
    Eigen::VectorXd d2(N);
    for (int i = 0; i < N; ++i) {
        double x = magDataPoints[i][0];
        double y = magDataPoints[i][1];
        double z = magDataPoints[i][2];
        D(i, 0) = x*x + y*y - 2*z*z;
        D(i, 1) = x*x + z*z - 2*y*y;
        D(i, 2) = 2*x*y;
        D(i, 3) = 2*x*z;
        D(i, 4) = 2*y*z;
        D(i, 5) = 2*x;
        D(i, 6) = 2*y;
        D(i, 7) = 2*z;
        D(i, 8) = 1;
        d2(i) = x*x + y*y + z*z;
    }
    Eigen::VectorXd u = (D.transpose() * D).ldlt().solve(D.transpose() * d2);
    Eigen::VectorXd v(10);
    v(0) = u(0) + u(1) - 1;
    v(1) = u(0) - 2*u(1) - 1;
    v(2) = u(1) - 2*u(0) - 1;
    v.segment(3, 3) = u.segment(2, 3);
    v.segment(6, 3) = u.segment(5, 3);
    v(9) = u(8);
    Eigen::Matrix4d A;
    A << v(0), v(3), v(4), v(6),
        v(3), v(1), v(5), v(7),
        v(4), v(5), v(2), v(8),
        v(6), v(7), v(8), v(9);
    magCenter = -A.block<3,3>(0,0).ldlt().solve(A.block<3,1>(0,3));
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.row(3).head(3) = magCenter.transpose();
    Eigen::Matrix4d R = T * A * T.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(R.block<3,3>(0,0) / -R(3,3));
    Eigen::Vector3d evals = eig.eigenvalues();
    magRadii = (1.0 / evals.array().abs().sqrt()).matrix().cwiseProduct(evals.array().sign().matrix());
}

void CalibrationPage::handleUSBConnection(bool connected)
{
    isUSBConnected = connected;
}
