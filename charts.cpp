#include "charts.h"
#include "imguiplot.h"
#include "imu.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QTimer>
#include <QVector3D>
#include <QQuaternion>
#include <QApplication>

Charts::Charts(QWidget *parent, int numImus, bool isDarkTheme, USBReceiver *usbReceiver)
    : QWidget(parent),
    numImus(numImus),
    isDarkTheme(isDarkTheme),
    isUSBConnected(false),
    usbReceiver(usbReceiver),
    m_layout(new QVBoxLayout(this)),
    m_hLayout(new QHBoxLayout()),
    m_label(nullptr),
    m_imu_combo_box(nullptr),
    m_timer(new QTimer(this)),
    m_plot(nullptr)
{
    // Initialize IMU selection
    m_label = new QLabel("Select IMU:");
    m_hLayout->addWidget(m_label);

    m_imu_combo_box = new QComboBox();
    for (int i = 1; i <= numImus; ++i)
        m_imu_combo_box->addItem(QString("IMU %1").arg(i), i);
    m_hLayout->addWidget(m_imu_combo_box);

    m_layout->addLayout(m_hLayout);

    // Initialize plot
    m_plot = new ImGuiPlot(this);
    m_plot->setChartTitle(0, "Acceleration (g)");
    m_plot->setPlotRange(0, -2, 2);
    m_plot->setChartTitle(1, "Gyroscope (deg/s)");
    m_plot->setPlotRange(1, -360, 360);
    m_plot->setChartTitle(2, "Linear Acceleration (g)");
    m_plot->setPlotRange(2, -2, 2);
    m_plot->setChartTitle(3, "Euler Angles (deg)");
    m_plot->setPlotRange(3, -360, 360);
    m_layout->addWidget(m_plot);

    isPlotting = false;

    // اضافه کردن دکمه‌های Start و Stop
    m_startButton = new QPushButton("▶️", this);
    m_stopButton = new QPushButton("⏹️", this);
    m_startButton->setFixedSize(40, 30); // اندازه کوچیک (40 عرض، 30 ارتفاع)
    m_stopButton->setFixedSize(40, 30);  // اندازه کوچیک
    m_hLayout->addWidget(m_startButton);
    m_hLayout->addWidget(m_stopButton);

    // غیرفعال کردن دکمه Stop در ابتدا
    m_stopButton->setEnabled(false);

    // اتصال سیگنال‌های دکمه‌ها به اسلات‌ها
    connect(m_startButton, &QPushButton::clicked, this, &Charts::startPlotting);
    connect(m_stopButton, &QPushButton::clicked, this, &Charts::stopPlotting);

    setLayout(m_layout);

    // Connect timer to update plot
    connect(m_timer, &QTimer::timeout, m_plot, QOverload<>::of(&ImGuiPlot::update));
    m_timer->start(1000 / 40);

    // Connect USBReceiver signals
    if (usbReceiver) {
        connect(usbReceiver, &USBReceiver::data, this, &Charts::appendData);
        connect(usbReceiver, &USBReceiver::connectionStatus, this, &Charts::handleUSBConnection);
        connect(usbReceiver->findChild<RecorderService*>(), &RecorderService::recordingStarted, this, &Charts::handleRecordingStarted);
        connect(usbReceiver->findChild<RecorderService*>(), &RecorderService::recordingStopped, this, &Charts::handleRecordingStopped);
        isUSBConnected = usbReceiver->isRunning();
    }

    applyTheme();
}

Charts::~Charts() {}

void Charts::applyTheme()
{
    QString widgetStyle = isDarkTheme ?
                              "Charts { background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #232526, stop:1 #414345); }" :
                              "Charts { background: white; }";
    QString labelStyle = isDarkTheme ?
                             "QLabel { color: #f8f8f8; font: 16px 'Segoe UI'; }" :
                             "QLabel { color: black; font: 16px 'Segoe UI'; }";
    QString comboStyle = isDarkTheme ?
                             "QComboBox {"
                             "    background: #3e3e4e;"
                             "    color: #43cea2;"
                             "    border-radius: 8px;"
                             "    font: 15px 'Segoe UI';"
                             "}"
                             "QComboBox QAbstractItemView {"
                             "    background: #3e3e4e;"
                             "    color: #43cea2;"
                             "    selection-background-color: #185a9d;"
                             "}" :
                             "QComboBox {"
                             "    background: #e0e0e0;"
                             "    color: black;"
                             "    border: 1px solid #333;"
                             "    border-radius: 8px;"
                             "    font: 15px 'Segoe UI';"
                             "}"
                             "QComboBox QAbstractItemView {"
                             "    background: #ffffff;"
                             "    color: black;"
                             "    selection-background-color: #c0c0c0;"
                             "}";

    setStyleSheet(widgetStyle);
    m_label->setStyleSheet(labelStyle);
    m_imu_combo_box->setStyleSheet(comboStyle);

    // استایل دکمه‌های Start و Stop
    QString buttonStyle = isDarkTheme ?
                              "QPushButton { background: #232526; color: #43cea2; border-radius: 8px; font: 15px 'Segoe UI'; padding: 5px; }"
                              "QPushButton:hover { background: #43cea2; color: #232526; }"
                              "QPushButton:disabled { background: #414345; color: #888888; }" :
                              "QPushButton { background: #e0e0e0; color: black; border: 1px solid #333; border-radius: 8px; font: 15px 'Segoe UI'; padding: 5px; }"
                              "QPushButton:hover { background: #333; color: white; }"
                              "QPushButton:disabled { background: #ccc; color: #888888; }";
    m_startButton->setStyleSheet(buttonStyle);
    m_stopButton->setStyleSheet(buttonStyle);
}

void Charts::appendData(const IMU& data)
{
    if (!isPlotting) return; // فقط وقتی isPlotting فعال است داده‌ها رسم شوند

    if (m_imu_combo_box->currentData().toInt() == data.id) {
        m_plot->addData(0, static_cast<float>(index), data.acc[0], data.acc[1], data.acc[2]);
        m_plot->addData(1, static_cast<float>(index), data.gyro[0], data.gyro[1], data.gyro[2]);
        m_plot->addData(2, static_cast<float>(index), data.lacc[0], data.lacc[1], data.lacc[2]);

        QVector3D euler = data.eulars();
        m_plot->addData(3, static_cast<float>(index), euler.x(), euler.y(), euler.z());

        index++;
        m_plot->update(); // به‌روزرسانی نمودار
    }
}

void Charts::handleUSBConnection(bool connected)
{
    isUSBConnected = connected;
    m_startButton->setEnabled(connected && !isPlotting); // فعال بودن Start وقتی USB متصل است و رسم غیرفعال است
    m_stopButton->setEnabled(connected && isPlotting);   // فعال بودن Stop وقتی USB متصل است و رسم فعال است
}

void Charts::handleRecordingStarted(const QString &fileName)
{
    Q_UNUSED(fileName);
}

void Charts::handleRecordingStopped(const QString &fileName)
{
    Q_UNUSED(fileName);
}

void Charts::selectIMU(int imuId)
{
    int index = imuId - 1; // comboBox از 0 شروع می‌شه
    if (index >= 0 && index < m_imu_combo_box->count()) {
        m_imu_combo_box->setCurrentIndex(index); // انتخاب IMU در comboBox
    }
}

void Charts::startPlotting()
{
    if (isUSBConnected) {
        isPlotting = true;
        m_startButton->setEnabled(false);
        m_stopButton->setEnabled(true);

        // ریست اندیس و مقداردهی اولیه با صفر
        index = 0;
        for (int i = 0; i < 4; ++i) {
            m_plot->addData(i, 0.0f, 0.0f, 0.0f, 0.0f);
        }
        m_plot->update(); // به‌روزرسانی نمودار
    }
}

void Charts::stopPlotting()
{
    isPlotting = false;
    m_startButton->setEnabled(true);
    m_stopButton->setEnabled(false);

    // پر کردن کل نمودار با داده‌های صفر برای پاک کردن رسم قبلی و ایجاد خط صاف
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 400; ++j) { // استفاده از 400 که با data_size مطابقت داره
            m_plot->addData(i, static_cast<float>(j), 0.0f, 0.0f, 0.0f);
        }
    }
    m_plot->update(); // به‌روزرسانی نمودار

    // ریست اندیس برای شروع از ابتدا
    index = 0;
}
