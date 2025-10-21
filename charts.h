#ifndef CHARTS_H
#define CHARTS_H

#include <QWidget>
#include "usbreceiver.h"
#include "imu.h"
#include <QPushButton>

class QVBoxLayout;
class QHBoxLayout;
class QLabel;
class QComboBox;
class QTimer;
class ImGuiPlot;

class Charts : public QWidget {
    Q_OBJECT

public:
    explicit Charts(QWidget* parent = nullptr, int numImus = 18, bool isDarkTheme = true, USBReceiver *usbReceiver = nullptr);
    ~Charts();
    void applyTheme();

public slots:
    void appendData(const IMU& data);
    void handleUSBConnection(bool connected);
    void handleRecordingStarted(const QString &fileName);
    void handleRecordingStopped(const QString &fileName);
    void selectIMU(int imuId); // <--- slot جدید: انتخاب IMU در comboBox
    void startPlotting(); // اسلات برای شروع رسم
    void stopPlotting();  // اسلات برای توقف رسم

signals:
    void recordingStarted(const QString &fileName);
    void recordingStopped(const QString &fileName);

private:
    int numImus;
    bool isDarkTheme;
    bool isUSBConnected;
    bool isPlotting; // اضافه کردن متغیر برای کنترل وضعیت رسم
    USBReceiver* usbReceiver;
    QVBoxLayout* m_layout;
    QHBoxLayout* m_hLayout;
    QLabel* m_label;
    QComboBox* m_imu_combo_box;
    QTimer* m_timer;
    ImGuiPlot* m_plot;
    QPushButton* m_startButton; // دکمه Start
    QPushButton* m_stopButton;  // دکمه Stop
    int index; // اضافه کردن متغیر اندیس
};

#endif // CHARTS_H
