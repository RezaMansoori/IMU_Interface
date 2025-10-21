#ifndef STARTPAGE_H
#define STARTPAGE_H

#include <QWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QComboBox>
#include <QGridLayout>
#include <QTimer>
#include <QMap>
#include <QFrame>
#include "usbreceiver.h"
#include "imu.h"
#include "object3d.h"

class StartPage : public QWidget
{
    Q_OBJECT

public:
    explicit StartPage(QWidget *parent = nullptr, bool isDarkTheme = true, USBReceiver *usbReceiver = nullptr);
    void applyTheme();

    int numImus; // Added for dynamic IMU count
    QQuaternion referenceQuat; // کوانترنیون مرجع برای کالیبراسیون (public برای دسترسی در main.cpp)

signals:
    void imuDataReceived(const IMU &data);
    void recordingStarted(const QString &fileName);
    void recordingStopped(const QString &fileName);
    void startSimulationRequested(); // سیگنال قبلی
    void showInTableRequested(int imuId); // <--- سیگنال جدید: درخواست نمایش در جدول
    void showInChartRequested(int imuId); // <--- سیگنال جدید: درخواست نمایش در چارت
    void calibrateIMURequested(int imuId); // <--- سیگنال جدید: درخواست کالیبراسیون
    void imusSelected(const QMap<QString, QVariant> &selectedImus); // سیگنال جدید

public slots:
    void showStep1();
    void showStep2();
    void showStep3();
    void startCountdown();
    void updateCountdown();
    void saveAndNext();
    void rotate();
    void toTPose();
    void resetView();
    void handleUSBData(const IMU &data);
    void handleUSBConnection(bool connected);
    void updateModel(const IMU &data);
    void calibratePelvis(const IMU &data);

private:
    void clearLeft();
    void buildLeftFrame(const QString &title, const QList<QWidget*> &bodyWidgets, const QList<QPushButton*> &buttons);
    void buildRight3DView();
    void showContextMenu(const QPoint &pos, int imuId); // <--- تابع جدید: نمایش context menu

    QVBoxLayout *vbox;
    QHBoxLayout *mainLayout;
    QWidget *leftWidget;
    QLabel *lblCount;
    QTimer *countdownTimer;
    int countValue;
    int countStage;
    QStringList settings;
    QMap<QString, QString> comboValues;
    QMap<QString, QComboBox*> combos;
    Object3D *plotter;
    bool isUSBConnected;
    USBReceiver *usbReceiver;
    bool isDarkTheme;
};

#endif // STARTPAGE_H
