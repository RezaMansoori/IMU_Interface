#ifndef IMUSDATAPAGE_H
#define IMUSDATAPAGE_H

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QTableWidget>
#include <QScrollArea>
#include <QFrame>
#include <QHeaderView>
#include <QMap>
#include <QPixmap>
#include <QTimer>
#include <QStackedWidget>
#include "usbreceiver.h"
#include "imu.h"

class IMUsDataPage : public QWidget
{
    Q_OBJECT

public:
    explicit IMUsDataPage(QWidget *parent = nullptr, int numImus = 18, bool isDarkTheme = true);
    void applyTheme();

public slots:
    void updateIMUData(const IMU &data);
    void toggleIMUSelection(int imuId, bool checked);
    void refreshTable();
    void handleRowSelection(QTableWidgetItem *item);
    void selectIMU(int imuId); // <--- slot جدید: انتخاب سطر IMU
    void onCalibrationCompleted(int imuId); // <--- اسلات جدید

private:
    void initUI();
    QPixmap getBatteryIcon(int level);
    QString getBatteryColor(int level);
    double calculateHertz(int idx);

    QVBoxLayout *mainLayout;
    QTableWidget *table;
    QList<QPushButton*> imuButtons;
    QList<QLabel*> batteryLabels;
    QList<QLabel*> batteryPercentLabels;
    QMap<int, QLabel*> tickLabels; // برای ردیابی لیبل‌های تیک هر IMU
    int numImus;
    int selectedRow;
    QMap<QString, QPixmap> battery_icons;
    QPixmap tick_icon;
    QMap<int, bool> imuSelection;
    QMap<int, IMU> imuData;
    QList<QList<qint64>> imuTimestamps;
    QTimer *refreshTimer;
    qint64 lastMasterTime;
    bool isDarkTheme;
    QMap<int, bool> imuDataChanged;
    QMap<int, int> lastBatteryLevel;
    QMap<int, bool> imuCalibrated; // <--- برای ردیابی وضعیت کالیبراسیون
};

#endif // IMUSDATAPAGE_H
