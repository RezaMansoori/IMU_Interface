#ifndef MODELVIEWERPAGE_H
#define MODELVIEWERPAGE_H

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QComboBox>
#include <QTimer>
#include "imu.h"
#include "object3d.h"

class ModelViewerPage : public QWidget
{
    Q_OBJECT

public:
    explicit ModelViewerPage(QWidget *parent = nullptr, int numImus = 18, bool isDarkTheme = true, QComboBox *externalCombo = nullptr);
    void setReferenceQuat(const QQuaternion &quat);
    void applyTheme();

public slots:
    void changeIMU(int index);
    void startSimulation();
    void stopSimulation();
    void rotateView();
    void resetView();
    void updateModel(const IMU &data);

private:
    void initUI();
    int numImus;
    bool isDarkTheme;
    bool isSimulating;
    int selectedIMU;
    QComboBox *combo;
    QVBoxLayout *mainLayout;
    Object3D *plotter;
    QPushButton *btnStart;
    QPushButton *btnStop;
    QPushButton *btnReset;
    QPushButton *btnRotate;
    QTimer *timer;
    QQuaternion referenceQuat;
};

#endif // MODELVIEWERPAGE_H
