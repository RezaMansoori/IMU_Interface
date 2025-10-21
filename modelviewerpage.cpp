#include "modelviewerpage.h"
#include <QMessageBox>

ModelViewerPage::ModelViewerPage(QWidget *parent, int numImus, bool isDarkTheme, QComboBox *externalCombo)
    : QWidget(parent),
    numImus(numImus),
    isDarkTheme(isDarkTheme),
    isSimulating(false),
    selectedIMU(-1),
    referenceQuat(QQuaternion()),
    combo(externalCombo)
{
    timer = new QTimer(this);
    initUI();
    applyTheme();
}

void ModelViewerPage::initUI()
{
    mainLayout = new QVBoxLayout(this);

    plotter = new Object3D();
    mainLayout->addWidget(plotter, 1);

    /*
    if (!combo) {
        combo = new QComboBox();
        for (int i = 1; i <= numImus; ++i) {
            combo->addItem(QString("IMU %1").arg(i));
        }
        mainLayout->addWidget(combo);
    }
    */

    connect(combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &ModelViewerPage::changeIMU);

    QHBoxLayout *controls = new QHBoxLayout();
    btnStart = new QPushButton("▶ Start Simulation");
    btnStop = new QPushButton("⏹ Stop Simulation");
    btnReset = new QPushButton("🔄 Reset View");
    btnRotate = new QPushButton("⟳ Rotate");

    for (QPushButton *btn : {btnStart, btnStop, btnReset, btnRotate}) {
        controls->addWidget(btn);
    }

    connect(btnStart, &QPushButton::clicked, this, &ModelViewerPage::startSimulation);
    connect(btnStop, &QPushButton::clicked, this, &ModelViewerPage::stopSimulation);
    connect(btnReset, &QPushButton::clicked, this, &ModelViewerPage::resetView);
    connect(btnRotate, &QPushButton::clicked, this, &ModelViewerPage::rotateView);

    btnStop->setEnabled(false);

    mainLayout->addLayout(controls);
}

void ModelViewerPage::applyTheme()
{
    QString widgetStyle = isDarkTheme ?
                              "ModelViewerPage { background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #232526, stop:1 #414345); }" :
                              "ModelViewerPage { background: white; }";
    QString comboStyle = isDarkTheme ?
                             "QComboBox {"
                             "    background: #232526;"
                             "    color: #43cea2;"
                             "    border-radius: 8px;"
                             "    font: 15pt 'Segoe UI';"
                             "}"
                             "QComboBox QAbstractItemView {"
                             "    background: #232526;"
                             "    color: #43cea2;"
                             "    selection-background-color: #185a9d;"
                             "}" :
                             "QComboBox {"
                             "    background: #e0e0e0;"
                             "    color: black;"
                             "    border: 1px solid #333;"
                             "    border-radius: 8px;"
                             "    font: 15pt 'Segoe UI';"
                             "}"
                             "QComboBox QAbstractItemView {"
                             "    background: #ffffff;"
                             "    color: black;"
                             "    selection-background-color: #333;"
                             "}";
    QString buttonStyle = isDarkTheme ?
                              "QPushButton {"
                              "    background: #232526;"
                              "    color: #43cea2;"
                              "    border-radius: 8px;"
                              "    font: 14pt 'Segoe UI';"
                              "}"
                              "QPushButton:hover {"
                              "    background: #43cea2;"
                              "    color: #232526;"
                              "}" :
                              "QPushButton {"
                              "    background: #e0e0e0;"
                              "    color: black;"
                              "    border: 1px solid #333;"
                              "    border-radius: 8px;"
                              "    font: 14pt 'Segoe UI';"
                              "}"
                              "QPushButton:hover {"
                              "    background: #333;"
                              "    color: white;"
                              "}"
                              "QPushButton:pressed {"
                              "    background: #ccc;"
                              "}";
    QString plotterStyle = isDarkTheme ?
                               "Object3D { background: #232526; }" :
                               "Object3D { background: #f5f5f5; }";

    setStyleSheet(widgetStyle);
    if (combo) {
        combo->setStyleSheet(comboStyle);
    }
    for (QPushButton *btn : {btnStart, btnStop, btnReset, btnRotate}) {
        btn->setStyleSheet(buttonStyle);
    }
    plotter->setStyleSheet(plotterStyle);
}

void ModelViewerPage::setReferenceQuat(const QQuaternion &quat)
{
    referenceQuat = quat;
}

void ModelViewerPage::changeIMU(int index)
{
    selectedIMU = index + 1;
}

void ModelViewerPage::startSimulation()
{
    if (selectedIMU >= 1) {
        isSimulating = true;
        btnStart->setEnabled(false);
        btnStop->setEnabled(true);
    } else {
        QMessageBox::warning(this, "Error", "Please select an IMU first!");
    }
}

void ModelViewerPage::stopSimulation()
{
    isSimulating = false;
    btnStart->setEnabled(true);
    btnStop->setEnabled(false);
}

void ModelViewerPage::rotateView()
{
    QQuaternion currentRotation = plotter->getRotation();
    QQuaternion rotationDelta = QQuaternion::fromAxisAndAngle(0, 1, 0, 10);
    plotter->setRotation(currentRotation * rotationDelta);
}

void ModelViewerPage::resetView()
{
    plotter->setRotation(QQuaternion());
    plotter->resetCamera();
}

void ModelViewerPage::updateModel(const IMU &data)
{
    if (isSimulating && data.id == selectedIMU) {
        QQuaternion quat(data.quat[0], data.quat[1], data.quat[2], data.quat[3]);
        if (!referenceQuat.isIdentity()) {
            QQuaternion adjustedQuat = referenceQuat.conjugated() * quat;
            QQuaternion viewAdjust = QQuaternion::fromAxisAndAngle(1, 0, 0, -90);
            QQuaternion zAdjust = QQuaternion::fromAxisAndAngle(0, 0, 1, -90);
            adjustedQuat = zAdjust * viewAdjust * adjustedQuat;
            plotter->setRotation(adjustedQuat.normalized());
        } else {
            plotter->setRotation(quat);
        }
    }
}
