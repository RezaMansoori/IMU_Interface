#include <QApplication>
#include "loginpage.h"
#include "registerpage.h"
#include "startpage.h"
#include "simulationpage.h"
#include "modelviewerpage.h"
#include "filespage.h"
#include "calibrationpage.h"
#include "imusdatapage.h"
#include "charts.h"
#include <QMainWindow>
#include <QStackedWidget>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QSpinBox>
#include "usbreceiver.h"
#include "combinedcalibrationviewerpage.h"
#include "postprocesspage.h"

class IMUInterface : public QMainWindow {
public:
    IMUInterface(int axisMode, int numImus, bool isDarkTheme, QWidget *parent = nullptr) : QMainWindow(parent), isDarkTheme(isDarkTheme) {
        setWindowTitle("IMU Interface");
        resize(1440, 850);

        QWidget *centralWidget = new QWidget(this);
        setCentralWidget(centralWidget);
        QVBoxLayout *mainVerticalLayout = new QVBoxLayout(centralWidget); // Layout اصلی عمودی برای toolbar و بقیه

        // Initialize shared toolbar
        initToolbar();
        delayTimer = new QTimer(this);
        delayTimer->setSingleShot(true);
        connect(delayTimer, &QTimer::timeout, this, &IMUInterface::actualStartRecording);

        durationTimer = new QTimer(this);
        durationTimer->setSingleShot(true);
        connect(durationTimer, &QTimer::timeout, this, &IMUInterface::stopRecording);
        QFrame *toolbarFrame = new QFrame();
        toolbarFrame->setLayout(toolbar);
        toolbarFrame->setFixedHeight(60);
        toolbarFrame->setStyleSheet(isDarkTheme ? "background: #232526; border-radius: 18px;" : "background: #e0e0e0; border: 1px solid #333; border-radius: 18px;");
        mainVerticalLayout->addWidget(toolbarFrame);

        // Sidebar and stackedWidget layout
        QHBoxLayout *contentLayout = new QHBoxLayout();
        QVBoxLayout *sidebar = new QVBoxLayout();
        QPushButton *btnStart = new QPushButton("🏠");
        QPushButton *btnSimulation = new QPushButton("🧍");
        QPushButton *btnFiles = new QPushButton("📁");
        QPushButton *btnIMUDashboard = new QPushButton("📊");
        QPushButton *btnIMUsData = new QPushButton("📋");
        QPushButton *btnModelViewer = new QPushButton("🔍");
        QPushButton *btnCalibration = new QPushButton("🔧");
        QPushButton *btnPostProcess = new QPushButton("📈");

        for (QPushButton *btn : {btnStart, btnSimulation, btnFiles, btnIMUDashboard, btnIMUsData, btnModelViewer, btnCalibration, btnPostProcess})
            btn->setFixedWidth(80);

        QString buttonStyle = isDarkTheme ?
                                  "QPushButton { background: #232526; color: #43cea2; border-radius: 12px; font: 22pt 'Segoe UI Emoji'; margin: 8px; min-height: 60px; }"
                                  "QPushButton:hover { background: #43cea2; color: #232526; }"
                                  "QPushButton:disabled { background: #414345; color: #888888; border: 1px solid #555555; }" :
                                  "QPushButton { background: #e0e0e0; color: black; border: 1px solid #333; border-radius: 12px; font: 22pt 'Segoe UI Emoji'; margin: 8px; min-height: 60px; }"
                                  "QPushButton:hover { background: #333; color: white; }"
                                  "QPushButton:disabled { background: #ccc; color: #888888; border: 1px solid #555555; }";

        for (QPushButton *btn : {btnStart, btnSimulation, btnFiles, btnIMUDashboard, btnIMUsData, btnModelViewer, btnCalibration, btnPostProcess})
            btn->setStyleSheet(buttonStyle);

        sidebar->addWidget(btnStart);
        sidebar->addWidget(btnCalibration);
        sidebar->addWidget(btnIMUsData);
        sidebar->addWidget(btnIMUDashboard);
        sidebar->addWidget(btnSimulation);
        sidebar->addWidget(btnFiles);
        //sidebar->addWidget(btnModelViewer);
        sidebar->addWidget(btnPostProcess);
        sidebar->addStretch();

        stackedWidget = new QStackedWidget();
        usbReceiver = new USBReceiver(axisMode, numImus, this);
        std::map<std::string, int> selectedImus;
        QVariantMap imus;
        for (const auto &p : selectedImus) {
            imus[QString::fromStdString(p.first)] = p.second;
        }

        simulationPage = new SimulationPage(this, usbReceiver, true, imus);

        // simulationPage = new SimulationPage(this, usbReceiver, isDarkTheme, selectedImus);
        startPage = new StartPage(this, isDarkTheme, usbReceiver);
        connect(startPage, &StartPage::startSimulationRequested, this, [this]() { stackedWidget->setCurrentWidget(simulationPage); });
        connect(startPage, &StartPage::showInTableRequested, this, [this](int imuId) {
            stackedWidget->setCurrentWidget(imusDataPage);
            imusDataPage->selectIMU(imuId); // انتخاب IMU در صفحه جدول
        });

        connect(startPage, &StartPage::showInChartRequested, this, [this](int imuId) {
            stackedWidget->setCurrentWidget(imuDashboardPage);
            imuDashboardPage->selectIMU(imuId); // انتخاب IMU در صفحه چارت
        });

        connect(startPage, &StartPage::calibrateIMURequested, this, [this](int imuId) {
            stackedWidget->setCurrentWidget(combinedPage);
            combinedPage->getSharedComboBox()->setCurrentIndex(imuId - 1); // انتخاب IMU در comboBox مشترک (شروع از 0)
        });
        connect(startPage, &StartPage::imusSelected, simulationPage, &SimulationPage::updateSelectedImus);
        QMap<QString, QVariant> qSelectedImus = property("selectedImus").toMap();
        //std::map<std::string, int> selectedImus;
        for (auto it = qSelectedImus.constBegin(); it != qSelectedImus.constEnd(); ++it) {
            selectedImus[it.key().toStdString()] = it.value().toInt();
        }
        //simulationPage = new SimulationPage(this, isDarkTheme, selectedImus); // پاس دادن selectedImus
        //simulationPage = new SimulationPage(this, isDarkTheme, selectedImus);
        modelViewerPage = new ModelViewerPage(this, numImus, isDarkTheme);
        filesPage = new FilesPage(this, isDarkTheme);
        imuDashboardPage = new Charts(this, numImus, isDarkTheme, usbReceiver);
        imusDataPage = new IMUsDataPage(this, numImus, isDarkTheme);
        combinedPage = new CombinedCalibrationViewerPage(this, numImus, isDarkTheme, usbReceiver);
        postProcessPage = new PostProcessPage(this, isDarkTheme);

        stackedWidget->addWidget(startPage);
        stackedWidget->addWidget(simulationPage);
        stackedWidget->addWidget(filesPage);
        stackedWidget->addWidget(imuDashboardPage);
        stackedWidget->addWidget(imusDataPage);
        stackedWidget->addWidget(modelViewerPage);
        stackedWidget->addWidget(combinedPage);
        stackedWidget->addWidget(postProcessPage);

        connect(usbReceiver, &USBReceiver::data, imusDataPage, &IMUsDataPage::updateIMUData);
        connect(combinedPage->getCalibrationPage(), &CalibrationPage::calibrationCompleted, imusDataPage, &IMUsDataPage::onCalibrationCompleted);

        // Connect shared toolbar signals
        connect(btnSearchUSB, &QPushButton::clicked, this, &IMUInterface::searchUSB);
        connect(btnRecord, &QPushButton::clicked, this, &IMUInterface::startRecording);
        connect(btnStop, &QPushButton::clicked, this, &IMUInterface::stopRecording);

        // Connect USBReceiver signals (shared)
        if (usbReceiver) {
            connect(usbReceiver, &USBReceiver::connectionStatus, this, &IMUInterface::handleUSBConnection);
            RecorderService *recorder = usbReceiver->findChild<RecorderService*>();
            if (recorder) {
                connect(recorder, &RecorderService::recordingStarted, this, &IMUInterface::handleRecordingStarted);
                connect(recorder, &RecorderService::recordingStopped, this, &IMUInterface::handleRecordingStopped);
                connect(recorder, &RecorderService::recordingStarted, filesPage, &FilesPage::addFileToList); // مستقیم به filesPage
                connect(recorder, &RecorderService::recordingStopped, filesPage, &FilesPage::addFileToList); // مستقیم به filesPage
            }
            isUSBConnected = usbReceiver->isRunning();
            updateConnectionStatus(isUSBConnected);
            btnRecord->setEnabled(isUSBConnected);
            btnStop->setEnabled(isUSBConnected && usbReceiver->isRecording());
        }

        connect(btnStart, &QPushButton::clicked, this, [this]() { stackedWidget->setCurrentWidget(startPage); });
        connect(btnSimulation, &QPushButton::clicked, this, [this]() { stackedWidget->setCurrentWidget(simulationPage); });
        connect(btnModelViewer, &QPushButton::clicked, this, [this]() {
            stackedWidget->setCurrentWidget(modelViewerPage);
            modelViewerPage->setReferenceQuat(startPage->referenceQuat);
            connect(usbReceiver, &USBReceiver::data, modelViewerPage, &ModelViewerPage::updateModel);
        });
        connect(btnFiles, &QPushButton::clicked, this, [this]() {
            stackedWidget->setCurrentWidget(filesPage);
            disconnect(usbReceiver, &USBReceiver::data, modelViewerPage, &ModelViewerPage::updateModel);
        });
        connect(btnIMUDashboard, &QPushButton::clicked, this, [this]() {
            stackedWidget->setCurrentWidget(imuDashboardPage);
            disconnect(usbReceiver, &USBReceiver::data, modelViewerPage, &ModelViewerPage::updateModel);
        });
        connect(btnIMUsData, &QPushButton::clicked, this, [this]() {
            stackedWidget->setCurrentWidget(imusDataPage); // اصلاح نام متغیر
            disconnect(usbReceiver, &USBReceiver::data, modelViewerPage, &ModelViewerPage::updateModel);
        });
        connect(btnCalibration, &QPushButton::clicked, this, [this]() {
            stackedWidget->setCurrentWidget(combinedPage);
            combinedPage->getModelViewerPage()->setReferenceQuat(startPage->referenceQuat);
            disconnect(usbReceiver, &USBReceiver::data, modelViewerPage, &ModelViewerPage::updateModel);
        });
        connect(btnPostProcess, &QPushButton::clicked, this, [this]() { stackedWidget->setCurrentWidget(postProcessPage); });

        contentLayout->addLayout(sidebar);
        contentLayout->addWidget(stackedWidget, 1);

        mainVerticalLayout->addLayout(contentLayout); // اضافه کردن contentLayout زیر toolbar

        setStyleSheet(isDarkTheme ?
                          "QMainWindow { background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #232526, stop:1 #414345); }" :
                          "QMainWindow { background: white; }");
        stackedWidget->setCurrentWidget(startPage);
    }

private:
    QStackedWidget *stackedWidget;
    USBReceiver *usbReceiver;
    StartPage *startPage;
    SimulationPage *simulationPage;
    ModelViewerPage *modelViewerPage;
    FilesPage *filesPage;
    Charts *imuDashboardPage;
    IMUsDataPage *imusDataPage; // اصلاح نام متغیر
    CombinedCalibrationViewerPage *combinedPage;
    PostProcessPage *postProcessPage;
    bool isDarkTheme;
    bool isUSBConnected;

    // Shared toolbar members
    QHBoxLayout *toolbar;
    QPushButton *connectionLabel;
    QPushButton *btnStop;
    QPushButton *btnRecord;
    QLabel *recordLabel;
    QPushButton *btnSearchUSB;
    QSpinBox *startDelaySpinBox;
    QSpinBox *recordingDurationSpinBox;
    QTimer *delayTimer;
    QTimer *durationTimer;

    void initToolbar() {
        toolbar = new QHBoxLayout();

        connectionLabel = new QPushButton("🔴 Disconnected");
        connectionLabel->setEnabled(false);
        connectionLabel->setFixedWidth(170);
        updateConnectionStatus(false); // Will apply theme-based style
        toolbar->addWidget(connectionLabel);
        toolbar->addSpacing(20);

        btnStop = new QPushButton("⏹️ Stop");
        btnRecord = new QPushButton("⏺️ Record");
        recordLabel = new QLabel("●");
        btnSearchUSB = new QPushButton("🔄 Search USB");

        for (QPushButton *btn : {btnStop, btnRecord, btnSearchUSB}) {
            toolbar->addWidget(btn);
        }

        toolbar->addSpacing(20);

        QLabel *delayLabel = new QLabel("Start Delay (s):");
        delayLabel->setStyleSheet(isDarkTheme ? "color: #43cea2; font: 18px 'Segoe UI';" : "color: black; font: 18px 'Segoe UI'; border: 1px solid #e0e0e0;");
        toolbar->addWidget(delayLabel);

        startDelaySpinBox = new QSpinBox();
        startDelaySpinBox->setRange(0, 60); // محدوده 0 تا 60 ثانیه
        startDelaySpinBox->setValue(0); // مقدار پیش‌فرض 0
        startDelaySpinBox->setStyleSheet(isDarkTheme ?
                                             "QSpinBox { background: #3e3e4e; color: #43cea2; border-radius: 8px; padding: 5px; }"
                                             "QSpinBox::up-button { width: 20px; background: #43cea2; color: #232526; }"
                                             "QSpinBox::down-button { width: 20px; background: #43cea2; color: #232526; }" :
                                             "QSpinBox { background: #e0e0e0; color: black; border: 1px solid #333; border-radius: 8px; padding: 5px; }"
                                             "QSpinBox::up-button { width: 20px;}"
                                             "QSpinBox::down-button { width: 20px;}");
        startDelaySpinBox->setFixedWidth(100);
        toolbar->addWidget(startDelaySpinBox);

        toolbar->addSpacing(20);

        QLabel *durationLabel = new QLabel("Duration (s):");
        durationLabel->setStyleSheet(isDarkTheme ? "color: #43cea2; font: 18px 'Segoe UI';" : "color: black; font: 18px 'Segoe UI'; border: 1px solid #e0e0e0;");
        toolbar->addWidget(durationLabel);

        recordingDurationSpinBox = new QSpinBox();
        recordingDurationSpinBox->setRange(0, 3600); // محدوده 0 تا 3600 ثانیه (1 ساعت)
        recordingDurationSpinBox->setValue(0); // مقدار پیش‌فرض 0 (یعنی نامحدود)
        recordingDurationSpinBox->setStyleSheet(isDarkTheme ?
                                                    "QSpinBox { background: #3e3e4e; color: #43cea2; border-radius: 8px; padding: 5px; }"
                                                    "QSpinBox::up-button { width: 20px; background: #43cea2; color: #232526; }"
                                                    "QSpinBox::down-button { width: 20px; background: #43cea2; color: #232526; }" :
                                                    "QSpinBox { background: #e0e0e0; color: black; border: 1px solid #333; border-radius: 8px; padding: 5px; }"
                                                    "QSpinBox::up-button { width: 20px;}"
                                                    "QSpinBox::down-button { width: 20px;}");
        recordingDurationSpinBox->setFixedWidth(100);
        toolbar->addWidget(recordingDurationSpinBox);

        toolbar->addSpacing(20);

        toolbar->addWidget(recordLabel);

        toolbar->addStretch();

        // Apply theme to toolbar buttons
        QString toolbarButtonStyle = isDarkTheme ?
                                         "QPushButton { background: #232526; color: #43cea2; border-radius: 12px; font: 18px 'Segoe UI'; min-width: 140px; margin: 6px; }"
                                         "QPushButton:hover { background: #43cea2; color: #232526; }" :
                                         "QPushButton { background: #e0e0e0; color: black; border: 1px solid #333; border-radius: 12px; font: 18px 'Segoe UI'; min-width: 140px; margin: 6px; }"
                                         "QPushButton:hover { background: #333; color: white; }"
                                         "QPushButton:pressed { background: #ccc; }";
        QString recordLabelStyle = isDarkTheme ? "color: gray; font: 26px;" : "color: gray; font: 26px; border: 1px solid #e0e0e0;";

        for (QPushButton *btn : {btnStop, btnRecord, btnSearchUSB}) {
            btn->setStyleSheet(toolbarButtonStyle);
        }
        recordLabel->setStyleSheet(recordLabelStyle);
    }

    void searchUSB() {
        connectionLabel->setText("🔄 Searching...");
        connectionLabel->setStyleSheet(
            isDarkTheme ?
                "QPushButton { background: #232526; color: #f7971e; border-radius: 12px; font: 18px 'Segoe UI'; min-height: 38px; margin: 6px; padding-left: 12px; text-align: left; }" :
                "QPushButton { background: #e0e0e0; color: #333; border: 1px solid #333; border-radius: 12px; font: 18px 'Segoe UI'; min-height: 38px; margin: 6px; padding-left: 12px; text-align: left; }"
            );
        if (usbReceiver && !usbReceiver->isRunning()) {
            usbReceiver->searchUSB();
            usbReceiver->start();
        }
    }

    void startRecording() {
        if (isUSBConnected && usbReceiver) {
            int delay = startDelaySpinBox->value() * 1000; // تبدیل به میلی‌ثانیه
            if (delay > 0) {
                delayTimer->start(delay);
                // می‌تونی یک پیام یا تغییر UI اضافه کنی، مثلاً recordLabel->setText("Delaying...");
            } else {
                actualStartRecording();
            }
        } else {
            QMessageBox::warning(this, "Error", "Recording is only available with USB hardware!");
        }
    }

    void stopRecording() {
        if (isUSBConnected && usbReceiver) {
            usbReceiver->stopRecording();
        } else {
            QMessageBox::warning(this, "Error", "Recording is only available with USB hardware!");
        }
    }

    void actualStartRecording() {
        usbReceiver->startRecording();
        int duration = recordingDurationSpinBox->value() * 1000; // تبدیل به میلی‌ثانیه
        if (duration > 0) {
            durationTimer->start(duration);
        }
    }

    void handleUSBConnection(bool connected) {
        isUSBConnected = connected;
        updateConnectionStatus(connected);
        btnRecord->setEnabled(connected);
        btnStop->setEnabled(connected && usbReceiver && usbReceiver->isRecording());
    }

    void updateConnectionStatus(bool connected) {
        if (connected) {
            connectionLabel->setText("🟢 Connected");
            connectionLabel->setStyleSheet(
                isDarkTheme ?
                    "QPushButton { background: #232526; color: #00A841; border-radius: 12px; font: 18px 'Segoe UI'; min-height: 38px; margin: 6px; padding-left: 12px; text-align: left; }" :
                    "QPushButton { background: #e0e0e0; color: #00A841; border: 1px solid #e0e0e0; border-radius: 12px; font: 18px 'Segoe UI'; min-height: 38px; margin: 6px; padding-left: 12px; text-align: left; }"
                );
        } else {
            connectionLabel->setText("🔴 Disconnected");
            connectionLabel->setStyleSheet(
                isDarkTheme ?
                    "QPushButton { background: #232526; color: #FF3F15; border-radius: 12px; font: 18px 'Segoe UI'; min-height: 38px; margin: 6px; padding-left: 12px; text-align: left; }" :
                    "QPushButton { background: #e0e0e0; color: #FF3F15; border: 1px solid #e0e0e0; border-radius: 12px; font: 18px 'Segoe UI'; min-height: 38px; margin: 6px; padding-left: 12px; text-align: left; }"
                );
        }
    }

    void handleRecordingStarted(const QString &fileName) {
        Q_UNUSED(fileName);
        recordLabel->setStyleSheet(isDarkTheme ? "color: red; font: 26px;" : "color: red; font: 26px;");
        btnRecord->setEnabled(false);
        btnStop->setEnabled(true);
    }

    void handleRecordingStopped(const QString &fileName) {
        Q_UNUSED(fileName);
        recordLabel->setStyleSheet(isDarkTheme ? "color: gray; font: 26px;" : "color: gray; font: 26px;");
        btnRecord->setEnabled(true);
        btnStop->setEnabled(false);
    }
};

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    RegisterPage *registerPage = new RegisterPage();
    registerPage->setWindowTitle("Register");
    registerPage->resize(500, 400);
    registerPage->show();

    QObject::connect(registerPage, &RegisterPage::proceedToLogin, [=]() {
        LoginPage *loginWindow = new LoginPage();
        loginWindow->setWindowTitle("Login");
        loginWindow->resize(600, 500);
        loginWindow->show();
        registerPage->close();

        QObject::connect(loginWindow, &LoginPage::loginConfirmed, [=](int axisMode, int numImus, bool isDarkTheme) {
            IMUInterface *mainWindow = new IMUInterface(axisMode, numImus, isDarkTheme);
            mainWindow->show();
            loginWindow->close();
        });
    });

    return app.exec();
}
