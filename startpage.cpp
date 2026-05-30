#include "startpage.h"
#include <QMessageBox>
#include <QStackedWidget>
#include <QFileDialog>
#include <QMenu>
#include <bodywidget.h>

StartPage::StartPage(QWidget *parent, bool isDarkTheme, USBReceiver *usbReceiver)
    : QWidget(parent),
    numImus(18), // Added for dynamic IMU count
    referenceQuat(QQuaternion()),
    isUSBConnected(false),
    usbReceiver(usbReceiver),
    vbox(new QVBoxLayout(this)),
    mainLayout(nullptr),
    leftWidget(nullptr),
    lblCount(nullptr),
    countdownTimer(nullptr),
    countValue(0),
    countStage(1),
    plotter(nullptr),
    isDarkTheme(isDarkTheme)
{
    displayToInternal["Pelvis"]        = "Pelvis";
    displayToInternal["Back"]          = "Back";
    displayToInternal["Head"]          = "Head";

    displayToInternal["Right Arm"]     = "ArmR";
    displayToInternal["Right forearm"] = "ForearmR";
    displayToInternal["Left Arm"]     = "ArmL";
    displayToInternal["Left forearm"] = "ForearmL";

    displayToInternal["Right Hand"]    = "HandR";
    displayToInternal["Left Hand"]    = "HandL";

    displayToInternal["Right Foot"]    = "FootR";
    displayToInternal["Left Foot"]    = "FootL";

    displayToInternal["Right Thigh"]   = "ThighR";
    displayToInternal["Right Shank"]   = "ShankR";
    displayToInternal["Left Thigh"]   = "ThighL";
    displayToInternal["Left Shank"]   = "ShankL";

    // Initialize settings
    settings = {"Pelvis", "Right Arm", "Head", "Back", "Right Hand", "Right forearm", "Right Shank", "Right Tight"};
    for (const QString &setting : settings) {
        comboValues[setting] = "1";
    }

    // Add main layout to vbox (بدون toolbar)
    mainLayout = new QHBoxLayout();
    buildRight3DView();
    leftWidget = new QWidget();
    mainLayout->addWidget(leftWidget, 2);
    vbox->addLayout(mainLayout);

    // Countdown timer
    countdownTimer = new QTimer(this);
    connect(countdownTimer, &QTimer::timeout, this, &StartPage::updateCountdown);

    // Show initial step
    showStep1();

    // Connect USBReceiver signals
    if (usbReceiver) {
        //connect(usbReceiver, &USBReceiver::error, this, &StartPage::handleUSBError);
        connect(usbReceiver, &USBReceiver::data, this, &StartPage::handleUSBData);
        connect(usbReceiver, &USBReceiver::connectionStatus, this, &StartPage::handleUSBConnection);
        connect(usbReceiver, &USBReceiver::data, this, &StartPage::calibratePelvis);
        connect(usbReceiver, &USBReceiver::data, this, [this](const IMU &data) {
            emit imuDataReceived(data);
            updateModel(data);
        });
        connect(usbReceiver->findChild<RecorderService*>(), &RecorderService::recordingStarted, this, &StartPage::recordingStarted);
        connect(usbReceiver->findChild<RecorderService*>(), &RecorderService::recordingStopped, this, &StartPage::recordingStopped);
        isUSBConnected = usbReceiver->isRunning();
    }

    // Apply theme
    applyTheme();
}


void StartPage::applyTheme()
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
                             "QLabel { color: #f8f8f8; }" :
                             "QLabel { color: black; }";
    QString comboBoxStyle = isDarkTheme ?
                                "QComboBox { background: #3e3e4e; color: #43cea2; border-radius: 8px; font: 15px 'Segoe UI'; }" :
                                "QComboBox { background: #e0e0e0; color: black; border: 1px solid #333; border-radius: 8px; font: 15px 'Segoe UI'; }"
                                "QComboBox QAbstractItemView { background: #ffffff; color: black; selection-background-color: #c0c0c0; }";

    setStyleSheet(widgetStyle);

    // Apply to plotter
    plotter->setStyleSheet(isDarkTheme ? "background: #3e3e4e; border-radius: 5px;" : "background: #f5f5f5; border: 1px solid #333; border-radius: 5px;");

    // Rebuild current step to apply theme to left frame
    if (countStage == 1) showStep1();
    else if (countStage == 2) showStep2();
    else showStep3();
}

void StartPage::clearLeft()
{
    mainLayout->removeWidget(leftWidget);
    leftWidget->deleteLater();
    leftWidget = nullptr;
}

void StartPage::buildLeftFrame(const QString &title, const QList<QWidget*> &bodyWidgets, const QList<QPushButton*> &buttons)
{
    clearLeft();
    leftWidget = new QWidget();
    QVBoxLayout *lay = new QVBoxLayout(leftWidget);

    QLabel *titleLbl = new QLabel(title);
    titleLbl->setStyleSheet(isDarkTheme ?
                                "font: 24px 'Segoe UI'; color: #43cea2; margin-bottom: 16px;" :
                                "font: 24px 'Segoe UI'; color: #333; margin-bottom: 16px;");
    lay->addWidget(titleLbl);

    for (QWidget *w : bodyWidgets) {
        lay->addWidget(w);
    }

    lblCount = new QLabel("");
    lblCount->setStyleSheet(isDarkTheme ?
                                "font: 60px 'Segoe UI'; color: #f7971e; qproperty-alignment: AlignCenter;" :
                                "font: 60px 'Segoe UI'; color: #333; qproperty-alignment: AlignCenter;");
    lay->addWidget(lblCount);

    QHBoxLayout *btnBar = new QHBoxLayout();
    for (QPushButton *b : buttons) {
        b->setStyleSheet(
            isDarkTheme ?
                "QPushButton { background: #3e3e4e; color: #43cea2; border-radius: 12px; min-height: 38px; font: 16px 'Segoe UI'; }"
                "QPushButton:pressed { background: #4e4e5e; }"
                "QPushButton:hover { background: #43cea2; color: #232526; }" :
                "QPushButton { background: #e0e0e0; color: black; border: 1px solid #333; border-radius: 12px; min-height: 38px; font: 16px 'Segoe UI'; }"
                "QPushButton:hover { background: #333; color: white; }"
                "QPushButton:pressed { background: #ccc; }"
            );
        btnBar->addWidget(b);
    }
    lay->addLayout(btnBar);
    lay->addStretch();

    mainLayout->addWidget(leftWidget, 2);
}

void StartPage::showStep1()
{
    // اگر اولین بار وارد شویم:
    if (!comboRegion) {
        comboRegion = new QComboBox();
        comboRegion->addItem("Full Body");
        comboRegion->addItem("Half Body");
        comboRegion->addItem("Upper Body");
        comboRegion->addItem("Lower Body");

        comboRegion->setMinimumWidth(140);
        comboRegion->setStyleSheet(
            "QComboBox { background:#000; color:#fff; border:1px solid #555; border-radius:6px; padding:4px; }"
            "QComboBox QAbstractItemView { background:#000; color:#fff; }"
            );
    }

    // ساخت لیبل region
    QLabel *regionLabel = new QLabel("Body Region:");
    regionLabel->setStyleSheet(
        isDarkTheme ?
            "font: 16px 'Segoe UI'; color:#f8f8f8;" :
            "font: 16px 'Segoe UI'; color:black;"
        );

    // ساخت گرید تنظیمات فقط یک بار
    if (!settingsGrid) {
        settingsGrid = new QGridLayout();
    }

    // ایجاد wrapper
    QWidget *wrapper = new QWidget();
    QVBoxLayout *outer = new QVBoxLayout(wrapper);

    // ردیف region
    QHBoxLayout *rowRegion = new QHBoxLayout();
    rowRegion->addWidget(regionLabel);
    rowRegion->addWidget(comboRegion);
    rowRegion->addStretch();
    outer->addLayout(rowRegion);

    // گرید لیست تنظیمات
    QWidget *gridHolder = new QWidget();
    gridHolder->setLayout(settingsGrid);
    outer->addWidget(gridHolder);

    // ساخت دکمه Next
    QPushButton *btnNext = new QPushButton("Next ➜");
    connect(btnNext, &QPushButton::clicked, this, &StartPage::saveAndNext);

    QHBoxLayout *btnRow = new QHBoxLayout();
    btnRow->addStretch();
    btnRow->addWidget(btnNext);
    outer->addLayout(btnRow);

    // قرار دادن صفحه در پنل سمت چپ
    buildLeftFrame("Step 1: Assign IMUs", {wrapper}, {btnNext});

    // اتصال سیگنال فقط یکبار
    static bool connected = false;
    if (!connected) {
        connected = true;
        connect(comboRegion, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, [this]() {
                    updateJointList(comboRegion->currentText());
                });
    }

    // اجرای اولیه — Full Body
    updateJointList(comboRegion->currentText());
}

void StartPage::updateJointList(const QString &region)
{
    // تنظیم لیست‌ها
    QStringList halfBodySettings = {
        "Head", "Pelvis", "Back", "Right Arm",
        "Right forearm", "Right Hand", "Right Shank", "Right Thigh", "Right Foot"
    };

    QStringList upperSettings = {
        "Head", "Pelvis" , "Back", "Right Arm", "Right forearm", "Right Hand", "Left Arm", "Left forearm" ,"Left Hand"
    };

    QStringList lowerSettings = {
        "Pelvis", "Right Shank", "Right Thigh", "Right Foot","Left Shank", "Left Thigh", "Left Foot"
    };

    QStringList fullSettings = {
        "Head", "Pelvis",  "Back", "Right Arm", "Right forearm", "Right Hand", "Right Shank", "Right Thigh", "Right Foot", "Left Arm", "Left forearm", "Left Hand", "Left Shank", "Left Thigh", "Left Foot"
    };

    if (region == "Upper Body")
        newSettings = upperSettings;
    else if (region == "Lower Body")
        newSettings = lowerSettings;
    else if (region == "Full Body")
        newSettings = fullSettings;
    else
        newSettings = halfBodySettings;

    // حفظ انتخاب‌های فعلی
    for (auto it = combos.begin(); it != combos.end(); ++it) {
        if (it.value())
            comboValues[it.key()] = it.value()->currentText();
    }

    // پاک کردن گرید قبلی
    while (QLayoutItem *item = settingsGrid->takeAt(0)) {
        if (item->widget())
            item->widget()->deleteLater();
        delete item;
    }

    combos.clear(); // کامبوهای جدید بعداً اضافه می‌شوند

    // ساخت گرید جدید بر اساس newSettings
    int row = 0;
    for (const QString &setting : newSettings) {

        QLabel *lbl = new QLabel(setting);
        lbl->setStyleSheet(isDarkTheme ?
                               "font: 16px 'Segoe UI'; color:#f8f8f8;" :
                               "font: 16px 'Segoe UI'; color:black;");

        QComboBox *cb = new QComboBox();

        for (int n = 1; n <= numImus; ++n)
            cb->addItem(QString::number(n));

        // بازیابی انتخاب قبلی
        if (comboValues.contains(setting))
            cb->setCurrentText(comboValues[setting]);

        combos[setting] = cb;

        // حفظ خودکار انتخاب‌های جدید بعد از تغییر کاربر
        connect(cb, QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, [this, setting, cb]() {
                    comboValues[setting] = cb->currentText();
                });

        settingsGrid->addWidget(lbl, row, 0);
        settingsGrid->addWidget(cb, row, 1);
        row++;
        cb->setStyleSheet(
            "QComboBox {"
            "    background-color: #000000;"
            "    color: #ffffff;"
            "    border: 1px solid #555;"
            "    border-radius: 6px;"
            "    padding: 4px;"
            "}"
            "QComboBox QAbstractItemView {"
            "    background-color: #000000;"
            "    color: #ffffff;"
            "    selection-background-color: #333333;"
            "}"
            );
    }

}

void StartPage::showStep2()
{
    QList<QWidget*> body;
    QLabel *msg = new QLabel("Selected IMUs:");
    msg->setStyleSheet(isDarkTheme ? "font: 18px 'Segoe UI'; color: #f8f8f8;" : "font: 18px 'Segoe UI'; color: black;");
    body << msg;

    QWidget *contentWidget = new QWidget();
    QHBoxLayout *contentLayout = new QHBoxLayout(contentWidget);

    QVBoxLayout *imuListLayout = new QVBoxLayout();
    QMap<QString, QLabel*> imuLabels;
    for (const QString &setting : newSettings) {
        QString imuIdStr = comboValues[setting];
        QLabel *imuLabel = new QLabel(QString("%1: IMU %2").arg(setting).arg(imuIdStr));
        imuLabel->setStyleSheet(isDarkTheme ? "font: 16px 'Segoe UI'; color: #43cea2;" : "font: 16px 'Segoe UI'; color: #333;");
        imuLabel->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(imuLabel, &QLabel::customContextMenuRequested, this, [this, imuIdStr](const QPoint &pos) {
            showContextMenu(pos, imuIdStr.toInt());
        });
        imuListLayout->addWidget(imuLabel);
        imuLabels[setting] = imuLabel;
    }
    contentLayout->addLayout(imuListLayout);

    BodyWidget *bodyModel = new BodyWidget();
    bodyModel->setMinimumSize(300, 300);
    contentLayout->addWidget(bodyModel);

    // Map joint names to their corresponding IMU settings and BodyWidget joint indices.
    QMap<int, QString> jointIndexToSetting;
    // Indices are based on the jointPointsPercent in BodyWidget, which are
    // mapped from the original pixel points.
    jointIndexToSetting[15] = "ThighR";
    jointIndexToSetting[0] = "Pelvis";
    jointIndexToSetting[18] = "ArmR";
    jointIndexToSetting[19] = "HandR";
    jointIndexToSetting[4] = "ForearmR";
    jointIndexToSetting[5] = "ShankR";
    jointIndexToSetting[8] = "Head";
    jointIndexToSetting[8] = "Back";

    // Connect the BodyWidget's jointSelected signal.
    connect(bodyModel, &BodyWidget::jointSelected, this, [this, imuLabels, jointIndexToSetting](int index){
        QString setting = jointIndexToSetting.value(index);

        // Update the style for all IMU labels based on the selected joint.
        for (const QString &key : imuLabels.keys()) {
            if (key == setting) {
                imuLabels[key]->setStyleSheet(isDarkTheme ?
                                                  "font: bold 16px 'Segoe UI'; color: #43cea2;" :
                                                  "font: bold 16px 'Segoe UI'; color: #333;");
            } else {
                imuLabels[key]->setStyleSheet(isDarkTheme ?
                                                  "font: 16px 'Segoe UI'; color: #43cea2;" :
                                                  "font: 16px 'Segoe UI'; color: #333;");
            }
        }
    });

    body << contentWidget;

    QPushButton *btnPrevious = new QPushButton("⬅️ Previous");
    QPushButton *btnNext = new QPushButton("➡️ Next");

    QString buttonStyle = isDarkTheme ?
                              "QPushButton { background: #3e3e4e; color: #43cea2; border-radius: 12px; min-height: 38px; font: 16px 'Segoe UI'; }"
                              "QPushButton:pressed { background: #4e4e5e; }"
                              "QPushButton:hover { background: #43cea2; color: #232526; }" :
                              "QPushButton { background: #e0e0e0; color: black; border: 1px solid #333; border-radius: 12px; min-height: 38px; font: 16px 'Segoe UI'; }"
                              "QPushButton:hover { background: #333; color: white; }"
                              "QPushButton:pressed { background: #ccc; }";
    btnPrevious->setStyleSheet(buttonStyle);
    btnNext->setStyleSheet(buttonStyle);

    connect(btnPrevious, &QPushButton::clicked, this, &StartPage::showStep1);
    connect(btnNext, &QPushButton::clicked, this, &StartPage::showStep3);

    buildLeftFrame("Step 2: Review Selected IMUs", body, {btnPrevious, btnNext});
}

void StartPage::showStep3()
{
    QList<QWidget*> body;
    QLabel *msg = new QLabel("Simulation is ready. Press the button to start.");
    msg->setStyleSheet(isDarkTheme ? "font: 18px 'Segoe UI'; color: #f8f8f8;" : "font: 18px 'Segoe UI'; color: black;");
    body << msg;

    QPushButton *btnPrevious = new QPushButton("Previous");
    QPushButton *btnStartSim = new QPushButton("Start Simulation");

    QString buttonStyle = isDarkTheme ?
                              "QPushButton { background: #3e3e4e; color: #43cea2; border-radius: 12px; min-height: 38px; font: 16px 'Segoe UI'; }"
                              "QPushButton:pressed { background: #4e4e5e; }"
                              "QPushButton:hover { background: #43cea2; color: #232526; }" :
                              "QPushButton { background: #e0e0e0; color: black; border: 1px solid #333; border-radius: 12px; min-height: 38px; font: 16px 'Segoe UI'; }"
                              "QPushButton:hover { background: #333; color: white; }"
                              "QPushButton:pressed { background: #ccc; }";

    btnStartSim->setStyleSheet(buttonStyle);
    btnPrevious->setStyleSheet(buttonStyle);

    connect(btnStartSim, &QPushButton::clicked, this, &StartPage::startSimulationRequested);
    connect(btnPrevious, &QPushButton::clicked, this, &StartPage::showStep2);

    buildLeftFrame("Step 3: Start Simulation", body, {btnPrevious, btnStartSim});
}

void StartPage::showContextMenu(const QPoint &pos, int imuId)
{
    QMenu contextMenu(this);
    QAction *actionTable = new QAction("Show in Table", this);
    QAction *actionChart = new QAction("Show in Chart", this);
    QAction *actionCalibrate = new QAction("Calibrate IMU", this);

    connect(actionTable, &QAction::triggered, this, [this, imuId]() { emit showInTableRequested(imuId); });
    connect(actionChart, &QAction::triggered, this, [this, imuId]() { emit showInChartRequested(imuId); });
    connect(actionCalibrate, &QAction::triggered, this, [this, imuId]() { emit calibrateIMURequested(imuId); });

    contextMenu.addAction(actionTable);
    contextMenu.addAction(actionChart);
    contextMenu.addAction(actionCalibrate);

    QString menuStyle = isDarkTheme ?
                            "QMenu { background: #232526; color: #43cea2; border: 1px solid #414345; border-radius: 8px; }"
                            "QMenu::item { background: #232526; color: #43cea2; padding: 5px 20px; }"
                            "QMenu::item:selected { background: #43cea2; color: #232526; }" :
                            "QMenu { background: #ffffff; color: black; border: 1px solid #333; border-radius: 8px; }"
                            "QMenu::item { background: #ffffff; color: black; padding: 5px 20px; }"
                            "QMenu::item:selected { background: #333; color: white; }";
    contextMenu.setStyleSheet(menuStyle);

    QLabel *senderLabel = qobject_cast<QLabel*>(sender());
    if (senderLabel) {
        contextMenu.exec(senderLabel->mapToGlobal(pos));
    }
}

void StartPage::startCountdown()
{
    countStage = 1;
    countValue = 3;
    lblCount->setText(QString::number(countValue));
    countdownTimer->start(1000);
}

void StartPage::updateCountdown()
{
    countValue--;
    if (countValue >= 0) {
        lblCount->setText(QString::number(countValue));
    } else {
        if (countStage == 1) {
            countStage = 2;
            countValue = 5;
            lblCount->setText(QString::number(countValue));
        } else {
            lblCount->setText("✅");
            countdownTimer->stop();
        }
    }
}
// helper: تبدیل نام نمایش به یک کلید امن (بدون فاصله یا کاراکترهای غیرمجاز)
static QString sanitizeKey(const QString &name) {
    QString key = name;
    key.remove(' ');  // حذف فاصله‌ها

    // حذف تمام کاراکترهای غیر الفانومریک
    QRegularExpression rx("[^A-Za-z0-9_]");
    key.remove(rx);

    return key;
}


#include <QDebug>
void StartPage::saveAndNext() {
    QSet<int> uniqueValues;
    for (const QString &setting : newSettings) {
        // 1) اطمینان از وجود combo برای این setting
        if (!combos.contains(setting) || combos[setting] == nullptr) {
            qDebug() << "saveAndNext: combo for setting not found or null:" << setting;
            QMessageBox::warning(this, tr("Missing Input"),
                                 tr("Combo for '%1' not found. Please re-open the Assign IMUs step.").arg(setting));
            return;
        }

        // 2) خواندن مقدار و بررسی اعتبار عددی
        QString valueStr = combos[setting]->currentText().trimmed();
        bool ok = false;
        int val = valueStr.toInt(&ok);
        if (!ok) {
            qDebug() << "saveAndNext: invalid number for" << setting << ":" << valueStr;
            QMessageBox::warning(this, tr("Invalid Input"),
                                 tr("Invalid IMU ID for '%1'. Please choose a number.").arg(setting));
            return;
        }

        // 3) اعتبارسنجی یکتا بودن
        if (uniqueValues.contains(val)) {
            qDebug() << "saveAndNext: duplicate IMU id" << val << "for" << setting;
            QMessageBox::warning(this, tr("Invalid Input"),
                                 tr("Each IMU must have a unique ID. Duplicate: %1").arg(val));
            return;
        }

        // 4) ذخیره و اضافه به مجموعهٔ یکتاها
        comboValues[setting] = QString::number(val);
        uniqueValues.insert(val);
    }


    combos.clear();
    QMap<QString, QVariant> selectedImus;
    QSet<int> usedIds;

    for (const QString &displayName : newSettings) {
        int id = -1;
        bool ok = false;

        // 1) Prefer the live combo (visible) if exists
        if (combos.contains(displayName) && combos[displayName]) {
            QString txt = combos[displayName]->currentText().trimmed();
            id = txt.toInt(&ok);
        }

        // 2) Fallback to previously stored comboValues (if live combo missing)
        if (!ok) {
            QString stored = comboValues.value(displayName, "");
            id = stored.toInt(&ok);
        }

        // 3) If still not ok, skip this setting (or you can choose a default)
        if (!ok) {
            qDebug() << "saveAndNext: no valid IMU id for" << displayName << "- skipping";
            continue;
        }

        // 4) Validate uniqueness (optional: show warning and return if duplicate)
        if (usedIds.contains(id)) {
            QMessageBox::warning(this, tr("Invalid Input"),
                                 tr("Each IMU must have a unique ID. Duplicate: %1").arg(id));
            return;
        }
        usedIds.insert(id);

        // 5) Map display name to internal key (use displayToInternal if available)
        QString internalKey;
        if (this->property("displayToInternal").isValid()) {
            // if you stored displayToInternal as a member QMap, prefer that:
            // assume displayToInternal is a QMap<QString,QString> member
        }
        if (displayToInternal.contains(displayName)) {
            internalKey = displayToInternal[displayName];
        } else {
            internalKey = sanitizeKey(displayName);
        }

        // 6) store
        selectedImus[internalKey] = QVariant(id);

        // 7) persist selection in comboValues
        comboValues[displayName] = QString::number(id);
    }

    // now use selectedImus as before
    if (parent()) parent()->setProperty("selectedImus", QVariant::fromValue(selectedImus));

    emit imusSelected(selectedImus);
    showStep2();
}


void StartPage::calibratePelvis(const IMU &data)
{
    if (countStage == 2 && countValue > 0 && data.id == comboValues["Pelvis"].toInt()) {
        referenceQuat = QQuaternion(data.quat[0], data.quat[1], data.quat[2], data.quat[3]).normalized();
    }
}

void StartPage::buildRight3DView()
{
    QVBoxLayout *right = new QVBoxLayout();
    QLabel *label = new QLabel("<span style='font:18px \"Segoe UI\"; color:" + QString(isDarkTheme ? "#43cea2" : "black") + ";'>3D Model Viewer</span>");
    right->addWidget(label);

    plotter = new Object3D();
    right->addWidget(plotter, 1);

    QHBoxLayout *bar = new QHBoxLayout();
    QPushButton *btnReset = new QPushButton("🔄 Reset View");
    connect(btnReset, &QPushButton::clicked, this, &StartPage::resetView);
    QPushButton *btnTurn = new QPushButton("⟳ Rotate");
    connect(btnTurn, &QPushButton::clicked, this, &StartPage::rotate);
    QPushButton *btnTPose = new QPushButton("🧍 T-Pose");
    connect(btnTPose, &QPushButton::clicked, this, &StartPage::toTPose);

    for (QPushButton *b : {btnReset, btnTurn, btnTPose}) {
        b->setStyleSheet(
            isDarkTheme ?
                "QPushButton { background: #3e3e4e; color: #43cea2; border-radius: 12px; min-height: 38px; font: 16px 'Segoe UI'; }"
                "QPushButton:pressed { background: #4e4e5e; }"
                "QPushButton:hover { background: #43cea2; color: #232526; }" :
                "QPushButton { background: #e0e0e0; color: black; border: 1px solid #333; border-radius: 12px; min-height: 38px; font: 16px 'Segoe UI'; }"
                "QPushButton:hover { background: #333; color: white; }"
                "QPushButton:pressed { background: #ccc; }"
            );
        bar->addWidget(b);
    }

    right->addLayout(bar);
    mainLayout->addLayout(right, 3);
}

void StartPage::rotate()
{
    QQuaternion currentRotation = plotter->getRotation();
    QQuaternion rotationDelta = QQuaternion::fromAxisAndAngle(0, 1, 0, 10);
    plotter->setRotation(currentRotation * rotationDelta);
}

void StartPage::toTPose()
{
    QMessageBox::warning(this, "Error", "T-Pose functionality not implemented in this placeholder.");
}

void StartPage::resetView()
{
    plotter->setRotation(QQuaternion());
    plotter->resetCamera();
}
/*
void StartPage::handleUSBError(USBReceiver::USBReceiverErrors code, QString)
{
    QString errorType;
    switch (code) {
    case USBReceiver::USBPortNotFound:
        errorType = "USB Port Not Found";
        break;
    case USBReceiver::USBPortNotOpen:
        errorType = "USB Port Not Open";
        break;
    case USBReceiver::DataReadTimedOut:
        errorType = "Data Read Timed Out";
        break;
    case USBReceiver::DataReadReachMaximumRetry:
        errorType = "Max Retry Reached";
        break;
    case USBReceiver::InvalidData:
        errorType = "Invalid Data";
        break;
    default:
        errorType = "Unknown Error";
    }
    updateConnectionStatus(false);
    //QMessageBox::warning(this, errorType, error);
}
*/
void StartPage::handleUSBData(const IMU &data)
{
    emit imuDataReceived(data);
}

void StartPage::handleUSBConnection(bool connected)
{
    isUSBConnected = connected;
}

void StartPage::updateModel(const IMU &data)
{
    if (data.id == comboValues["Pelvis"].toInt()) {
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
