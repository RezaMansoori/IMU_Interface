#include "imusdatapage.h"
#include <QFont>
#include <QDateTime>

IMUsDataPage::IMUsDataPage(QWidget *parent, int numImus, bool isDarkTheme)
    : QWidget(parent),
    numImus(numImus),
    selectedRow(0),
    refreshTimer(new QTimer(this)),
    lastMasterTime(QDateTime::currentMSecsSinceEpoch()),
    isDarkTheme(isDarkTheme)
{
    for (int i = 0; i < numImus; ++i) {
        imuSelection[i + 1] = true; // All rows visible by default
        imuTimestamps.append(QList<qint64>());
        imuDataChanged[i + 1] = false;
        lastBatteryLevel[i + 1] = -1;
        imuCalibrated[i + 1] = false;
        tickLabels[i + 1] = nullptr; // مقدار اولیه برای هر IMU
    }

    battery_icons["critical"] = QPixmap("icons/critical_battery.png");
    battery_icons["low"] = QPixmap("icons/low_battery.png");
    battery_icons["medium"] = QPixmap("icons/medium_battery.png");
    battery_icons["high"] = QPixmap("icons/high_battery.png");
    battery_icons["full"] = QPixmap("icons/full_battery.png");
    tick_icon = QPixmap("icons/tick.png");

    initUI();

    connect(refreshTimer, &QTimer::timeout, this, &IMUsDataPage::refreshTable);
    refreshTimer->start(10);

    applyTheme();
}

void IMUsDataPage::initUI()
{
    mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(5);

    QVBoxLayout *tableBox = new QVBoxLayout();
    tableBox->setContentsMargins(16, 0, 16, 16);

    table = new QTableWidget(numImus, 14);
    table->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    table->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    table->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    table->setSizeAdjustPolicy(QTableWidget::AdjustToContents);
    table->setSelectionMode(QTableWidget::SingleSelection); // Enable single row selection
    table->setSelectionBehavior(QTableWidget::SelectRows); // Select entire rows
    connect(table, &QTableWidget::itemClicked, this, &IMUsDataPage::handleRowSelection); // اتصال سیگنال itemClicked

    QStringList columnInfo = {
        "IMU",
        "🟢\nAcceleration\nx", "🟢\nAcceleration\ny", "🟢\nAcceleration\nz",
        "🟠\nGyroscope\nx", "🟠\nGyroscope\ny", "🟠\nGyroscope\nz",
        "🔵\nQuaternion\nw", "🔵\nQuaternion\nx", "🔵\nQuaternion\ny", "🔵\nQuaternion\nz",
        "🔴\nLinear\nAcceleration\nx", "🔴\nLinear\nAcceleration\ny", "🔴\nLinear\nAcceleration\nz"
    };
    QStringList toolTips = {
        "IMU Info",
        "Acceleration X", "Acceleration Y", "Acceleration Z",
        "Gyroscope X", "Gyroscope Y", "Gyroscope Z",
        "Quaternion W", "Quaternion X", "Quaternion Y", "Quaternion Z",
        "Linear Acceleration X", "Linear Acceleration Y", "Linear Acceleration Z"
    };

    for (int col = 0; col < 14; ++col) {
        QTableWidgetItem *item = new QTableWidgetItem(columnInfo[col]);
        item->setFont(QFont("Segoe UI", 10, QFont::Bold)); // کاهش اندازه فونت به 10pt
        item->setToolTip(toolTips[col]);
        table->setHorizontalHeaderItem(col, item);
    }

    // Initialize table items to avoid creating new ones
    for (int row = 0; row < numImus; ++row) {
        for (int col = 1; col < 14; ++col) {
            QTableWidgetItem *item = new QTableWidgetItem("");
            table->setItem(row, col, item);
        }
    }

    for (int i = 0; i < numImus; ++i) {
        QWidget *cellWidget = new QWidget();
        QHBoxLayout *rowLayout = new QHBoxLayout(cellWidget);
        rowLayout->setContentsMargins(5, 0, 5, 0);
        rowLayout->setSpacing(5);

        QPushButton *btn = new QPushButton(QString::number(i + 1));
        btn->setFixedSize(30, 30);
        btn->setCheckable(true);
        btn->setChecked(true); // Buttons checked by default (rows visible)
        imuButtons.append(btn);
        connect(btn, &QPushButton::toggled, this, [this, i](bool checked) {
            toggleIMUSelection(i + 1, checked);
            if (checked) selectedRow = i;
        });

        QLabel *batteryLabel = new QLabel();
        batteryLabel->setFixedSize(20, 30);
        batteryLabel->setPixmap(getBatteryIcon(100).scaled(20, 20, Qt::KeepAspectRatio, Qt::SmoothTransformation));
        batteryLabels.append(batteryLabel);

        QLabel *percentLabel = new QLabel("100%");
        batteryPercentLabels.append(percentLabel);

        rowLayout->addWidget(btn);
        rowLayout->addWidget(batteryLabel);
        //rowLayout->addWidget(percentLabel);

        rowLayout->addStretch();

        table->setCellWidget(i, 0, cellWidget);
    }

    table->verticalHeader()->setVisible(false);
    table->verticalHeader()->setDefaultSectionSize(70);
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table->horizontalHeader()->setMinimumSectionSize(100); // تنظیم حداقل عرض ستون
    table->horizontalHeader()->setDefaultSectionSize(120); // تنظیم عرض پیش‌فرض ستون
    table->horizontalHeader()->setMinimumHeight(60); // افزایش ارتفاع هدر برای نمایش متن چندخطی
    tableBox->addWidget(table);

    QWidget *combinedWidget = new QWidget();
    QHBoxLayout *combinedLayout = new QHBoxLayout(combinedWidget);
    combinedLayout->setContentsMargins(0, 0, 0, 0);
    QWidget *tableContainer = new QWidget();
    tableContainer->setLayout(tableBox);
    combinedLayout->addWidget(tableContainer, 1);

    mainLayout->addWidget(combinedWidget);
}

void IMUsDataPage::applyTheme()
{
    QString tableStyle = isDarkTheme ?
                             "QTableWidget { background: #232526; color: #e0e5ec; font: 12pt 'Segoe UI'; gridline-color: #393939; border: none; border-radius: 14px; }"
                             "QHeaderView::section { background: #232526; color: #43cea2; font: bold 10pt; border: none; padding: 8px; }" // کاهش فونت هدر به 10pt
                             "QTableWidget::item { border-bottom: 1.5px solid #393939; padding: 6px; }"
                             "QTableWidget::item:selected { background: #185a9d; color: #fff; }" :
                             "QTableWidget { background: #e0e0e0; color: black; font: 12pt 'Segoe UI'; gridline-color: #333; border: 1px solid #333; border-radius: 14px; }"
                             "QHeaderView::section { background: #e0e0e0; color: black; font: bold 10pt; border: 1px solid #333; padding: 8px; }" // کاهش فونت هدر به 10pt
                             "QTableWidget::item { border-bottom: 1.5px solid #333; padding: 6px; }"
                             "QTableWidget::item:selected { background: #333; color: white; }";

    table->setStyleSheet(tableStyle);
    for (int col = 0; col < 14; ++col) {
        QTableWidgetItem *item = table->horizontalHeaderItem(col);
        if (item) {
            item->setForeground(QColor(
                isDarkTheme ?
                    (col == 0 ? "white" :
                         (col < 4 ? "#43cea2" : (col < 7 ? "#f7971e" : (col < 11 ? "#43a1ce" : "#e43c6f"))) )
                            : "black"));
        }
    }

    for (int i = 0; i < batteryPercentLabels.size(); ++i) {
        QLabel *label = batteryPercentLabels[i];
        label->setStyleSheet(QString("QLabel { color: %1; font: 9pt 'Segoe UI'; background: none; }").arg(getBatteryColor(100)));
    }
}

QPixmap IMUsDataPage::getBatteryIcon(int level)
{
    if (level <= 20) return battery_icons["critical"];
    else if (level <= 40) return battery_icons["low"];
    else if (level <= 60) return battery_icons["medium"];
    else if (level <= 80) return battery_icons["high"];
    else return battery_icons["full"];
}

QString IMUsDataPage::getBatteryColor(int level)
{
    if (level <= 20) return "#FF3F15";
    else if (level <= 40) return "#FCC419";
    else if (level <= 60) return "#F1FC00";
    else if (level <= 80) return "#A2D94F";
    else return "#00A841";
}

void IMUsDataPage::updateIMUData(const IMU &data)
{
    imuData[data.id] = data;
    imuDataChanged[data.id] = true;
    imuTimestamps[data.id - 1].append(QDateTime::currentMSecsSinceEpoch());

    if (data.id - 1 < batteryLabels.size() && data.id - 1 < batteryPercentLabels.size()) {
        if (lastBatteryLevel[data.id] != data.battery) {
            batteryLabels[data.id - 1]->setPixmap(getBatteryIcon(data.battery).scaled(20, 20, Qt::KeepAspectRatio, Qt::SmoothTransformation));
            batteryPercentLabels[data.id - 1]->setText(QString("%1%").arg(data.battery));
            batteryPercentLabels[data.id - 1]->setStyleSheet(QString("QLabel { color: %1; font: 9pt 'Segoe UI'; background: none; }").arg(getBatteryColor(data.battery)));
            lastBatteryLevel[data.id] = data.battery;
        }
    }
}

void IMUsDataPage::toggleIMUSelection(int imuId, bool checked)
{
    imuSelection[imuId] = checked;
    int row = imuId - 1;
    if (row >= 0 && row < table->rowCount()) {
        for (int col = 1; col < table->columnCount(); ++col) {
            QTableWidgetItem *item = table->item(row, col);
            if (item) {
                item->setFlags(checked ? (Qt::ItemIsSelectable | Qt::ItemIsEnabled) : Qt::NoItemFlags);
                item->setText(checked && imuData.contains(imuId) ? QString::number(imuData[imuId].acc[0], 'f', 3) : "");
            }
        }
        // Ensure the first column (with buttons) remains visible
        table->setCellWidget(row, 0, table->cellWidget(row, 0)); // Re-attach widget to ensure visibility
        if (checked && imuData.contains(imuId)) {
            updateIMUData(imuData[imuId]);
        }
    }
}

void IMUsDataPage::refreshTable()
{
    table->blockSignals(true);
    for (int row = 0; row < numImus; ++row) {
        int imuId = row + 1;
        if (imuSelection.value(imuId, false) && imuData.contains(imuId) && imuDataChanged[imuId]) {
            IMU data = imuData[imuId];
            table->item(row, 1)->setText(QString::number(data.acc[0], 'f', 3));
            table->item(row, 2)->setText(QString::number(data.acc[1], 'f', 3));
            table->item(row, 3)->setText(QString::number(data.acc[2], 'f', 3));
            table->item(row, 4)->setText(QString::number(data.gyro[0], 'f', 3));
            table->item(row, 5)->setText(QString::number(data.gyro[1], 'f', 3));
            table->item(row, 6)->setText(QString::number(data.gyro[2], 'f', 3));
            table->item(row, 7)->setText(QString::number(data.quat[0], 'f', 3));
            table->item(row, 8)->setText(QString::number(data.quat[1], 'f', 3));
            table->item(row, 9)->setText(QString::number(data.quat[2], 'f', 3));
            table->item(row, 10)->setText(QString::number(data.quat[3], 'f', 3));
            table->item(row, 11)->setText(QString::number(data.lacc[0], 'f', 3));
            table->item(row, 12)->setText(QString::number(data.lacc[1], 'f', 3));
            table->item(row, 13)->setText(QString::number(data.lacc[2], 'f', 3));
            imuDataChanged[imuId] = false;
        }
    }
    table->blockSignals(false);

    qint64 now = QDateTime::currentMSecsSinceEpoch();
    double interval = (now - lastMasterTime) / 1000.0;
    if (interval > 0) {
        double hz = qRound(1.0 / interval * 10.0) / 10.0;
        QStackedWidget *stacked = qobject_cast<QStackedWidget*>(parentWidget());
        if (stacked) {
            QWidget *mainWindow = stacked->parentWidget();
            if (mainWindow) {
                QLabel *hzLabel = mainWindow->findChild<QLabel*>("masterHzLabel");
                if (hzLabel) {
                    hzLabel->setText(QString("Hz: %1").arg(hz));
                }
            }
        }
    }
    lastMasterTime = now;
}

void IMUsDataPage::handleRowSelection(QTableWidgetItem *item)
{
    if (item) {
        selectedRow = item->row();
        // فقط سطر انتخاب‌شده را به‌روزرسانی می‌کنیم، بدون تغییر حالت دکمه‌های سایر سطرها
    }
}

double IMUsDataPage::calculateHertz(int idx)
{
    if (idx < 0 || idx >= imuTimestamps.size()) return 0.0;
    const QList<qint64> &timestamps = imuTimestamps[idx];
    if (timestamps.size() < 2) return 0.0;
    double duration = (timestamps.last() - timestamps.first()) / 1000.0;
    if (duration == 0.0) return 0.0;
    return qRound((timestamps.size() - 1) / duration * 10.0) / 10.0;
}

void IMUsDataPage::selectIMU(int imuId)
{
    int row = imuId - 1; // سطرها از 0 شروع می‌شن
    if (row >= 0 && row < table->rowCount()) {
        table->selectRow(row); // انتخاب سطر IMU
        table->scrollToItem(table->item(row, 0)); // اسکرول به سطر
    }
}

void IMUsDataPage::onCalibrationCompleted(int imuId) {
    if (imuId >= 1 && imuId <= numImus) {
        imuCalibrated[imuId] = true;
        int row = imuId - 1;
        if (row >= 0 && row < table->rowCount()) {
            QWidget *cellWidget = table->cellWidget(row, 0);
            QHBoxLayout *rowLayout = qobject_cast<QHBoxLayout*>(cellWidget->layout());
            if (rowLayout) {
                // چک کن اگر تیک قبلا اضافه نشده
                if (!tickLabels.value(imuId, nullptr)) {
                    QLabel *tickLabel = new QLabel();
                    tickLabel->setFixedSize(20, 20);
                    tickLabel->setPixmap(tick_icon.scaled(20, 20, Qt::KeepAspectRatio, Qt::SmoothTransformation));
                    rowLayout->insertWidget(rowLayout->count() - 1, tickLabel); // اضافه کردن کنار باتری، قبل از stretch
                    tickLabels[imuId] = tickLabel; // ذخیره در مپ برای چک بعدی
                }
            }
        }
    }
}
