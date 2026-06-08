#include "postprocesspage.h"
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QInputDialog>
#include <QPrinter>
#include <QPainter>
#include <QDebug>
#include <algorithm>
#include <cmath>
#include <QToolTip>
#include <QApplication>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <QRegularExpression> // برای parsing CSV اگر لازم
QString angleNames[3] = {"Roll", "Pitch", "Yaw"};

PostProcessPage::PostProcessPage(QWidget *parent, bool isDarkTheme)
    : QWidget(parent), isDarkTheme(isDarkTheme), isTrimming(false) {
    QVBoxLayout *layout = new QVBoxLayout(this);

    // Mode selection
    timeRadio = new QRadioButton("Time Mode", this);
    xyRadio = new QRadioButton("XY Mode (Joints Angles)", this);
    modeGroup = new QButtonGroup(this);
    modeGroup->addButton(timeRadio);
    modeGroup->addButton(xyRadio);
    timeRadio->setChecked(true);
    connect(modeGroup, &QButtonGroup::buttonClicked, this, &PostProcessPage::onModeChanged);

    // File combo (shared, populated based on mode)
    fileCombo = new QComboBox(this);
    populateFiles(); // Initially for Time mode
    connect(fileCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &PostProcessPage::loadFile);

    // Wrap time select in widget
    timeSelectWidget = new QWidget(this);
    QHBoxLayout *timeSelectLayout = new QHBoxLayout(timeSelectWidget);
    QLabel *imuLabel = new QLabel("IMU:");
    imuCombo = new QComboBox(this);
    for (int i = 1; i <= 18; ++i) {
        imuCombo->addItem(QString::number(i));
    }
    QLabel *paramLabel = new QLabel("Parameter:");
    paramCombo = new QComboBox(this);
    QStringList params = {"AccX", "AccY", "AccZ", "GyroX", "GyroY", "GyroZ", "QuatW", "QuatX", "QuatY", "QuatZ", "LAccX", "LAccY", "LAccZ"};
    for (const auto& p : params) {
        paramCombo->addItem(p);
    }
    QPushButton *applyTimeBtn = new QPushButton("Apply", this);
    connect(applyTimeBtn, &QPushButton::clicked, this, [this]() {
        loadFile(fileCombo->currentIndex());
        updatePlot();
    });
    timeSelectLayout->addWidget(imuLabel);
    timeSelectLayout->addWidget(imuCombo);
    timeSelectLayout->addWidget(paramLabel);
    timeSelectLayout->addWidget(paramCombo);
    timeSelectLayout->addWidget(applyTimeBtn);

    // Wrap XY select in widget (initially hidden)
    xySelectWidget = new QWidget(this);
    xySelectWidget->setVisible(false);
    QHBoxLayout *xySelectLayout = new QHBoxLayout(xySelectWidget);
    jointNames = {"pelvis", "LHip", "RHip", "SPINE1", "LKNEE", "RKNEE", "SPINE2", "LANCLE", "RANCLE",
                  "SPINE3", "LFOOT", "RFOOT", "NECK", "LCLAVICLE", "RCLAVICLE", "HEAD",
                  "LSHOULDER", "RSHOULDER", "LELBOW", "RELBOW", "LWRIST", "RWRIST", "LHAND", "RHAND"};
    jointXCombo = new QComboBox(this);
    jointYCombo = new QComboBox(this);
    for (const auto& name : jointNames) {
        jointXCombo->addItem(name);
        jointYCombo->addItem(name);
    }
    angleCombo = new QComboBox(this);
    angleCombo->addItems({"Roll", "Pitch", "Yaw"});
    applyXYBtn = new QPushButton("Apply", this);
    connect(applyXYBtn, &QPushButton::clicked, this, [this]() {
        loadFile(fileCombo->currentIndex());
        updatePlot();
    });
    connect(angleCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
        if (xyMode()) {
            loadFile(fileCombo->currentIndex());
            updatePlot();
        }
    });
    xySelectLayout->addWidget(new QLabel("Joint X:", this));
    xySelectLayout->addWidget(jointXCombo);
    xySelectLayout->addWidget(new QLabel("Joint Y:", this));
    xySelectLayout->addWidget(jointYCombo);
    xySelectLayout->addWidget(new QLabel("Angle:", this));
    xySelectLayout->addWidget(angleCombo);
    xySelectLayout->addWidget(applyXYBtn);

    // Add mode radios and selects to layout
    QHBoxLayout *modeLayout = new QHBoxLayout;
    modeLayout->addWidget(timeRadio);
    modeLayout->addWidget(xyRadio);
    layout->addLayout(modeLayout);

    fileLabel = new QLabel("No file selected", this);
    layout->addWidget(fileCombo);
    layout->addWidget(fileLabel);
    layout->addWidget(timeSelectWidget);
    layout->addWidget(xySelectWidget);

    QHBoxLayout *buttonLayout = new QHBoxLayout;
    trimBtn = new QPushButton("Trim", this);
    connect(trimBtn, &QPushButton::clicked, this, [this]() {
        isTrimming = true;
        if (xyMode()) {
            angleXBackup = angleX;
            angleYBackup = angleY;
        } else {
            trimmedTimesBackup = currentTimes;
            trimmedValuesBackup = currentValues;
        }
        chartView->setRubberBand(QChartView::RectangleRubberBand);
    });
    QPushButton *undoBtn = new QPushButton("Undo Trim", this); // Renamed for clarity
    connect(undoBtn, &QPushButton::clicked, this, &PostProcessPage::undoTrim);
    undoFilterBtn = new QPushButton("Undo Filter", this); // New undo filter button
    connect(undoFilterBtn, &QPushButton::clicked, this, &PostProcessPage::undoFilter);
    buttonLayout->addWidget(trimBtn);
    buttonLayout->addWidget(undoBtn);
    buttonLayout->addWidget(undoFilterBtn); // Add the new button
    QPushButton *undoMovingAverageBtn = new QPushButton("Undo Moving Average", this);
    connect(undoMovingAverageBtn, &QPushButton::clicked, this, &PostProcessPage::undoMovingAverage);
    buttonLayout->addWidget(undoMovingAverageBtn);
    filterBtn = new QPushButton("Filter", this);
    connect(filterBtn, &QPushButton::clicked, this, &PostProcessPage::toggleFilterFrame);
    buttonLayout->addWidget(filterBtn);

    filterFrame = new QWidget(this);
    filterFrame->setVisible(false);
    QVBoxLayout *filterLayout = new QVBoxLayout(filterFrame);
    QLabel *typeLabel = new QLabel("Filter Type:");
    typeCombo = new QComboBox(this);
    typeCombo->addItem("Low Pass");
    typeCombo->addItem("High Pass");
    QLabel *cutoffLabel = new QLabel("Cutoff Frequency (Hz):");
    cutoffSlider = new QSlider(Qt::Horizontal, this);
    cutoffSlider->setRange(1, 10000); // Range 0.01 to 100 Hz (scaled by 100 for integer slider)
    cutoffSlider->setValue(100); // Default 1.0 Hz
    cutoffValueLabel = new QLabel("1.00 Hz", this);
    QLabel *orderLabel = new QLabel("Order:");
    orderSlider = new QSlider(Qt::Horizontal, this);
    orderSlider->setRange(1, 10);
    orderSlider->setValue(2);
    orderValueLabel = new QLabel("2", this);
    QLabel *titleLabel = new QLabel("Chart Title:");
    titleEdit = new QLineEdit(this);
    applyBtn = new QPushButton("Apply", this);
    connect(applyBtn, &QPushButton::clicked, this, &PostProcessPage::applyFilter);
    saveBtn = new QPushButton("Save HTML", this);
    connect(saveBtn, &QPushButton::clicked, this, &PostProcessPage::saveToHtml);
    connect(cutoffSlider, &QSlider::valueChanged, this, &PostProcessPage::updateCutoff);
    connect(orderSlider, &QSlider::valueChanged, this, &PostProcessPage::updateOrder);
    filterLayout->addWidget(typeLabel);
    filterLayout->addWidget(typeCombo);
    filterLayout->addWidget(cutoffLabel);
    filterLayout->addWidget(cutoffSlider);
    filterLayout->addWidget(cutoffValueLabel);
    filterLayout->addWidget(orderLabel);
    filterLayout->addWidget(orderSlider);
    filterLayout->addWidget(orderValueLabel);
    filterLayout->addWidget(titleLabel);
    filterLayout->addWidget(titleEdit);
    filterLayout->addWidget(applyBtn);
    filterLayout->addWidget(saveBtn);

    chart = new QChart();
    series = new QLineSeries();
    filteredSeries = new QLineSeries(); // اضافه کردن سری فیلترشده
    movingAverageSeries = new QLineSeries();
    chart->addSeries(series);
    chart->addSeries(filteredSeries); // اضافه کردن سری فیلترشده به نمودار
    chart->addSeries(movingAverageSeries);
    QValueAxis *axisX = new QValueAxis;
    axisX->setTitleText("Time (s)");
    QValueAxis *axisY = new QValueAxis;
    axisY->setTitleText(currentParam);
    chart->addAxis(axisX, Qt::AlignBottom);
    chart->addAxis(axisY, Qt::AlignLeft);
    series->attachAxis(axisX);
    series->attachAxis(axisY);
    filteredSeries->attachAxis(axisX); // اتصال سری فیلترشده به محورها
    filteredSeries->attachAxis(axisY);
    movingAverageSeries->attachAxis(axisX);
    movingAverageSeries->attachAxis(axisY);

    // XY series
    xySeries = new QLineSeries();
    xyFilteredSeries = new QLineSeries();
    xyMovingSeries = new QLineSeries();
    chart->addSeries(xySeries);
    chart->addSeries(xyFilteredSeries);
    chart->addSeries(xyMovingSeries);
    xySeries->attachAxis(axisX);
    xySeries->attachAxis(axisY);
    xyFilteredSeries->attachAxis(axisX);
    xyFilteredSeries->attachAxis(axisY);
    xyMovingSeries->attachAxis(axisX);
    xyMovingSeries->attachAxis(axisY);
    connect(xySeries, &QLineSeries::hovered, this, &PostProcessPage::handlePointHover);
    connect(xyFilteredSeries, &QLineSeries::hovered, this, &PostProcessPage::handlePointHover);
    connect(xyMovingSeries, &QLineSeries::hovered, this, &PostProcessPage::handlePointHover);

    // Hide XY series initially
    xySeries->setVisible(false);
    xyFilteredSeries->setVisible(false);
    xyMovingSeries->setVisible(false);

    chartView = new QChartView(chart, this);
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->setInteractive(true); // فعال کردن تعامل
    chartView->setRubberBand(QChartView::RectangleRubberBand); // زوم با انتخاب مستطیلی
    chart->setAnimationOptions(QChart::SeriesAnimations); // انیمیشن برای تغییرات
    chartView->setMouseTracking(true); // فعال کردن ردیابی ماوس

    movingAverageBtn = new QPushButton("Moving Average", this);
    connect(movingAverageBtn, &QPushButton::clicked, this, &PostProcessPage::toggleMovingAverageFrame);
    buttonLayout->addWidget(movingAverageBtn);

    movingAverageFrame = new QWidget(this);
    movingAverageFrame->setVisible(false);
    QVBoxLayout *movingAverageLayout = new QVBoxLayout(movingAverageFrame);
    QLabel *windowSizeLabel = new QLabel("Window Size:");
    windowSizeSlider = new QSlider(Qt::Horizontal, this);
    windowSizeSlider->setRange(1, 100); // محدوده ۱ تا ۱۰۰ نمونه
    windowSizeSlider->setValue(10); // مقدار پیش‌فرض ۱۰
    windowSizeValueLabel = new QLabel("10", this);
    applyMovingBtn = new QPushButton("Apply", this);
    connect(applyMovingBtn, &QPushButton::clicked, this, &PostProcessPage::applyMovingAverage);
    connect(windowSizeSlider, &QSlider::valueChanged, this, &PostProcessPage::updateWindowSize);
    movingAverageLayout->addWidget(windowSizeLabel);
    movingAverageLayout->addWidget(windowSizeSlider);
    movingAverageLayout->addWidget(windowSizeValueLabel);
    movingAverageLayout->addWidget(applyMovingBtn);
    layout->addWidget(movingAverageFrame);

    // اتصال سیگنال برای نمایش مختصات هنگام هاور
    connect(series, &QLineSeries::hovered, this, &PostProcessPage::handlePointHover);
    connect(filteredSeries, &QLineSeries::hovered, this, &PostProcessPage::handlePointHover);
    connect(movingAverageSeries, &QLineSeries::hovered, this, &PostProcessPage::handlePointHover);

    chartView->setRubberBand(QChartView::NoRubberBand);
    connect(chartView, &QChartView::rubberBandChanged, this, [this](QRectF rect, QPointF from) {
        if (isTrimming) {
            if (rect.isNull()) {
                endTrimming(from);
            } else {
                updateTrimming(from);
            }
        }
    });

    layout->addLayout(buttonLayout);
    layout->addWidget(filterFrame);
    layout->addWidget(chartView);

    paramMap = {
        {"AccX", 1}, {"AccY", 2}, {"AccZ", 3},
        {"GyroX", 4}, {"GyroY", 5}, {"GyroZ", 6},
        {"QuatW", 7}, {"QuatX", 8}, {"QuatY", 9}, {"QuatZ", 10},
        {"LAccX", 11}, {"LAccY", 12}, {"LAccZ", 13}
    };

    QPalette tooltipPalette = QToolTip::palette();
    tooltipPalette.setColor(QPalette::ToolTipText, isDarkTheme ? Qt::white : Qt::black);
    tooltipPalette.setColor(QPalette::ToolTipBase, isDarkTheme ? Qt::darkGray : Qt::white);
    QToolTip::setPalette(tooltipPalette);

    applyTheme();
}

void PostProcessPage::applyTheme() {
    QString widgetStyle = isDarkTheme ?
                              "QWidget { background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #232526, stop:1 #414345); }"
                              "QRadioButton { color: #f8f8f8; }" :
                              "QWidget { background: white; }"
                              "QRadioButton { color: black; }";
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
                                "QComboBox QAbstractItemView { background: #ffffff; color: black; selection-background-color: #c0c0c0; }"
                                "QComboBox QAbstractItemView::item { color: black; }";
    QString lineEditStyle = isDarkTheme ?
                                "QLineEdit { color: #43cea2; font: 15px 'Segoe UI'; }" :
                                "QLineEdit { color: black; font: 15px 'Segoe UI'; }";
    QString spinBoxTextStyle = isDarkTheme ?
                                   "QDoubleSpinBox, QSpinBox { color: #43cea2; }" :
                                   "QDoubleSpinBox, QSpinBox { color: black; }";
    QString sliderStyle = isDarkTheme ?
                              "QSlider::groove:horizontal { border: 1px solid #43cea2; height: 8px; background: #3e3e4e; margin: 2px 0; border-radius: 4px; }"
                              "QSlider::handle:horizontal { background: #43cea2; border: 1px solid #232526; width: 18px; margin: -2px 0; border-radius: 9px; }" :
                              "QSlider::groove:horizontal { border: 1px solid #333; height: 8px; background: #e0e0e0; margin: 2px 0; border-radius: 4px; }"
                              "QSlider::handle:horizontal { background: #333; border: 1px solid #e0e0e0; width: 18px; margin: -2px 0; border-radius: 9px; }";

    // Apply to buttons (member ones)
    trimBtn->setStyleSheet(buttonStyle);
    filterBtn->setStyleSheet(buttonStyle);
    applyBtn->setStyleSheet(buttonStyle);
    saveBtn->setStyleSheet(buttonStyle);
    applyMovingBtn->setStyleSheet(buttonStyle);
    movingAverageBtn->setStyleSheet(buttonStyle);
    undoFilterBtn->setStyleSheet(buttonStyle);

    // Apply to local buttons in constructor (use findChild or make them members; here assuming global covers them, or manual after creation)
    // If undoBtn and undoMovingAverageBtn are local, add in constructor after creation: undoBtn->setStyleSheet(buttonStyle);

    // Radio buttons are now in widgetStyle (global)

    // Apply to combo boxes
    fileCombo->setStyleSheet(comboBoxStyle);
    imuCombo->setStyleSheet(comboBoxStyle);
    paramCombo->setStyleSheet(comboBoxStyle);
    typeCombo->setStyleSheet(comboBoxStyle);
    jointXCombo->setStyleSheet(comboBoxStyle);
    jointYCombo->setStyleSheet(comboBoxStyle);
    angleCombo->setStyleSheet(comboBoxStyle);

    // Apply to labels (members)
    fileLabel->setStyleSheet(labelStyle);
    cutoffValueLabel->setStyleSheet(labelStyle);
    orderValueLabel->setStyleSheet(labelStyle);
    windowSizeValueLabel->setStyleSheet(labelStyle);
    // For local labels like imuLabel, paramLabel, etc., global labelStyle covers them

    // Apply to line edit
    titleEdit->setStyleSheet(lineEditStyle);

    // Apply to sliders
    cutoffSlider->setStyleSheet(sliderStyle);
    orderSlider->setStyleSheet(sliderStyle);
    windowSizeSlider->setStyleSheet(sliderStyle);

    // Apply to frames
    filterFrame->setStyleSheet(isDarkTheme ? "background: #3e3e4e; border-radius: 5px;" : "background: #f5f5f5; border: 1px solid #333; border-radius: 5px;");
    movingAverageFrame->setStyleSheet(isDarkTheme ? "background: #3e3e4e; border-radius: 5px;" : "background: #f5f5f5; border: 1px solid #333; border-radius: 5px;");

    // Apply to chart series colors
    movingAverageSeries->setColor(isDarkTheme ? QColor("#FF0000") : QColor("#8B0000"));
    filteredSeries->setColor(isDarkTheme ? QColor("#00ff00") : QColor("#008000"));
    series->setColor(isDarkTheme ? QColor("#43cea2") : QColor("#000000"));
    // For XY series
    xySeries->setColor(isDarkTheme ? QColor("#43cea2") : QColor("#000000"));
    xyFilteredSeries->setColor(isDarkTheme ? QColor("#00ff00") : QColor("#008000"));
    xyMovingSeries->setColor(isDarkTheme ? QColor("#FF0000") : QColor("#8B0000"));

    // Global stylesheet (includes radio button styles now)
    setStyleSheet(widgetStyle + buttonStyle + labelStyle + comboBoxStyle + lineEditStyle + sliderStyle);

    // Apply to chart view
    chartView->setStyleSheet(isDarkTheme ? "background: #3e3e4e; border-radius: 5px;" : "background: #f5f5f5; border: 1px solid #333; border-radius: 5px;");
}

void PostProcessPage::onModeChanged() {
    // Clean all data and series to prevent state conflicts/crash on switch
    originalTimes.clear();
    currentTimes.clear();
    originalValues.clear();
    currentValues.clear();
    movingAverageValues.clear();
    angleX.clear();
    angleY.clear();
    filteredY.clear();
    movingY.clear();
    series->clear();
    filteredSeries->clear();
    movingAverageSeries->clear();
    xySeries->clear();
    xyFilteredSeries->clear();
    xyMovingSeries->clear();
    fs = 500.0; // Reset fs

    // Hide all series initially
    series->setVisible(false);
    filteredSeries->setVisible(false);
    movingAverageSeries->setVisible(false);
    xySeries->setVisible(false);
    xyFilteredSeries->setVisible(false);
    xyMovingSeries->setVisible(false);

    bool isXY = xyMode();
    timeSelectWidget->setVisible(!isXY);
    xySelectWidget->setVisible(isXY);

    // Reset defaults for Time mode
    if (!isXY) {
        imuCombo->setCurrentIndex(0); // Default IMU 1
        paramCombo->setCurrentIndex(0); // Default AccX
    }

    // Same CSV list for both modes
    populateFiles();
    if (fileCombo->count() > 0) {
        int currentIndex = fileCombo->currentIndex();
        if (currentIndex < 0) currentIndex = 0;
        loadFile(currentIndex);
    }
    updatePlot(); // Refresh chart with cleaned state
}

void PostProcessPage::startTrimming(QPointF point) {
    trimStartPoint = point;
    isTrimming = true;
    trimRect = QRectF(point, QSizeF(0, 0));
    chartView->setRubberBand(QChartView::RectangleRubberBand);
}

void PostProcessPage::updateTrimming(QPointF point) {
    if (isTrimming) {
        qreal left = qMin(trimStartPoint.x(), point.x());
        qreal right = qMax(trimStartPoint.x(), point.x());
        qreal top = qMin(trimStartPoint.y(), point.y());
        qreal bottom = qMax(trimStartPoint.y(), point.y());
        trimRect = QRectF(left, top, right - left, bottom - top);
        // برای نمایش تعاملی می‌توانید اینجا رندر را به‌روز کنید، اما فعلاً فقط ذخیره می‌کنیم
    }
}

void PostProcessPage::endTrimming(QPointF point) {
    if (isTrimming) {
        isTrimming = false;
        chartView->setRubberBand(QChartView::NoRubberBand);

        // تبدیل مختصات نمودار به داده‌ها
        QValueAxis *axisX = qobject_cast<QValueAxis*>(chart->axes(Qt::Horizontal).first());
        QValueAxis *axisY = qobject_cast<QValueAxis*>(chart->axes(Qt::Vertical).first());
        qreal minX = axisX->min();
        qreal maxX = axisX->max();
        qreal minY = axisY->min();
        qreal maxY = axisY->max();

        qreal trimMinX = minX + (trimRect.left() / chartView->width()) * (maxX - minX);
        qreal trimMaxX = minX + (trimRect.right() / chartView->width()) * (maxX - minX);

        // فقط بر اساس محور X trim می‌کنیم (زمان)
        std::vector<double> newTimes, newValues;
        for (size_t i = 0; i < currentTimes.size(); ++i) {
            if (currentTimes[i] >= trimMinX && currentTimes[i] <= trimMaxX) {
                newTimes.push_back(currentTimes[i]);
                newValues.push_back(currentValues[i]);
            }
        }

        if (!newTimes.empty()) {
            trimmedTimesBackup = currentTimes; // ذخیره وضعیت قبل از trim
            trimmedValuesBackup = currentValues;
            currentTimes = newTimes;
            currentValues = newValues;
            calculateFs();
            updatePlot();
        }
    }
}

void PostProcessPage::undoTrim() {
    if (!trimmedTimesBackup.empty() && !trimmedValuesBackup.empty()) {
        currentTimes = trimmedTimesBackup;
        currentValues = trimmedValuesBackup;
        trimmedTimesBackup = originalTimes; // ذخیره وضعیت اولیه برای Undo بعدی
        trimmedValuesBackup = originalValues;
        calculateFs();
        updatePlot();
    }
}

void PostProcessPage::populateFiles() {
    fileCombo->clear();
    QDir dir("data");
    dir.setNameFilters(QStringList() << "*.csv");
    QStringList files = dir.entryList(QDir::Files, QDir::Time);
    fileCombo->addItems(files);
}

void PostProcessPage::loadFile(int index) {
    if (index < 0) return;
    currentFile = "data/" + fileCombo->currentText(); // Path کامل برای هر دو حالت
    fileLabel->setText("Selected File: " + fileCombo->currentText());

    if (xyMode()) {
        int jx = jointXCombo->currentIndex();
        int jy = jointYCombo->currentIndex();
        XYMetric metric = currentXYMetric();
        loadXYData(jx, jy, metric);
        calculateFs(); // اگر fs برای XY نیاز باشه (مثلاً بر اساس طول angleX محاسبه کن)
        // تنظیم اسلایدر cutoff (فرض fs محاسبه‌شده)
        cutoffSlider->setRange(1, static_cast<int>(fs * 50.0));
        cutoffSlider->setValue(std::min(cutoffSlider->value(), static_cast<int>(fs * 50.0)));
        cutoffValueLabel->setText(QString("%1 Hz").arg(cutoffSlider->value() / 100.0, 0, 'f', 2));
        updatePlot();
        return;
    }

    // Time mode (کد اصلی بدون تغییر، از این به بعد)
    currentIMU = imuCombo->currentText().toInt();
    currentParam = paramCombo->currentText();
    // Guard against invalid param (prevents crash if combo not set)
    if (paramMap.find(currentParam) == paramMap.end()) {
        currentParam = "AccX"; // Default
        paramCombo->setCurrentText("AccX");
    }
    if (currentIMU < 1 || currentIMU > 18) {
        currentIMU = 1;
        imuCombo->setCurrentIndex(0);
    }
    QFile file(currentFile);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "Error", "Failed to open file: " + currentFile);
        return;
    }
    QTextStream in(&file);
    QString header = in.readLine(); // Skip header
    originalTimes.clear();
    originalValues.clear();
    while (!in.atEnd()) {
        QString line = in.readLine();
        QStringList parts = line.split(",");
        if (parts.size() < 16) continue; // حداقل 16 column
        int id = parts[0].toInt();
        if (id == currentIMU) {
            double time = parts[14].toDouble() / 1000.0; // Time در ms به s
            int col = paramMap[currentParam];
            double value = parts[col].toDouble();
            originalTimes.push_back(time);
            originalValues.push_back(value);
        }
    }
    file.close();
    // اگر داده‌ای نبود (مثل IMU خاموش)، warning بده
    if (originalTimes.empty()) {
        QMessageBox::information(this, "Info", "No data for selected IMU in this file.");
        return;
    }
    currentTimes = originalTimes;
    currentValues = originalValues;
    calculateFs();
    // محدود کردن اسلایدر cutoff به fs/2
    cutoffSlider->setRange(1, static_cast<int>(fs * 50.0)); // fs/2 * 100 برای اسلایدر
    cutoffSlider->setValue(std::min(cutoffSlider->value(), static_cast<int>(fs * 50.0))); // تنظیم مقدار فعلی
    cutoffValueLabel->setText(QString("%1 Hz").arg(cutoffSlider->value() / 100.0, 0, 'f', 2));
    updatePlot();
}

void PostProcessPage::loadXYData(int jointXIdx, int jointYIdx, XYMetric metric) {
    angleX.clear();
    angleY.clear();
    filteredY.clear();
    movingY.clear(); // Reset filtered/moving for new selection

    QFile file(currentFile);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "Error", "Cannot open CSV file.");
        return;
    }
    QTextStream in(&file);
    in.readLine(); // Skip header

    int colX = 3 * jointXIdx + static_cast<int>(metric);
    int colY = 3 * jointYIdx + static_cast<int>(metric);

    while (!in.atEnd()) {
        QString line = in.readLine();
        QStringList fields = line.split(',');
        if (fields.size() > std::max(colX, colY)) {
            bool okX, okY;
            double vx = fields[colX].toDouble(&okX);
            double vy = fields[colY].toDouble(&okY);
            if (okX && okY) {
                angleX.push_back(vx);
                angleY.push_back(vy);
            }
        }
    }
    file.close();

    // Backup for trim (initial)
    angleXBackup = angleX;
    angleYBackup = angleY;
}

QString PostProcessPage::getAngleName(XYMetric metric) const {
    return angleNames[static_cast<int>(metric)];
}

void PostProcessPage::calculateFs() {
    if (currentTimes.size() < 2) {
        fs = 500.0; // Default
        return;
    }
    double dt = 0.0;
    for (size_t i = 1; i < currentTimes.size(); ++i) {
        dt += currentTimes[i] - currentTimes[i - 1];
    }
    dt /= (currentTimes.size() - 1);
    fs = 500.0 / dt;
    qDebug() << "fs:" << fs;
}

void PostProcessPage::updatePlot() {
    if (xyMode()) {
        updateXYPlot();
        return;
    }

    if (currentTimes.empty()) {
        // Handle no data case (unchanged)
        series->clear();
        filteredSeries->clear();
        movingAverageSeries->clear();
        chart->setTitle("No data for selected IMU. Please select an appropriate file or IMU.");
        QValueAxis *axisX = qobject_cast<QValueAxis*>(chart->axes(Qt::Horizontal).first());
        QValueAxis *axisY = qobject_cast<QValueAxis*>(chart->axes(Qt::Vertical).first());
        if (axisX) axisX->setRange(0, 1);
        if (axisY) axisY->setRange(-1, 1);
        series->setVisible(false);
        filteredSeries->setVisible(false);
        movingAverageSeries->setVisible(false);
        xySeries->setVisible(false);
        xyFilteredSeries->setVisible(false);
        xyMovingSeries->setVisible(false);
        return;
    }

    // Append original (always)
    series->clear();
    for (size_t i = 0; i < currentTimes.size(); ++i) {
        series->append(currentTimes[i], currentValues[i]);
    }

    // Re-append filtered if available (from vector to persist after clear/switch)
    if (!filteredValues.empty()) {
        filteredSeries->clear();
        for (size_t i = 0; i < currentTimes.size() && i < filteredValues.size(); ++i) {
            filteredSeries->append(currentTimes[i], filteredValues[i]);
        }
        filteredSeries->setVisible(true);
    } else {
        filteredSeries->setVisible(false);
    }

    // Re-append moving average if available (from vector to persist after clear/switch)
    if (!movingAverageValues.empty()) {
        movingAverageSeries->clear();
        for (size_t i = 0; i < currentTimes.size() && i < movingAverageValues.size(); ++i) {
            movingAverageSeries->append(currentTimes[i], movingAverageValues[i]);
        }
        movingAverageSeries->setVisible(true);
    } else {
        movingAverageSeries->setVisible(false);
    }

    // Range calculation (include all, using vectors for persistence)
    double minT = *std::min_element(currentTimes.begin(), currentTimes.end());
    double maxT = *std::max_element(currentTimes.begin(), currentTimes.end());
    double minV = *std::min_element(currentValues.begin(), currentValues.end());
    double maxV = *std::max_element(currentValues.begin(), currentValues.end());

    // Include filtered from vector
    if (!filteredValues.empty()) {
        double fMin = *std::min_element(filteredValues.begin(), filteredValues.end());
        double fMax = *std::max_element(filteredValues.begin(), filteredValues.end());
        minV = std::min(minV, fMin);
        maxV = std::max(maxV, fMax);
    }

    // Include moving average from vector
    if (!movingAverageValues.empty()) {
        double maMin = *std::min_element(movingAverageValues.begin(), movingAverageValues.end());
        double maMax = *std::max_element(movingAverageValues.begin(), movingAverageValues.end());
        minV = std::min(minV, maMin);
        maxV = std::max(maxV, maMax);
    }

    chart->setTitle(titleEdit->text().isEmpty() ? (currentParam + " for IMU " + QString::number(currentIMU)) : titleEdit->text());
    QValueAxis *axisX = qobject_cast<QValueAxis*>(chart->axes(Qt::Horizontal).first());
    QValueAxis *axisY = qobject_cast<QValueAxis*>(chart->axes(Qt::Vertical).first());
    axisX->setRange(minT, maxT);
    axisY->setRange(minV - 0.1 * (maxV - minV), maxV + 0.1 * (maxV - minV));

    // Hide XY series in time mode
    xySeries->setVisible(false);
    xyFilteredSeries->setVisible(false);
    xyMovingSeries->setVisible(false);

    // Show time series (original always visible)
    series->setVisible(true);
}

void PostProcessPage::updateXYPlot() {
    if (angleX.empty() || angleY.size() != angleX.size()) return;

    QString angleName = getAngleName(currentXYMetric());
    QString jxName = jointNames[jointXCombo->currentIndex()];
    QString jyName = jointNames[jointYCombo->currentIndex()];
    QString title = titleEdit->text().isEmpty() ? QString("%1 (%2) vs %3 (%4)").arg(angleName, jxName, jyName, angleName) : titleEdit->text();
    chart->setTitle(title);

    QValueAxis *axisX = qobject_cast<QValueAxis*>(chart->axes(Qt::Horizontal).first());
    QValueAxis *axisY = qobject_cast<QValueAxis*>(chart->axes(Qt::Vertical).first());
    axisX->setTitleText(QString("%1 %2").arg(jxName, angleName));
    axisY->setTitleText(QString("%1 %2").arg(jyName, angleName));

    // Original
    xySeries->clear();
    for (size_t i = 0; i < angleX.size(); ++i) {
        xySeries->append(angleX[i], angleY[i]);
    }
    xySeries->setVisible(true);

    // Filtered (if applied)
    xyFilteredSeries->clear();
    if (!filteredY.empty()) {
        for (size_t i = 0; i < angleX.size(); ++i) {
            xyFilteredSeries->append(angleX[i], filteredY[i]);
        }
        xyFilteredSeries->setVisible(true);
    } else {
        xyFilteredSeries->setVisible(false);
    }

    // Moving average (on Y)
    xyMovingSeries->clear();
    if (!movingY.empty()) {
        for (size_t i = 0; i < angleX.size(); ++i) {
            xyMovingSeries->append(angleX[i], movingY[i]);
        }
        xyMovingSeries->setVisible(true);
    } else {
        xyMovingSeries->setVisible(false);
    }

    // Hide time series
    series->setVisible(false);
    filteredSeries->setVisible(false);
    movingAverageSeries->setVisible(false);

    // Auto range
    double minX = *std::min_element(angleX.begin(), angleX.end());
    double maxX = *std::max_element(angleX.begin(), angleX.end());
    double minY = *std::min_element(angleY.begin(), angleY.end());
    double maxY = *std::max_element(angleY.begin(), angleY.end());
    // Include filtered/moving if visible
    if (!filteredY.empty()) {
        minY = std::min(minY, *std::min_element(filteredY.begin(), filteredY.end()));
        maxY = std::max(maxY, *std::max_element(filteredY.begin(), filteredY.end()));
    }
    if (!movingY.empty()) {
        minY = std::min(minY, *std::min_element(movingY.begin(), movingY.end()));
        maxY = std::max(maxY, *std::max_element(movingY.begin(), movingY.end()));
    }
    double marginX = (maxX - minX) * 0.05;
    double marginY = (maxY - minY) * 0.05;
    axisX->setRange(minX - marginX, maxX + marginX);
    axisY->setRange(minY - marginY, maxY + marginY);
}

void PostProcessPage::trimData() {
    if (currentTimes.empty()) return;

    double minT = *std::min_element(currentTimes.begin(), currentTimes.end());
    double maxT = *std::max_element(currentTimes.begin(), currentTimes.end());

    bool ok;
    double start = QInputDialog::getDouble(this, "Trim Start", "Enter start time:", minT, minT, maxT, 2, &ok);
    if (!ok) return;
    double end = QInputDialog::getDouble(this, "Trim End", "Enter end time:", maxT, start, maxT, 2, &ok);
    if (!ok) return;

    std::vector<double> newTimes, newValues;
    for (size_t i = 0; i < currentTimes.size(); ++i) {
        if (currentTimes[i] >= start && currentTimes[i] <= end) {
            newTimes.push_back(currentTimes[i]);
            newValues.push_back(currentValues[i]);
        }
    }

    currentTimes = newTimes;
    currentValues = newValues;
    calculateFs();
    updatePlot();
}

void PostProcessPage::toggleFilterFrame() {
    filterFrame->setVisible(!filterFrame->isVisible());
}

void PostProcessPage::applyFilter() {
    if (xyMode()) {
        return; // فیلتر فقط برای Time mode
    }
    if (currentValues.empty()) {
        return;
    }

    double cutoff = cutoffSlider->value() / 100.0; // Convert to Hz
    if (cutoff > fs / 2.0) {
        QMessageBox::warning(this, "Error", QString("Cutoff frequency (%1 Hz) cannot exceed half the sampling frequency (%2 Hz).").arg(cutoff, 0, 'f', 2).arg(fs / 2.0, 0, 'f', 2));
        return;
    }

    int order = orderSlider->value();
    bool highpass = typeCombo->currentText() == "High Pass";

    filteredValues = applyButterworthFilter(currentValues, cutoff, order, fs, highpass); // ست vector

    // Append به series (مثل اولیه)
    filteredSeries->clear();
    for (size_t i = 0; i < currentTimes.size() && i < filteredValues.size(); ++i) {
        filteredSeries->append(currentTimes[i], filteredValues[i]);
    }

    QString title = titleEdit->text().isEmpty() ? QString("%1 (IMU %2)").arg(currentParam).arg(currentIMU) : titleEdit->text();
    chart->setTitle(title);

    QValueAxis *axisX = qobject_cast<QValueAxis*>(chart->axes(Qt::Horizontal).first());
    QValueAxis *axisY = qobject_cast<QValueAxis*>(chart->axes(Qt::Vertical).first());
    axisY->setTitleText(currentParam);

    double minY = *std::min_element(filteredValues.begin(), filteredValues.end());
    double maxY = *std::max_element(filteredValues.begin(), filteredValues.end());
    double minYOrig = *std::min_element(currentValues.begin(), currentValues.end());
    double maxYOrig = *std::max_element(currentValues.begin(), currentValues.end());
    minY = std::min(minY, minYOrig);
    maxY = std::max(maxY, maxYOrig);
    double margin = (maxY - minY) * 0.1;
    axisY->setRange(minY - margin, maxY + margin);

    if (!currentTimes.empty()) {
        axisX->setRange(currentTimes.front(), currentTimes.back());
    }

    updatePlot(); // Refresh to show all series together (original + filtered)
}

void PostProcessPage::updateCutoff(int value) {
    if (xyMode()) return; // فقط Time mode

    double cutoff = value / 100.0; // Convert to Hz
    if (cutoff > fs / 2.0) {
        cutoffSlider->setValue(static_cast<int>(fs * 50.0)); // تنظیم به حداکثر مجاز
        cutoff = fs / 2.0;
        QMessageBox::warning(this, "Error", QString("Cutoff frequency cannot exceed half the sampling frequency (%1 Hz).").arg(fs / 2.0, 0, 'f', 2));
    }
    cutoffValueLabel->setText(QString("%1 Hz").arg(cutoff, 0, 'f', 2));
    applyFilter(); // Update the plot in real-time
}

void PostProcessPage::updateOrder(int value) {
    if (xyMode()) return; // فقط Time mode

    orderValueLabel->setText(QString("%1").arg(value));
    applyFilter(); // Update the plot in real-time
}

void PostProcessPage::saveToHtml() {
    QString fileName = QFileDialog::getSaveFileName(this, "Save HTML", "", "HTML (*.html)");
    if (fileName.isEmpty()) return;

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "Error", "Failed to save HTML file.");
        return;
    }

    QTextStream out(&file);
    out << "<!DOCTYPE html>\n";
    out << "<html>\n<head>\n";
    out << "<script src='https://cdn.plot.ly/plotly-latest.min.js'></script>\n";
    out << "<style>body { background: " << (isDarkTheme ? "#232526" : "#ffffff") << "; color: " << (isDarkTheme ? "#f8f8f8" : "#000000") << "; }</style>\n";
    out << "</head>\n<body>\n";
    out << "<div id='chart' style='width:100%;height:600px;'></div>\n";
    out << "<script>\n";
    out << "var trace1 = {\n";
    out << "  x: [";
    for (size_t i = 0; i < currentTimes.size(); ++i) {
        out << QString::number(currentTimes[i], 'f', 6);
        if (i < currentTimes.size() - 1) out << ",";
    }
    out << "],\n";
    out << "  y: [";
    for (size_t i = 0; i < currentValues.size(); ++i) {
        out << QString::number(currentValues[i], 'f', 6);
        if (i < currentValues.size() - 1) out << ",";
    }
    out << "],\n";
    out << "  type: 'scatter',\n";
    out << "  mode: 'lines',\n";
    out << "  name: 'Original Data',\n";
    out << "  line: {color: '" << (isDarkTheme ? "#43cea2" : "#000000") << "'}\n";
    out << "};\n";

    out << "var trace2 = {\n";
    out << "  x: [";
    for (size_t i = 0; i < currentTimes.size(); ++i) {
        out << QString::number(currentTimes[i], 'f', 6);
        if (i < currentTimes.size() - 1) out << ",";
    }
    out << "],\n";
    out << "  y: [";
    for (size_t i = 0; i < currentValues.size() && i < filteredSeries->points().size(); ++i) {
        out << QString::number(filteredSeries->points()[i].y(), 'f', 6);
        if (i < currentValues.size() - 1) out << ",";
    }
    out << "],\n";
    out << "  type: 'scatter',\n";
    out << "  mode: 'lines',\n";
    out << "  name: 'Filtered Data',\n";
    out << "  line: {color: '" << (isDarkTheme ? "#00ff00" : "#008000") << "'}\n";
    out << "};\n";

    out << "var trace3 = {\n";
    out << "  x: [";
    for (size_t i = 0; i < currentTimes.size(); ++i) {
        out << QString::number(currentTimes[i], 'f', 6);
        if (i < currentTimes.size() - 1) out << ",";
    }
    out << "],\n";
    out << "  y: [";
    for (size_t i = 0; i < movingAverageValues.size(); ++i) {
        out << QString::number(movingAverageValues[i], 'f', 6);
        if (i < movingAverageValues.size() - 1) out << ",";
    }
    out << "],\n";
    out << "  type: 'scatter',\n";
    out << "  mode: 'lines',\n";
    out << "  name: 'Moving Average',\n";
    out << "  line: {color: '" << (isDarkTheme ? "#FF0000" : "#8B0000") << "'}\n";
    out << "};\n";

    out << "var data = [trace1, trace2, trace3];\n";
    out << "var layout = {\n";
    out << "  title: '" << chart->title() << "',\n";
    out << "  xaxis: {title: 'Time (s)'},\n";
    out << "  yaxis: {title: '" << currentParam << "'},\n";
    out << "  showlegend: true,\n";
    out << "  plot_bgcolor: '" << (isDarkTheme ? "#3e3e4e" : "#f5f5f5") << "',\n";
    out << "  paper_bgcolor: '" << (isDarkTheme ? "#232526" : "#ffffff") << "',\n";
    out << "  font: {color: '" << (isDarkTheme ? "#f8f8f8" : "#000000") << "'}\n";
    out << "};\n";
    out << "Plotly.newPlot('chart', data, layout, {responsive: true, displayModeBar: true});\n";
    out << "</script>\n</body>\n</html>\n";

    file.close();
}

struct SOSSection {
    double b0, b1, b2;
    double a1, a2;
};

struct Biquad {
    // Direct Form I (or II-T) state
    double b0{}, b1{}, b2{}, a0{1.0}, a1{}, a2{};
    double z1{0.0}, z2{0.0};

    inline double process(double x) {
        // Transposed Direct Form II (stable, low-state)
        double y = b0/a0 * x + z1;
        z1 = b1/a0 * x - a1/a0 * y + z2;
        z2 = b2/a0 * x - a2/a0 * y;
        return y;
    }
};

// Compute Butterworth biquad Qs for any positive order N (modified to handle odd N by reserving for floor(N/2))
static std::vector<double> butterworth_Qs(int N) {
    if (N <= 0) {
        throw std::invalid_argument("Butterworth order N must be a positive integer (1,2,3,...)");
    }
    std::vector<double> Qs;
    Qs.reserve(N/2);
    for (int k = 0; k < N/2; ++k) {
        double theta = (2.0 * k + 1.0) * M_PI / (2.0 * N);
        double Q = 1.0 / (2.0 * std::cos(theta));
        Qs.push_back(Q);
    }
    return Qs;
}

// RBJ cookbook biquad design for low-pass
static Biquad design_biquad_lowpass(double Fs, double Fc, double Q) {
    if (!(Fc > 0.0 && Fc < Fs * 0.5)) {
        throw std::invalid_argument("Fc must be in (0, Fs/2).");
    }
    double w0 = 2.0 * M_PI * (Fc / Fs);
    double cw = std::cos(w0);
    double sw = std::sin(w0);
    double alpha = sw / (2.0 * Q);

    Biquad bq;
    bq.b0 = (1.0 - cw) * 0.5;
    bq.b1 = (1.0 - cw);
    bq.b2 = (1.0 - cw) * 0.5;
    bq.a0 = 1.0 + alpha;
    bq.a1 = -2.0 * cw;
    bq.a2 = 1.0 - alpha;
    return bq;
}

// RBJ cookbook biquad design for high-pass
static Biquad design_biquad_highpass(double Fs, double Fc, double Q) {
    if (!(Fc > 0.0 && Fc < Fs * 0.5)) {
        throw std::invalid_argument("Fc must be in (0, Fs/2).");
    }
    double w0 = 2.0 * M_PI * (Fc / Fs);
    double cw = std::cos(w0);
    double sw = std::sin(w0);
    double alpha = sw / (2.0 * Q);

    Biquad bq;
    bq.b0 = (1.0 + cw) * 0.5;
    bq.b1 = -(1.0 + cw);
    bq.b2 = (1.0 + cw) * 0.5;
    bq.a0 = 1.0 + alpha;
    bq.a1 = -2.0 * cw;
    bq.a2 = 1.0 - alpha;
    return bq;
}

// Process a signal through a cascade (one-way filter application)
static std::vector<double> iir_process(const std::vector<Biquad>& sos, const std::vector<double>& x) {
    std::vector<double> y(x);
    for (auto bq : sos) {  // Copy each bq to avoid modifying original states
        Biquad section = bq;
        for (size_t n = 0; n < y.size(); ++n) {
            y[n] = section.process(y[n]);
        }
    }
    return y;
}

std::vector<double> PostProcessPage::applyButterworthFilter(const std::vector<double>& signal, double cutoff, int order, double fs, bool highpass) {
    if (signal.empty() || order < 1 || cutoff <= 0 || cutoff >= fs / 2.0) {
        return signal;
    }

    const double pi = std::acos(-1.0);
    double fc_norm = cutoff / (fs / 2.0);
    double tan_half_w0 = std::tan(pi * fc_norm / 2.0);
    double w = tan_half_w0;

    std::vector<Biquad> sos;
    auto Qs = butterworth_Qs(order);
    for (double Q : Qs) {
        sos.push_back(design_biquad_lowpass(fs, cutoff, Q)); // Always design low-pass filter
    }

    // Handle odd order by adding first-order section (using bilinear transform)
    if (order % 2 == 1) {
        double den = 1.0 + w;
        Biquad bq;
        bq.b0 = w;
        bq.b1 = w;
        bq.b2 = 0.0;
        bq.a0 = den;
        bq.a1 = w - 1.0;
        bq.a2 = 0.0;
        sos.push_back(bq);
    }

    // Apply zero-phase filtering (filtfilt) for low-pass
    std::vector<double> lowpass_y = iir_process(sos, signal);
    std::reverse(lowpass_y.begin(), lowpass_y.end());
    lowpass_y = iir_process(sos, lowpass_y);
    std::reverse(lowpass_y.begin(), lowpass_y.end());

    if (!highpass) {
        return lowpass_y; // Return low-pass filtered signal
    }

    // For high-pass, subtract low-pass output from original signal
    std::vector<double> highpass_y(signal.size());
    for (size_t i = 0; i < signal.size(); ++i) {
        highpass_y[i] = signal[i] - lowpass_y[i];
    }

    return highpass_y;
}

void PostProcessPage::toggleMovingAverageFrame() {
    movingAverageFrame->setVisible(!movingAverageFrame->isVisible());
}

void PostProcessPage::applyMovingAverage() {
    if (xyMode()) {
        return; // Moving average فقط برای Time mode
    }
    if (currentValues.empty()) {
        return;
    }

    int windowSize = windowSizeSlider->value();
    movingAverageValues = calculateMovingAverage(currentValues, windowSize); // Set vector for persistence

    QString title = titleEdit->text().isEmpty() ? QString("%1 (IMU %2)").arg(currentParam).arg(currentIMU) : titleEdit->text();
    chart->setTitle(title);

    updatePlot(); // Refresh to append and show all series (original + filtered + moving)
}

void PostProcessPage::updateWindowSize(int value) {
    if (xyMode()) return;
    windowSizeValueLabel->setText(QString("%1").arg(value));
    applyMovingAverage(); // به‌روزرسانی بلادرنگ
}

void PostProcessPage::undoMovingAverage() {
    if (xyMode()) return; // فقط Time mode
    movingAverageSeries->clear();
    movingAverageValues.clear(); // Clear vector to persist
    updatePlot(); // به‌روزرسانی نمودار
}

std::vector<double> PostProcessPage::calculateMovingAverage(const std::vector<double>& signal, int windowSize) {
    std::vector<double> result(signal.size(), 0.0);
    for (size_t i = 0; i < signal.size(); ++i) {
        if (i < static_cast<size_t>(windowSize)) {
            double sum = std::accumulate(signal.begin(), signal.begin() + i + 1, 0.0);
            result[i] = sum / (i + 1);
        } else {
            double sum = std::accumulate(signal.begin() + i - windowSize + 1, signal.begin() + i + 1, 0.0);
            result[i] = sum / windowSize;
        }
    }
    return result;
}

void PostProcessPage::handlePointHover(const QPointF &point, bool state) {
    if (state) {
        QValueAxis *axisX = qobject_cast<QValueAxis*>(chart->axes(Qt::Horizontal).first());
        QValueAxis *axisY = qobject_cast<QValueAxis*>(chart->axes(Qt::Vertical).first());
        QString tooltip = QString("Time: %1 s\nValue: %2")
                              .arg(point.x(), 0, 'f', 2)
                              .arg(point.y(), 0, 'f', 2);
        QToolTip::showText(QCursor::pos(), tooltip, chartView);
    } else {
        QToolTip::hideText();
    }
}

void PostProcessPage::undoFilter() {
    if (xyMode()) return; // فقط Time mode
    filteredSeries->clear();
    filteredValues.clear(); // Clear vector to persist
    updatePlot(); // Update the plot to reflect the change
}
