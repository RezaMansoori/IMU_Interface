#include "combinedcalibrationviewerpage.h"

CombinedCalibrationViewerPage::CombinedCalibrationViewerPage(QWidget *parent, int numImus, bool isDarkTheme, USBReceiver *usbReceiver)
    : QWidget(parent)
{
    mainLayout = new QHBoxLayout(this);

    // ساخت کمبوباکس مشترک
    sharedComboBox = new QComboBox(this);
    for (int i = 1; i <= numImus; ++i) {
        sharedComboBox->addItem(QString("IMU %1").arg(i));
    }

    // ساخت صفحات و پاس دادن کمبوباکس مشترک
    modelViewer = new ModelViewerPage(this, numImus, isDarkTheme, sharedComboBox);
    calibrationView = new CalibrationPage(this, numImus, isDarkTheme, usbReceiver, sharedComboBox);

    mainLayout->addWidget(modelViewer, 1);
    mainLayout->addWidget(calibrationView, 1);

    // اتصال داده‌های ورودی
    if (usbReceiver) {
        connect(usbReceiver, &USBReceiver::data, modelViewer, &ModelViewerPage::updateModel);
        connect(usbReceiver, &USBReceiver::data, calibrationView, &CalibrationPage::handleIMUData);
        connect(usbReceiver, &USBReceiver::connectionStatus, calibrationView, &CalibrationPage::handleUSBConnection);
    }

    // اتصال تغییرات انتخاب کمبوباکس مشترک به هردو صفحه
    connect(sharedComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), modelViewer, &ModelViewerPage::changeIMU);
    connect(sharedComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), calibrationView, &CalibrationPage::selectIMUFromCombo);
}

ModelViewerPage* CombinedCalibrationViewerPage::getModelViewerPage() const {
    return modelViewer;
}

CalibrationPage* CombinedCalibrationViewerPage::getCalibrationPage() const {
    return calibrationView;
}

QComboBox* CombinedCalibrationViewerPage::getSharedComboBox() const {
    return sharedComboBox;
}
