#ifndef COMBINEDCALIBRATIONVIEWERPAGE_H
#define COMBINEDCALIBRATIONVIEWERPAGE_H

#include <QWidget>
#include <QHBoxLayout>
#include <QComboBox>
#include "modelviewerpage.h"
#include "calibrationpage.h"
#include "usbreceiver.h"

class CombinedCalibrationViewerPage : public QWidget {
    Q_OBJECT

public:
    explicit CombinedCalibrationViewerPage(QWidget *parent = nullptr, int numImus = 18, bool isDarkTheme = true, USBReceiver* usbReceiver = nullptr);

    ModelViewerPage* getModelViewerPage() const;
    CalibrationPage* getCalibrationPage() const;
    QComboBox* getSharedComboBox() const;

private:
    QHBoxLayout *mainLayout;
    ModelViewerPage *modelViewer;
    CalibrationPage *calibrationView;
    QComboBox *sharedComboBox;
};

#endif // COMBINEDCALIBRATIONVIEWERPAGE_H