#ifndef USBRECEIVER_H
#define USBRECEIVER_H

#include <QThread>
#include <QtSerialPort/QSerialPortInfo>
#include "imu.h"
#include "recorderservice.h"

#define DEFAULT_INTERVAL_MS 1
#define DEFAULT_MAX_RETRY 3
#define USB_VID 0x303a
#define USB_PID 0x4001

class USBReceiver : public QThread {
    Q_OBJECT
public:
    enum USBReceiverErrors {
        USBPortNotFound,
        USBPortNotOpen,
        DataReadTimedOut,
        DataReadReachMaximumRetry,
        InvalidData
    };
    Q_ENUM(USBReceiverErrors)
    explicit USBReceiver(int axisMode = 9, int numImus = 18, QObject *parent = nullptr);

    void stop();
    void searchUSB();
    void startRecording();
    void stopRecording();
    bool isRecording() const;
    void setAxisMode(int mode);
    void setNumImus(int num);

signals:
    void error(USBReceiverErrors code, QString error);
    void data(const IMU &data);
    void connectionStatus(bool connected);

protected:
    void run() override;

private:
    bool m_running;
    bool m_searchRequested;
    bool m_isRecording;
    int m_axisMode;
    int m_numImus;
    RecorderService *m_recorder;

    static QSerialPortInfo findPort();
    bool validateData(const IMU &data);
};

#endif // USBRECEIVER_H
