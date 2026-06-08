#include "usbreceiver.h"
#include <QtCore/QDebug>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <chrono>

USBReceiver::USBReceiver(int axisMode, int numImus, QObject *parent)
    : QThread{parent},
    m_running(false),
    m_searchRequested(false),
    m_isRecording(false),
    m_axisMode(axisMode),
    m_numImus(numImus),
    m_recorder(new RecorderService(this))
{
    connect(m_recorder, &RecorderService::onError, this, [this](const QString &err) {
        emit error(InvalidData, err);
    });
}

bool USBReceiver::isRecording() const
{
    return m_isRecording;
}

void USBReceiver::stop()
{
    m_running = false;
    stopRecording();
}

void USBReceiver::searchUSB()
{
    m_searchRequested = true;
}

void USBReceiver::startRecording()
{
    if (!m_isRecording) {
        m_isRecording = true;
        m_recorder->start();
    }
}

void USBReceiver::stopRecording()
{
    if (m_isRecording) {
        m_isRecording = false;
        m_recorder->stop();
    }
}

void USBReceiver::setAxisMode(int mode)
{
    m_axisMode = mode;
}

void USBReceiver::setNumImus(int num)
{
    m_numImus = num;
}

void USBReceiver::run()
{
    QSerialPort *m_port = nullptr;
    m_running = true;
    QByteArray buffer(64, '\0');
    IMU temp;
    unsigned short retry_count = 0;

    QDataStream ds(&buffer, QIODevice::ReadOnly);
    ds.setByteOrder(QDataStream::LittleEndian);
    ds.setFloatingPointPrecision(QDataStream::SinglePrecision);

    while (m_running) {
        if (m_searchRequested || !m_port || !m_port->isOpen()) {
            if (m_port) {
                m_port->close();
                delete m_port;
                m_port = nullptr;
            }

            QSerialPortInfo info = findPort();
            if (info.isNull()) {
                if (m_searchRequested) {
                    emit error(USBPortNotFound,
                               QString("No USB device found with VID:0x%1, PID:0x%2")
                                   .arg(USB_VID, 4, 16, QChar('0'))
                                   .arg(USB_PID, 4, 16, QChar('0')));
                }
                emit connectionStatus(false);
                m_searchRequested = false;
                msleep(2000);
                continue;
            }

            m_port = new QSerialPort(info);
            if (!m_port->open(QIODevice::ReadWrite)) {
                emit error(USBPortNotOpen, "Could not open device: " + m_port->errorString());
                emit connectionStatus(false);
                m_searchRequested = false;
                delete m_port;
                m_port = nullptr;
                msleep(2000);
                continue;
            }

            m_port->setReadBufferSize(64);
            m_port->write(QString("A%1").arg(m_axisMode).toUtf8());
            if (!m_port->waitForBytesWritten(20)) {
                qDebug() << "The axis mode cannot be set!";
            }
            emit connectionStatus(true);
            m_searchRequested = false;
            retry_count = 0;
        }

        m_port->write("R");
        if (m_port->waitForReadyRead(100)) {
            retry_count = 0;
            buffer = m_port->readAll();

            if (buffer.size() == 62) {
                ds.device()->seek(0);
                ds >> temp;
                if (validateData(temp)) {
                    temp.hostEmitTimeUs = std::chrono::duration_cast<std::chrono::microseconds>(
                                              std::chrono::steady_clock::now().time_since_epoch())
                                              .count();
                    emit data(temp);
                    if (m_isRecording) {
                        m_recorder->addData(temp);
                    }
                } else {
                    emit error(InvalidData, "Invalid IMU data received.");
                }
            } else {
                retry_count++;
                emit error(DataReadTimedOut, QString("Incorrect data size: %1 bytes").arg(buffer.size()));
            }
        } else {
            retry_count++;
            emit error(DataReadTimedOut, "Data read timed out after 100ms.");
        }

        if (retry_count >= DEFAULT_MAX_RETRY) {
            emit error(DataReadReachMaximumRetry, "Maximum retries exceeded.");
            if (m_port) {
                m_port->close();
                delete m_port;
                m_port = nullptr;
            }
            emit connectionStatus(false);
            retry_count = 0;
        }

        msleep(1);
    }

    if (m_port) {
        m_port->close();
        delete m_port;
    }
    stopRecording();
}

QSerialPortInfo USBReceiver::findPort()
{
    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &port : ports) {
        if (port.hasProductIdentifier() && port.hasVendorIdentifier() &&
            port.vendorIdentifier() == USB_VID &&
            port.productIdentifier() == USB_PID) {
            return port;
        }
    }
    return QSerialPortInfo();
}

bool USBReceiver::validateData(const IMU &data)
{
    if (data.id < 1 || data.id > m_numImus) {
        return false;
    }
    if (data.battery < 0 || data.battery > 100) {
        return false;
    }
    for (int i = 0; i < 3; i++) {
        if (qIsNaN(data.acc[i]) || qIsNaN(data.gyro[i]) || qIsNaN(data.lacc[i])) {
            return false;
        }
    }
    for (int i = 0; i < 4; i++) {
        if (qIsNaN(data.quat[i])) {
            return false;
        }
    }
    return true;
}
