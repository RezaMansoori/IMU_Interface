#include "recorderservice.h"
#include <QDateTime>
#include <QDir>
#include <QDebug>

RecorderService::RecorderService(QObject* parent)
    : QObject(parent), m_recording(false) {
    m_io.setRealNumberNotation(QTextStream::FixedNotation);
}

void RecorderService::start() {
    QDir().mkpath("data"); // ایجاد پوشه data اگر وجود نداشته باشد
    m_fileName = QString("data/imu_data_%1.csv").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));
    m_file.setFileName(m_fileName);
    if (m_file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        m_io.setDevice(&m_file);
        writeHeader();
        m_recording = true;
        emit recordingStarted(m_fileName);
        qDebug() << "Recording started, file:" << m_fileName;
    } else {
        emit onError("Failed to open CSV file for writing: " + m_fileName);
        m_recording = false;
    }
}

void RecorderService::stop() {
    if (m_recording) {
        m_recording = false;
        m_io.setDevice(nullptr);
        m_file.close();
        emit recordingStopped(m_fileName);
        qDebug() << "Recording stopped, file:" << m_fileName;
    }
}

void RecorderService::addData(const IMU& data) {
    if (m_recording) {
        m_io << data << Qt::endl;
        m_io.flush();
    }
}

void RecorderService::writeHeader() {
    m_io << IMU::csvHeader() << Qt::endl;
    qDebug() << "IMU Header is written";
}

QString RecorderService::getCurrentFileName() const {
    return m_fileName;
}
