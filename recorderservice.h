#ifndef RECORDERSERVICE_H
#define RECORDERSERVICE_H

#include <QtCore/QFile>
#include <QtCore/QObject>
#include <QtCore/QTextStream>
#include "imu.h"

class RecorderService : public QObject {
    Q_OBJECT
public:
    explicit RecorderService(QObject* parent = nullptr);

    void addData(const IMU& data);
    void start();
    void stop();
    QString getCurrentFileName() const; // برای دسترسی به نام فایل CSV

signals:
    void onError(const QString& err);
    void recordingStarted(const QString& fileName); // سیگنال برای اطلاع‌رسانی شروع ضبط
    void recordingStopped(const QString& fileName); // سیگنال برای اطلاع‌رسانی توقف ضبط

private:
    QTextStream m_io;
    QFile m_file;
    bool m_recording;
    QString m_fileName;

    void writeHeader();
};

#endif // RECORDERSERVICE_H