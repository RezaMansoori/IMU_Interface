#include "introplayer.h"
#include <QProcess>
#include <QTimer>
#include <QFile>
#include <QCoreApplication>
#include <QDebug>

IntroPlayer::IntroPlayer(QWidget *parent)
    : QWidget(parent)
{
    QString videoPath = QCoreApplication::applicationDirPath() + "/login.avi";
    QString playerPath = QCoreApplication::applicationDirPath() + "/ffplay.exe";

    if (!QFile::exists(videoPath) || !QFile::exists(playerPath)) {
        qWarning() << "⚠️ Video or player not found!";
        QTimer::singleShot(500, this, [=]() { emit introFinished(); });
        return;
    }

    QStringList arguments;
    arguments << "-autoexit"     // وقتی ویدیو تموم شد، پنجره بسته شه
              << "-fs"           // تمام صفحه
              << "-loglevel" << "quiet"  // بدون لاگ
              << videoPath;

    QProcess *process = new QProcess(this);
    connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [=](int, QProcess::ExitStatus) {
                emit introFinished();  // بعد از اتمام پخش، برو لاگین
            });

    process->start(playerPath, arguments);
}
