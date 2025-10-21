#ifndef INTROPLAYER_H
#define INTROPLAYER_H

#include <QWidget>
#include <QMediaPlayer>
#include <QVideoWidget>
#include <QVBoxLayout>

class IntroPlayer : public QWidget {
    Q_OBJECT

public:
    explicit IntroPlayer(QWidget *parent = nullptr);

signals:
    void introFinished();

private:
    QMediaPlayer *player;
    QVideoWidget *videoWidget;
};

#endif // INTROPLAYER_H
