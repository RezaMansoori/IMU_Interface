#ifndef IMAGEWITHTEXTWIDGET_H
#define IMAGEWITHTEXTWIDGET_H

#include <QWidget>
#include <QLabel>
#include <QTimer>

class ImageWithTextWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ImageWithTextWidget(QWidget *parent = nullptr);
    void setImage(const QString &path);
    void setText(const QString &text);
    void setCountdown(int seconds);

protected:
    void resizeEvent(QResizeEvent *event) override;

private slots:
    void updateCountdown();

private:
    QLabel *imageLabel;
    QLabel *textLabel;
    QLabel *countdownLabel;
    QTimer *countdownTimer;
    int countdownSeconds;
};

#endif // IMAGEWITHTEXTWIDGET_H
