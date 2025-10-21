#include "ImageWithTextWidget.h"
#include <QVBoxLayout>
#include <QDebug>

ImageWithTextWidget::ImageWithTextWidget(QWidget *parent)
    : QWidget(parent), countdownSeconds(0)
{
    // Style background of the whole widget
    setStyleSheet("background-color: #555555; border: 0px solid white;");
    setAutoFillBackground(true);

    // Layout that holds imageLabel filling entire widget
    QVBoxLayout *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0,0,0,0);
    layout->setSpacing(0);

    // Image label
    imageLabel = new QLabel(this);
    imageLabel->setScaledContents(true);
    imageLabel->setStyleSheet("border: 1px solid white;");
    layout->addWidget(imageLabel);

    // Instruction text overlay (child of imageLabel)
    textLabel = new QLabel(this);
    textLabel->setAlignment(Qt::AlignCenter);
    textLabel->setStyleSheet(
        "font-size: 16px; font-weight: bold; color: white; "
        "background-color: rgba(0,0,0,150); padding: 5px; border-radius: 3px;");
    textLabel->setAttribute(Qt::WA_TransparentForMouseEvents);

    // شمارش معکوس هم فرزند this:
    countdownLabel = new QLabel(this);
    countdownLabel->setAlignment(Qt::AlignCenter);
    countdownLabel->setStyleSheet(
        "font-size: 20px; font-weight: bold; color: yellow; "
        "background-color: rgba(0,0,0,150); padding: 5px; border-radius: 8px;");
    countdownLabel->setAttribute(Qt::WA_TransparentForMouseEvents);
    countdownLabel->hide();

    // Timer for countdown
    countdownTimer = new QTimer(this);
    connect(countdownTimer, &QTimer::timeout, this, &ImageWithTextWidget::updateCountdown);
}

void ImageWithTextWidget::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);

    int textW = imageLabel->width() * 0.8;
    int textH = 30;
    textLabel->resize(textW, textH);

    // متن راهنما را 10 پیکسل از پایین تصویر قرار می‌دهیم
    int textX = (imageLabel->width() - textW) / 2 - 20;
    int textY = imageLabel->height() - textH;  // 10 پیکسل فاصله از پایین
    textLabel->move(textX, textY);

    // اندازه و موقعیت شمارش معکوس
    int countW = 40;
    int countH = 30;
    countdownLabel->resize(countW, countH);

    // شمارش معکوس را بالاتر از متن راهنما (مثلاً 10 پیکسل فاصله بالاتر از textLabel)
    int countX = imageLabel->width() - countW;
    int countY = textY;  // 10 پیکسل فاصله بالاتر از متن راهنما

    countdownLabel->move(countX, countY);
}

void ImageWithTextWidget::setImage(const QString &path)
{
    QPixmap pix(path);
    if (pix.isNull()) {
        qWarning() << "Failed to load image:" << path;
        imageLabel->setText("Image not found");
        imageLabel->setAlignment(Qt::AlignCenter);
        imageLabel->setStyleSheet("color: white; background-color: black;");
    } else {
        imageLabel->setPixmap(pix);
    }
}

void ImageWithTextWidget::setText(const QString &text)
{
    textLabel->setText(text);
}

void ImageWithTextWidget::setCountdown(int seconds)
{
    countdownSeconds = seconds;

    if (seconds > 0) {
        countdownLabel->setText(QString::number(countdownSeconds));
        countdownLabel->show();
        countdownTimer->start(1000);
    } else {
        countdownTimer->stop();
        countdownLabel->hide();
        countdownLabel->clear();
    }
}

void ImageWithTextWidget::updateCountdown()
{
    countdownSeconds--;
    if (countdownSeconds >= 0) {
        countdownLabel->setText(QString::number(countdownSeconds));
    } else {
        countdownTimer->stop();
        countdownLabel->hide();
        countdownLabel->clear();
    }
}
