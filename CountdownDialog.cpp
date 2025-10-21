#include "CountdownDialog.h"

CountdownDialog::CountdownDialog(const QString& message, bool isDarkTheme, QWidget* parent)
    : QDialog(parent), isDarkTheme(isDarkTheme) {
    setWindowTitle("Calibration Test");
    setFixedSize(400, 250);  // اندازه بزرگ‌تر برای زیبایی بیشتر

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(20, 20, 20, 20);  // حاشیه بیشتر برای فضا
    layout->setSpacing(20);  // فاصله بیشتر بین المان‌ها

    messageLabel = new QLabel(message, this);
    messageLabel->setAlignment(Qt::AlignCenter);
    messageLabel->setStyleSheet("font: bold 18px 'Segoe UI';");  // فونت بزرگ‌تر و بولد
    layout->addWidget(messageLabel);

    timerLabel = new QLabel("", this);
    timerLabel->setAlignment(Qt::AlignCenter);
    timerLabel->setStyleSheet("font: bold 64px 'Segoe UI';");  // تایمر بزرگ‌تر و بولد برای دیجیتال بودن
    layout->addWidget(timerLabel);

    readyButton = new QPushButton("Ready", this);
    readyButton->setFixedHeight(50);  // ارتفاع باتن بیشتر
    readyButton->setCursor(Qt::PointingHandCursor);  // کرسر دست برای UX بهتر
    connect(readyButton, &QPushButton::clicked, this, &CountdownDialog::onReadyClicked);
    layout->addWidget(readyButton);

    countdownTimer = new QTimer(this);
    connect(countdownTimer, &QTimer::timeout, this, &CountdownDialog::updateCountdown);

    applyTheme();  // اعمال تم بعد از ساخت UI
}

void CountdownDialog::onReadyClicked() {
    countdownValue = 10;
    readyButton->hide();
    timerLabel->setText(QString::number(countdownValue));
    countdownTimer->start(1000); // هر ثانیه
    emit countdownStarted();
}

void CountdownDialog::updateCountdown() {
    countdownValue--;
    timerLabel->setText(QString::number(countdownValue));
    if (countdownValue <= 3) {
        timerLabel->setStyleSheet(timerLabel->styleSheet() + (isDarkTheme ? "color: #FF3F15;" : "color: #FF3F15;"));  // رنگ قرمز برای هیجان
    }
    if (countdownValue <= 0) {
        countdownTimer->stop();
        emit countdownFinished();
        accept();
    }
}

void CountdownDialog::startCountdown() {
    // اگر نیاز به شروع مستقیم باشه (نه استفاده می‌شه)
}

void CountdownDialog::applyTheme() {
    QString dialogStyle, labelStyle, buttonStyle;

    if (isDarkTheme) {
        // تم دارک (گرادیانت مشابه صفحات دیگر)
        dialogStyle = "QDialog { background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #232526, stop:1 #414345); border-radius: 15px; }";
        labelStyle = "color: #f8f8f8;";  // رنگ متن روشن
        buttonStyle = R"(
            QPushButton {
                background: #3e3e4e;
                color: #43cea2;
                border-radius: 12px;
                font: bold 16px 'Segoe UI';
                padding: 10px;
            }
            QPushButton:hover { background: #43cea2; color: #232526; }
            QPushButton:pressed { background: #4e4e5e; }
        )";
    } else {
        // تم لایت
        dialogStyle = "QDialog { background: white; border: 1px solid #d0d5dd; border-radius: 15px; }";
        labelStyle = "color: black;";  // رنگ متن تیره
        buttonStyle = R"(
            QPushButton {
                background: #e0e0e0;
                color: black;
                border: 1px solid #333;
                border-radius: 12px;
                font: bold 16px 'Segoe UI';
                padding: 10px;
            }
            QPushButton:hover { background: #333; color: white; }
            QPushButton:pressed { background: #ccc; }
        )";
    }

    setStyleSheet(dialogStyle);
    messageLabel->setStyleSheet(messageLabel->styleSheet() + labelStyle);  // ترکیب با استایل موجود
    timerLabel->setStyleSheet(timerLabel->styleSheet() + labelStyle);  // ترکیب با استایل موجود
    readyButton->setStyleSheet(buttonStyle);
}
