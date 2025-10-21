#include "loginpage.h"
#include "helpdialog.h"
#include <QMessageBox>
#include <QCoreApplication>
#include <QFile>
#include <QDebug>

LoginPage::LoginPage(QWidget *parent)
    : QWidget(parent)
{
    QHBoxLayout *mainLayout = new QHBoxLayout(this);
    mainLayout->setContentsMargins(20, 20, 20, 20);
    mainLayout->setSpacing(20);

    QVBoxLayout *leftLayout = new QVBoxLayout();
    QVBoxLayout *rightLayout = new QVBoxLayout();

    // نوار بالا سمت راست شامل: Toggle و Help
    QHBoxLayout *topRightLayout = new QHBoxLayout();
    topRightLayout->setAlignment(Qt::AlignRight);
    topRightLayout->setSpacing(10);

    themeToggle = new QCheckBox(this);
    themeToggle->setCursor(Qt::PointingHandCursor);
    themeToggle->setFixedSize(60, 30);
    themeToggle->setChecked(true);
    themeToggle->setStyleSheet(R"(
        QCheckBox {
            background-color: #e0e0e0;
            border-radius: 15px;
        }
        QCheckBox::indicator {
            width: 24px;
            height: 24px;
            border-radius: 12px;
            background-color: white;
            margin: 3px;
            position: absolute;
        }
        QCheckBox::indicator:checked {
            margin-left: 33px;
            background-color: #ffd700;
        }
    )");
    connect(themeToggle, &QCheckBox::toggled, this, &LoginPage::toggleTheme);
    topRightLayout->addWidget(themeToggle);

    helpButton = new QPushButton("❓", this);
    helpButton->setCursor(Qt::PointingHandCursor);
    helpButton->setStyleSheet(R"(
        QPushButton {
            background: transparent;
            color: #0078d7;
            border: none;
            font: bold 14px 'Segoe UI';
        }
        QPushButton:hover {
            text-decoration: underline;
        }
    )");
    connect(helpButton, &QPushButton::clicked, this, []() {
        HelpDialog *dialog = new HelpDialog();
        dialog->exec();
    });
    topRightLayout->addSpacing(200);
    topRightLayout->addWidget(helpButton);

    rightLayout->addLayout(topRightLayout);

    // ویدیو
    QString videoPath = QCoreApplication::applicationDirPath() + "/login.avi";
    if (!QFile::exists(videoPath)) {
        qWarning() << "⚠️ Video file not found!";
        QLabel *errorLabel = new QLabel("Video file not found!");
        errorLabel->setStyleSheet("font: 16px 'Segoe UI'; color: #f8f8f8;");
        errorLabel->setAlignment(Qt::AlignCenter);
        leftLayout->addWidget(errorLabel, 1);
    } else {
        player = new QMediaPlayer(this);
        videoWidget = new QVideoWidget(this);
        player->setVideoOutput(videoWidget);
        player->setSource(QUrl::fromLocalFile(videoPath));
        player->setLoops(QMediaPlayer::Infinite);
        videoWidget->setStyleSheet("background: black;");
        videoWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        videoWidget->setAspectRatioMode(Qt::IgnoreAspectRatio);
        leftLayout->addWidget(videoWidget, 1);
        player->play();
    }

    // فرم ورود
    QVBoxLayout *controlsLayout = new QVBoxLayout();
    controlsLayout->setSpacing(20);

    QHBoxLayout *axisLayout = new QHBoxLayout();
    axisLabel = new QLabel("Select Axis Mode:");
    axisLabel->setStyleSheet("font: 16px 'Segoe UI'; color: black;");
    axisModeCombo = new QComboBox();
    axisModeCombo->addItem("6-Axis Mode", 6);
    axisModeCombo->addItem("9-Axis Mode", 9);
    axisModeCombo->setStyleSheet(R"(
        QComboBox {
            background: #e0e0e0;
            color: black;
            border: 1px solid #333;
            border-radius: 8px;
            font: 15px 'Segoe UI';
            padding: 5px;
        }
        QComboBox QAbstractItemView {
            background: #ffffff;
            color: black;
            selection-background-color: #cce5ff;
        }
    )");
    axisLayout->addWidget(axisLabel);
    axisLayout->addWidget(axisModeCombo);
    controlsLayout->addLayout(axisLayout);

    QHBoxLayout *imuLayout = new QHBoxLayout();
    imuLabel = new QLabel("Number of IMUs:");
    imuLabel->setStyleSheet("font: 16px 'Segoe UI'; color: black;");
    numImusCombo = new QComboBox();
    for (int i = 1; i <= 30; ++i)
        numImusCombo->addItem(QString::number(i), i);
    numImusCombo->setCurrentIndex(17);
    numImusCombo->setStyleSheet(R"(
        QComboBox {
            background: #e0e0e0;
            color: black;
            border: 1px solid #333;
            border-radius: 8px;
            font: 15px 'Segoe UI';
            padding: 5px;
        }
        QComboBox QAbstractItemView {
            background: #ffffff;
            color: black;
            selection-background-color: #cce5ff;
        }
    )");
    imuLayout->addWidget(imuLabel);
    imuLayout->addWidget(numImusCombo);
    controlsLayout->addLayout(imuLayout);

    loginButton = new QPushButton("Start");
    loginButton->setStyleSheet(R"(
        QPushButton {
            background: #e0e0e0;
            color: black;
            border: 1px solid #333;
            border-radius: 12px;
            min-height: 38px;
            font: 16px 'Segoe UI';
        }
        QPushButton:hover { background: #333; color: white; }
        QPushButton:pressed { background: #ccc; }
    )");
    connect(loginButton, &QPushButton::clicked, this, &LoginPage::handleLogin);
    controlsLayout->addWidget(loginButton);
    controlsLayout->addStretch();

    rightLayout->addLayout(controlsLayout, 1);

    mainLayout->addLayout(leftLayout, 2);
    mainLayout->addLayout(rightLayout, 1);

    toggleTheme();  // اعمال تم لایت در شروع
}

void LoginPage::handleLogin()
{
    if (player) player->stop();
    int axisMode = axisModeCombo->currentData().toInt();
    int numImus = numImusCombo->currentData().toInt();
    if (axisModeCombo->currentIndex() == -1 || numImusCombo->currentIndex() == -1) {
        QMessageBox::warning(this, "Error", "Please select axis and IMU count");
        return;
    }
    emit loginConfirmed(axisMode, numImus, isDarkTheme);
}

void LoginPage::toggleTheme()
{
    isDarkTheme = !themeToggle->isChecked();

    if (themeToggle->isChecked()) {
        setStyleSheet("QWidget { background: white; }");
        if (videoWidget) videoWidget->setStyleSheet("background: black;");
        axisLabel->setStyleSheet("font: 16px 'Segoe UI'; color: black;");
        imuLabel->setStyleSheet("font: 16px 'Segoe UI'; color: black;");
        loginButton->setStyleSheet(R"(
            QPushButton {
                background: #e0e0e0;
                color: black;
                border: 1px solid #333;
                border-radius: 12px;
                min-height: 38px;
                font: 16px 'Segoe UI';
            }
            QPushButton:hover { background: #333; color: white; }
            QPushButton:pressed { background: #ccc; }
        )");
    } else {
        setStyleSheet("QWidget { background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #232526, stop:1 #414345); }");
        if (videoWidget) videoWidget->setStyleSheet("background: black;");
        axisLabel->setStyleSheet("font: 16px 'Segoe UI'; color: #f8f8f8;");
        imuLabel->setStyleSheet("font: 16px 'Segoe UI'; color: #f8f8f8;");
        loginButton->setStyleSheet(R"(
            QPushButton {
                background: #3e3e4e;
                color: #43cea2;
                border-radius: 12px;
                min-height: 38px;
                font: 16px 'Segoe UI';
            }
            QPushButton:hover { background: #43cea2; color: #232526; }
            QPushButton:pressed { background: #4e4e5e; }
        )");
    }
}
