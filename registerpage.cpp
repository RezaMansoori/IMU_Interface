#include "registerpage.h"
#include <QVBoxLayout>
#include <QFormLayout>
#include <QMessageBox>
#include <QLabel>
#include <QPixmap>
#include <QGraphicsDropShadowEffect>

void showStyledMessageBox(QWidget *parent, QMessageBox::Icon icon, const QString &title, const QString &message) {
    QMessageBox msgBox(parent);
    msgBox.setIcon(icon);
    msgBox.setWindowTitle(title);
    msgBox.setText(message);

    msgBox.setStyleSheet(R"(
        QMessageBox {
            background-color: #1d2939;
        }
        QLabel {
            color: white;
            background: transparent;
            font: 14px 'Segoe UI';
        }
        QPushButton {
            background-color: #00acc1;
            color: white;
            border: none;
            padding: 6px 12px;
            border-radius: 6px;
        }
        QPushButton:hover {
            background-color: #0097a7;
        }
    )");

    msgBox.exec();
}


RegisterPage::RegisterPage(QWidget *parent)
    : QWidget(parent)
{
    stackedLayout = new QStackedLayout(this);
    this->setStyleSheet("background-color: #f2f4f7;");

    // -------- فرم ساده login/register --------
    initialForm = new QWidget(this);
    QVBoxLayout *initialLayout = new QVBoxLayout(initialForm);
    initialLayout->setAlignment(Qt::AlignCenter);

    QWidget *loginBox = new QWidget();
    loginBox->setFixedWidth(400);
    loginBox->setStyleSheet("background-color: white; border-radius: 16px; padding: 24px;");
    QGraphicsDropShadowEffect *shadow = new QGraphicsDropShadowEffect();
    shadow->setBlurRadius(25);
    shadow->setOffset(0, 6);
    shadow->setColor(QColor(0, 0, 0, 50));
    loginBox->setGraphicsEffect(shadow);

    QVBoxLayout *boxLayout = new QVBoxLayout(loginBox);

    QLabel *logoLabel = new QLabel(this);
    QPixmap logoPixmap("icons/fum-care-high-quality.png");
    if (!logoPixmap.isNull())
        logoLabel->setPixmap(logoPixmap.scaledToWidth(260, Qt::SmoothTransformation));
    else
        logoLabel->setText("⚠️ Logo not found!");
    logoLabel->setAlignment(Qt::AlignCenter);
    boxLayout->addWidget(logoLabel);

    usernameInput = new QLineEdit();
    usernameInput->setPlaceholderText("Username");

    passwordInput = new QLineEdit();
    passwordInput->setPlaceholderText("Password");
    passwordInput->setEchoMode(QLineEdit::Password);

    QString styleLineEdit = R"(
        QLineEdit {
            background-color: #f7f9fc;
            border: 1px solid #d0d5dd;
            border-radius: 8px;
            padding: 6px;
            font: 14px 'Segoe UI';
            color: #1d2939;
        }
        QLineEdit::placeholder {
            color: #98a2b3;
        }
    )";


    QString styleButtonPrimary = R"(
        QPushButton {
            background-color: #00acc1;
            color: white;
            border-radius: 8px;
            padding: 10px;
            font: bold 15px 'Segoe UI';
        }
        QPushButton:hover {
            background-color: #007c91;
        }
    )";

    QString styleButtonOutline = R"(
        QPushButton {
            background-color: white;
            color: #00acc1;
            border: 2px solid #00acc1;
            border-radius: 8px;
            padding: 10px;
            font: bold 15px 'Segoe UI';
        }
        QPushButton:hover {
            background-color: #e0f7fa;
        }
    )";

    usernameInput->setStyleSheet(styleLineEdit);
    passwordInput->setStyleSheet(styleLineEdit);

    QPushButton *loginBtn = new QPushButton("Login");
    QPushButton *registerBtn = new QPushButton("Register");
    loginBtn->setStyleSheet(styleButtonPrimary);
    registerBtn->setStyleSheet(styleButtonOutline);

    connect(loginBtn, &QPushButton::clicked, this, &RegisterPage::handleLogin);
    connect(registerBtn, &QPushButton::clicked, this, &RegisterPage::showDetailedRegister);

    boxLayout->addSpacing(10);
    boxLayout->addWidget(usernameInput);
    boxLayout->addWidget(passwordInput);
    boxLayout->addSpacing(10);
    boxLayout->addWidget(loginBtn);
    boxLayout->addWidget(registerBtn);
    boxLayout->addSpacing(10);

    initialLayout->addWidget(loginBox);
    initialForm->setLayout(initialLayout);

    // -------- فرم ثبت‌نام کامل --------
    detailedForm = new QWidget(this);
    QVBoxLayout *registerLayout = new QVBoxLayout(detailedForm);
    registerLayout->setAlignment(Qt::AlignCenter);

    QWidget *formBox = new QWidget();
    formBox->setFixedWidth(500);
    formBox->setStyleSheet("background-color: white; border-radius: 16px; padding: 12px;");
    QGraphicsDropShadowEffect *regShadow = new QGraphicsDropShadowEffect();
    regShadow->setBlurRadius(25);
    regShadow->setOffset(0, 6);
    regShadow->setColor(QColor(0, 0, 0, 50));
    formBox->setGraphicsEffect(regShadow);

    QFormLayout *formLayout = new QFormLayout(formBox);
    formLayout->setSpacing(0);

    auto label = [](const QString &text) {
        QLabel *lbl = new QLabel(text);
        lbl->setStyleSheet("color: #00acc1; font: bold 13px 'Segoe UI';");
        return lbl;
    };

    auto styledLineEdit = [=]() {
        QLineEdit *le = new QLineEdit();
        le->setStyleSheet(styleLineEdit);
        return le;
    };

    firstNameInput = styledLineEdit();
    lastNameInput = styledLineEdit();
    instituteInput = styledLineEdit();
    emailInput = styledLineEdit();
    mobileInput = styledLineEdit();
    passwordRegInput = styledLineEdit();
    confirmPasswordInput = styledLineEdit();
    licenseKeyInput = styledLineEdit();

    passwordRegInput->setEchoMode(QLineEdit::Password);
    confirmPasswordInput->setEchoMode(QLineEdit::Password);

    bioInput = new QTextEdit();
    bioInput->setStyleSheet(R"(
        QTextEdit {
            background-color: #f7f9fc;
            border: 1px solid #d0d5dd;
            border-radius: 8px;
            padding: 6px;
            font: 14px 'Segoe UI';
            color: #1d2939;
        }
    )");


    QPushButton *submitBtn = new QPushButton("Register");
    QPushButton *backBtn = new QPushButton("Back");
    submitBtn->setStyleSheet(styleButtonPrimary);
    backBtn->setStyleSheet(styleButtonOutline);

    connect(submitBtn, &QPushButton::clicked, this, &RegisterPage::handleRegister);
    connect(backBtn, &QPushButton::clicked, this, &RegisterPage::backToInitialForm);

    formLayout->addRow(label("First Name *"), firstNameInput);
    formLayout->addRow(label("Last Name *"), lastNameInput);
    formLayout->addRow(label("Institute/Department"), instituteInput);
    formLayout->addRow(label("Biography"), bioInput);
    formLayout->addRow(label("Email Address"), emailInput);
    formLayout->addRow(label("Mobile Number *"), mobileInput);
    formLayout->addRow(label("Password *"), passwordRegInput);
    formLayout->addRow(label("Confirm Password *"), confirmPasswordInput);
    formLayout->addRow(label("Licence Key *"), licenseKeyInput);
    formLayout->addRow(submitBtn, backBtn);

    registerLayout->addWidget(formBox);
    detailedForm->setLayout(registerLayout);

    // اضافه کردن به stackedLayout
    stackedLayout->addWidget(initialForm);
    stackedLayout->addWidget(detailedForm);
    stackedLayout->setCurrentWidget(initialForm);
}

void RegisterPage::showDetailedRegister()
{
    stackedLayout->setCurrentWidget(detailedForm);
}

void RegisterPage::backToInitialForm()
{
    stackedLayout->setCurrentWidget(initialForm);
}

void RegisterPage::handleRegister()
{
    if (firstNameInput->text().isEmpty() ||
        lastNameInput->text().isEmpty() ||
        mobileInput->text().isEmpty() ||
        passwordRegInput->text().isEmpty() ||
        confirmPasswordInput->text().isEmpty() ||
        licenseKeyInput->text().isEmpty()) {
        showStyledMessageBox(this, QMessageBox::Warning, "Incomplete", "Please fill in all required fields (*)");
        return;
    }

    if (passwordRegInput->text() != confirmPasswordInput->text()) {
        showStyledMessageBox(this, QMessageBox::Warning, "Mismatch", "Passwords do not match.");
        return;
    }

    emit proceedToLogin();
}

void RegisterPage::handleLogin()
{
    if (usernameInput->text().isEmpty() || passwordInput->text().isEmpty()) {
        showStyledMessageBox(this, QMessageBox::Warning, "Empty", "Username and password required.");
        return;
    }

    emit proceedToLogin();
}
