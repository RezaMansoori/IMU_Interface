#ifndef REGISTERPAGE_H
#define REGISTERPAGE_H

#include <QWidget>
#include <QStackedLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QTextEdit>

class RegisterPage : public QWidget
{
    Q_OBJECT

public:
    explicit RegisterPage(QWidget *parent = nullptr);

signals:
    void proceedToLogin();

private slots:
    void showDetailedRegister();
    void backToInitialForm();
    void handleRegister();
    void handleLogin();

private:
    QStackedLayout *stackedLayout;

    QWidget *initialForm;
    QLineEdit *usernameInput;
    QLineEdit *passwordInput;

    QWidget *detailedForm;
    QLineEdit *firstNameInput;
    QLineEdit *lastNameInput;
    QLineEdit *instituteInput;
    QTextEdit *bioInput;
    QLineEdit *emailInput;
    QLineEdit *mobileInput;
    QLineEdit *passwordRegInput;
    QLineEdit *confirmPasswordInput;
    QLineEdit *licenseKeyInput;
};

#endif // REGISTERPAGE_H
