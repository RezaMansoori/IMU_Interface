#ifndef LOGINPAGE_H
#define LOGINPAGE_H

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QMediaPlayer>
#include <QVideoWidget>
#include <QCheckBox>

class LoginPage : public QWidget
{
    Q_OBJECT

public:
    explicit LoginPage(QWidget *parent = nullptr);

signals:
    void loginConfirmed(int axisMode, int numImus, bool isDarkTheme);

private slots:
    void handleLogin();
    void toggleTheme();

private:
    QComboBox *axisModeCombo;
    QComboBox *numImusCombo;
    QMediaPlayer *player;
    QVideoWidget *videoWidget;
    QLabel *axisLabel;
    QLabel *imuLabel;
    QPushButton *loginButton;
    QCheckBox *themeToggle;
    QPushButton *helpButton;
    bool isDarkTheme = false; // default = light
};

#endif // LOGINPAGE_H
