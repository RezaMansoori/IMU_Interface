#ifndef COUNTDOWNDIALOG_H
#define COUNTDOWNDIALOG_H

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QTimer>

class CountdownDialog : public QDialog {
    Q_OBJECT
public:
    explicit CountdownDialog(const QString& message, bool isDarkTheme, QWidget* parent = nullptr);
    void startCountdown();

signals:
    void countdownStarted();
    void countdownFinished();

private slots:
    void onReadyClicked();
    void updateCountdown();

private:
    QLabel* messageLabel;
    QLabel* timerLabel;
    QPushButton* readyButton;
    QTimer* countdownTimer;
    int countdownValue = 10;
    bool isDarkTheme;
    void applyTheme();
};

#endif // COUNTDOWNDIALOG_H
