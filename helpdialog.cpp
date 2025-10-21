#include "helpdialog.h"
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QDesktopServices>
#include <QUrl>

HelpDialog::HelpDialog(QWidget *parent) : QDialog(parent)
{
    setWindowTitle("Help & Support");
    setFixedSize(400, 300);
    setWindowModality(Qt::ApplicationModal);
    setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);

    QVBoxLayout *layout = new QVBoxLayout(this);

    QLabel *intro = new QLabel("This software helps you configure and visualize IMU data.\n\nIf you need help:");
    intro->setWordWrap(true);
    layout->addWidget(intro);

    QLabel *manual = new QLabel("<a href=\"https://example.com/user-manual\">📘 User Manual</a>");
    manual->setTextInteractionFlags(Qt::TextBrowserInteraction);
    manual->setOpenExternalLinks(true);
    layout->addWidget(manual);

    QLabel *email = new QLabel("📧 Support: <a href=\"mailto:support@example.com\">support@example.com</a>");
    email->setTextInteractionFlags(Qt::TextBrowserInteraction);
    email->setOpenExternalLinks(true);
    layout->addWidget(email);

    QLabel *info = new QLabel("\nFor any issues or feedback, contact our team.\nThank you for using our software.");
    info->setWordWrap(true);
    layout->addWidget(info);

    QPushButton *closeBtn = new QPushButton("Close");
    connect(closeBtn, &QPushButton::clicked, this, &QDialog::accept);
    layout->addWidget(closeBtn, 0, Qt::AlignRight);
}
