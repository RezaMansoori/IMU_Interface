#ifndef FILESPAGE_H
#define FILESPAGE_H

#include <QWidget>
#include <QVBoxLayout>
#include <QTreeWidget>
#include <QLabel>
#include <QPushButton>
#include <QFileSystemWatcher>

class FilesPage : public QWidget
{
    Q_OBJECT

public:
    explicit FilesPage(QWidget *parent = nullptr, bool isDarkTheme = true);
    void applyTheme();

public slots:
    void addFileToList(const QString &fileName); // برای اضافه کردن فایل جدید

private slots:
    void openFile(); // برای باز کردن فایل CSV
    void updateFileList(); // برای به‌روزرسانی لیست فایل‌ها
    void deleteFile();
    void renameFile();
    void openFileLocation();

private:
    void initUI();
    void populateFiles(); // برای پر کردن لیست فایل‌ها

    QVBoxLayout *mainLayout;
    QTreeWidget *tree;
    QLabel *titleLabel;
    QFileSystemWatcher *fileWatcher; // برای نظارت بر تغییرات در پوشه data
    bool isDarkTheme;
};

#endif // FILESPAGE_H
