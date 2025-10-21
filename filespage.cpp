#include "filespage.h"
#include <QDir>
#include <QFileInfo>
#include <QDesktopServices>
#include <QUrl>
#include <QMessageBox>
#include <QInputDialog>
#include <QLineEdit>
#include <QDebug>

FilesPage::FilesPage(QWidget *parent, bool isDarkTheme)
    : QWidget(parent), isDarkTheme(isDarkTheme)
{
    initUI();

    // Initialize file watcher
    fileWatcher = new QFileSystemWatcher(this);
    QDir dir("data");
    if (!dir.exists()) {
        qDebug() << "Data directory does not exist, creating...";
        dir.mkpath(".");
    }
    fileWatcher->addPath("data");
    qDebug() << "Watching directory:" << fileWatcher->directories();
    connect(fileWatcher, &QFileSystemWatcher::directoryChanged, this, &FilesPage::updateFileList);

    // Apply theme and populate initial file list
    applyTheme();
    populateFiles();
}

void FilesPage::initUI()
{
    mainLayout = new QVBoxLayout(this);

    // Title label
    titleLabel = new QLabel("📁 Files");
    mainLayout->addWidget(titleLabel);

    // Tree widget for file list
    tree = new QTreeWidget();
    tree->setHeaderLabels({"Name", "Size", "Modified", "Actions"});
    tree->setColumnCount(4);
    tree->setColumnWidth(0, 300);
    tree->setColumnWidth(1, 100);
    tree->setColumnWidth(2, 200);
    tree->setColumnWidth(3, 100);
    tree->setSelectionMode(QAbstractItemView::SingleSelection); // Only one item can be selected
    mainLayout->addWidget(tree);

    // Debug item clicks
    connect(tree, &QTreeWidget::itemClicked, this, [](QTreeWidgetItem *item, int column) {
        qDebug() << "Item clicked:" << item->text(0) << ", column:" << column;
    });
}

void FilesPage::applyTheme()
{
    QString widgetStyle = isDarkTheme ?
                              "FilesPage { background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #232526, stop:1 #414345); }" :
                              "FilesPage { background: white; }";
    QString labelStyle = isDarkTheme ?
                             "QLabel { font: 22pt 'Segoe UI'; color: #43cea2; margin-bottom: 12px; }" :
                             "QLabel { font: 22pt 'Segoe UI'; color: black; margin-bottom: 12px; }";
    QString treeStyle = isDarkTheme ?
                            "QTreeWidget {"
                            "    background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #232526, stop:1 #414345);"
                            "    color: #f8f8f8;"
                            "    border-radius: 10px;"
                            "    font: 15pt 'Segoe UI';"
                            "}"
                            "QHeaderView::section {"
                            "    background: #232526;"
                            "    color: #43cea2;"
                            "    font: bold 13pt 'Segoe UI';"
                            "    padding: 8px;"
                            "}"
                            "QTreeWidget::item {"
                            "    height: 42px;"
                            "}"
                            "QTreeWidget::item:selected {"
                            "    background: #185a9d;"
                            "    color: #ffffff;"
                            "}" :
                            "QTreeWidget {"
                            "    background: #e0e0e0;"
                            "    color: black;"
                            "    border: 1px solid #333;"
                            "    border-radius: 10px;"
                            "    font: 15pt 'Segoe UI';"
                            "}"
                            "QHeaderView::section {"
                            "    background: #e0e0e0;"
                            "    color: black;"
                            "    border: 1px solid #333;"
                            "    font: bold 13pt 'Segoe UI';"
                            "    padding: 8px;"
                            "}"
                            "QTreeWidget::item {"
                            "    height: 42px;"
                            "}"
                            "QTreeWidget::item:selected {"
                            "    background: #333;"
                            "    color: white;"
                            "}";

    setStyleSheet(widgetStyle);
    titleLabel->setStyleSheet(labelStyle);
    tree->setStyleSheet(treeStyle);

    // Re-populate files to apply theme to buttons
    populateFiles();
}

void FilesPage::populateFiles()
{
    tree->clear();
    QDir dir("data");
    dir.setNameFilters(QStringList() << "*.csv");
    QFileInfoList fileList = dir.entryInfoList(QDir::Files, QDir::Time);
    qDebug() << "Populating files, count:" << fileList.size();

    for (const QFileInfo &fileInfo : fileList) {
        QTreeWidgetItem *item = new QTreeWidgetItem(tree);
        item->setText(0, fileInfo.fileName());
        item->setText(1, QString("%1 KB").arg(fileInfo.size() / 1024));
        item->setText(2, fileInfo.lastModified().toString("yyyy-MM-dd hh:mm:ss"));
        item->setData(3, Qt::UserRole, fileInfo.filePath());

        QWidget *buttonWidget = new QWidget();
        QHBoxLayout *buttonLayout = new QHBoxLayout(buttonWidget);
        buttonLayout->setContentsMargins(0, 0, 0, 0);

        auto createButton = [&](const QString &text, void (FilesPage::*slot)()) {
            QPushButton *btn = new QPushButton(text);
            btn->setStyleSheet(isDarkTheme ?
                                   "QPushButton { background: #3e3e4e; color: #43cea2; border-radius: 8px; font: 12px 'Segoe UI'; padding: 4px; }"
                                   "QPushButton:hover { background: #43cea2; color: #232526; }" :
                                   "QPushButton { background: #e0e0e0; color: black; border: 1px solid #333; border-radius: 8px; font: 12px 'Segoe UI'; padding: 4px; }"
                                   "QPushButton:hover { background: #333; color: white; }");
            btn->setProperty("filePath", fileInfo.filePath()); // Store file path in button
            connect(btn, &QPushButton::clicked, this, slot);
            buttonLayout->addWidget(btn);
        };

        createButton("Open", &FilesPage::openFile);
        createButton("Delete", &FilesPage::deleteFile);
        createButton("Rename", &FilesPage::renameFile);
        createButton("Open Path", &FilesPage::openFileLocation);

        tree->setItemWidget(item, 3, buttonWidget);
    }
}

void FilesPage::addFileToList(const QString &fileName)
{
    QFileInfo fileInfo(fileName);
    if (fileInfo.exists() && fileInfo.suffix() == "csv") {
        qDebug() << "Adding file to list:" << fileInfo.fileName();
        QTreeWidgetItem *item = new QTreeWidgetItem(tree);
        item->setText(0, fileInfo.fileName());
        item->setText(1, QString("%1 KB").arg(fileInfo.size() / 1024));
        item->setText(2, fileInfo.lastModified().toString("yyyy-MM-dd hh:mm:ss"));
        item->setData(3, Qt::UserRole, fileInfo.filePath());

        QWidget *buttonWidget = new QWidget();
        QHBoxLayout *buttonLayout = new QHBoxLayout(buttonWidget);
        buttonLayout->setContentsMargins(0, 0, 0, 0);

        auto createButton = [&](const QString &text, void (FilesPage::*slot)()) {
            QPushButton *btn = new QPushButton(text);
            btn->setStyleSheet(isDarkTheme ?
                                   "QPushButton { background: #3e3e4e; color: #43cea2; border-radius: 8px; font: 12px 'Segoe UI'; padding: 4px; }"
                                   "QPushButton:hover { background: #43cea2; color: #232526; }" :
                                   "QPushButton { background: #e0e0e0; color: black; border: 1px solid #333; border-radius: 8px; font: 12px 'Segoe UI'; padding: 4px; }"
                                   "QPushButton:hover { background: #333; color: white; }");
            btn->setProperty("filePath", fileInfo.filePath()); // Store file path in button
            connect(btn, &QPushButton::clicked, this, slot);
            buttonLayout->addWidget(btn);
        };

        createButton("Open", &FilesPage::openFile);
        createButton("Delete", &FilesPage::deleteFile);
        createButton("Rename", &FilesPage::renameFile);
        createButton("Open Path", &FilesPage::openFileLocation);

        tree->setItemWidget(item, 3, buttonWidget);
    }
}

void FilesPage::updateFileList()
{
    qDebug() << "Updating file list";
    populateFiles();
}

void FilesPage::openFile()
{
    qDebug() << "openFile called";
    QPushButton *button = qobject_cast<QPushButton*>(sender());
    if (!button) {
        qDebug() << "No valid button sender";
        QMessageBox::warning(this, "Error", "Invalid button action.");
        return;
    }

    QString filePath = button->property("filePath").toString();
    qDebug() << "Attempting to open file:" << filePath;
    if (!QDesktopServices::openUrl(QUrl::fromLocalFile(filePath))) {
        QMessageBox::warning(this, "Error", "Failed to open file: " + filePath);
    }
}

void FilesPage::deleteFile()
{
    qDebug() << "deleteFile called";
    QPushButton *button = qobject_cast<QPushButton*>(sender());
    if (!button) {
        qDebug() << "No valid button sender";
        QMessageBox::warning(this, "Error", "Invalid button action.");
        return;
    }

    QString filePath = button->property("filePath").toString();
    if (QMessageBox::question(this, "Delete", "Are you sure you want to delete this file?") == QMessageBox::Yes) {
        QFile file(filePath);
        if (file.remove()) {
            qDebug() << "File deleted:" << filePath;
            updateFileList();
        } else {
            QMessageBox::warning(this, "Error", "Could not delete file.");
        }
    }
}

void FilesPage::renameFile()
{
    qDebug() << "renameFile called";
    QPushButton *button = qobject_cast<QPushButton*>(sender());
    if (!button) {
        qDebug() << "No valid button sender";
        QMessageBox::warning(this, "Error", "Invalid button action.");
        return;
    }

    QString oldPath = button->property("filePath").toString();
    QString oldName = QFileInfo(oldPath).fileName();

    QString newName = QInputDialog::getText(this, "Rename File", "Enter new file name:", QLineEdit::Normal, oldName);
    if (newName.isEmpty()) return;

    if (!newName.endsWith(".csv", Qt::CaseInsensitive))
        newName += ".csv";

    QString newPath = QFileInfo(oldPath).absoluteDir().filePath(newName);
    if (QFile::rename(oldPath, newPath)) {
        qDebug() << "File renamed from" << oldPath << "to" << newPath;
        updateFileList();
    } else {
        QMessageBox::warning(this, "Error", "Could not rename file.");
    }
}

void FilesPage::openFileLocation()
{
    qDebug() << "openFileLocation called";
    QPushButton *button = qobject_cast<QPushButton*>(sender());
    if (!button) {
        qDebug() << "No valid button sender";
        QMessageBox::warning(this, "Error", "Invalid button action.");
        return;
    }

    QString filePath = button->property("filePath").toString();
    QString folderPath = QFileInfo(filePath).absolutePath();
    qDebug() << "Opening file location:" << folderPath;

#if defined(Q_OS_WIN)
    QDesktopServices::openUrl(QUrl::fromLocalFile(folderPath));
#elif defined(Q_OS_MAC)
    QProcess::startDetached("open", QStringList() << folderPath);
#else
    QProcess::startDetached("xdg-open", QStringList() << folderPath);
#endif
}
