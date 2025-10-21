#ifndef BODYWIDGET_H
#define BODYWIDGET_H
#include <QWidget>
#include <QPixmap>
#include <QVector>
#include <QPoint>

class BodyWidget : public QWidget {
    Q_OBJECT
public:
    enum SelectionMode {
        SingleSelection,
        MultiSelection
    };

    explicit BodyWidget(QWidget *parent = nullptr);
    void setJointNames(const QStringList &names);
    void setSelectedJoints(const QSet<int> &joints);
    void setSelectionMode(SelectionMode mode);
    QSet<int> getSelectedJoints();
    void setVisibleJoints(const QSet<int> &joints);

signals:
    void jointSelected(int index);
    void jointsSelected(const QSet<int> &indices);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;

private:
    QPixmap bodyImage;
    QVector<QPoint> jointPoints;
    QVector<QPointF> jointPointsPercent;
    QStringList jointNames;
    QVector<QPointF> jointPositions;
    QSet<int> selectedJoints;
    SelectionMode selectionMode = SingleSelection;
    QSet<int> visibleJoints;
};

#endif // BODYWIDGET_H
