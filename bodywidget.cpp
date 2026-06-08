#include "BodyWidget.h"
#include <QPainter>
#include <QMouseEvent>
#include <QLineF>

BodyWidget::BodyWidget(QWidget *parent)
    : QWidget(parent)
{
    bodyImage = QPixmap("C:/Users/User/OneDrive/Desktop/qt project/IMU_Interfacevfinal/data/smpl2.png");

    QVector<QPoint> pixelPoints = {
        {150, 140}, // pelvis
        {135, 145}, // LHip
        {165, 145}, // RHip
        {150, 120}, // Spine1
        {135, 205}, // LKnee
        {165, 205}, // RKnee
        {150, 72}, // Spine2
        {135, 270}, // LAnkle
        {165, 270}, // RAnkle
        {150, 57}, // Spine3
        {130, 285}, // LFoot
        {170, 285}, // RFoot
        {150, 48}, // Neck
        {138, 62}, // LClavicle
        {162, 62}, // RClavicle
        {150, 28}, // Head
        {120, 57}, // LShoulder
        {180, 57}, // RShoulder
        {75, 60}, // LElbow
        {225, 60}, // RElbow
        {38, 62}, // LWrist
        {262, 62}, // RWrist
        {20, 62}, // LHand
        {280, 62} //RHand
    };

    for (const QPoint &pt : pixelPoints) {
        jointPointsPercent.append(QPointF(pt.x() / 300.0, pt.y() / 300.0));
    }

}

void BodyWidget::setVisibleJoints(const QSet<int> &joints) {
    visibleJoints = joints;
    update();
}

void BodyWidget::setJointNames(const QStringList &names) {
    jointNames = names;
}

void BodyWidget::paintEvent(QPaintEvent *event) {
    QPainter p(this);

    p.drawPixmap(0, 0, bodyImage);
    for (int i = 0; i < jointPointsPercent.size(); ++i) {
        if (!visibleJoints.isEmpty() && !visibleJoints.contains(i)) {
            continue;
        }
        QPointF percent = jointPointsPercent[i];
        QPointF pos(bodyImage.width() * percent.x(),
                    bodyImage.height() * percent.y());

        if (selectedJoints.contains(i)) {
            p.setBrush(Qt::red);
            p.setPen(Qt::NoPen);
            p.drawEllipse(pos, 10, 10);
        } else {
            p.setBrush(Qt::blue);
            p.setPen(Qt::NoPen);
            p.drawEllipse(pos, 6, 6);
        }
    }
}

void BodyWidget::mousePressEvent(QMouseEvent *event) {
    for (int i = 0; i < jointPointsPercent.size(); ++i) {
        if (!visibleJoints.isEmpty() && !visibleJoints.contains(i)) {
            continue;
        }
        QPointF percentPos = jointPointsPercent[i];
        QPoint pixelPos(percentPos.x() * width(), percentPos.y() * height());

        if (QLineF(event->pos(), pixelPos).length() < 10) {
            if (selectionMode == SingleSelection) {
                selectedJoints.clear();
                selectedJoints.insert(i);
                emit jointSelected(i);
            } else {
                if (selectedJoints.contains(i))
                    selectedJoints.remove(i);
                else
                    selectedJoints.insert(i);
                emit jointsSelected(selectedJoints);
            }
            update();
            break;
        }
    }
}

void BodyWidget::setSelectionMode(SelectionMode mode) {
    selectionMode = mode;
    selectedJoints.clear();
    update();
}

QSet<int> BodyWidget::getSelectedJoints(){
    return selectedJoints;
}
