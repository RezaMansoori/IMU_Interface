#ifndef EULERCHART_H
#define EULERCHART_H

#include <QWidget>
#include "components/imguiplot.h"

class EulerChart : public QWidget
{
    Q_OBJECT

public:
    explicit EulerChart(QWidget* parent = nullptr);
    void addEulerData(float roll, float pitch, float yaw);

    void setJointNum(int num);

private:
    ImGuiPlot* m_plot;
    int index;
};

#endif // EULERCHART_H
