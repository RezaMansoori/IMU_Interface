#include "eulerchart.h"
#include <QVBoxLayout>
#include <QTimer>

EulerChart::EulerChart(QWidget* parent)
    : QWidget(parent), index(0)
{
    QVBoxLayout* layout = new QVBoxLayout(this);
    m_plot = new ImGuiPlot(this);
    m_plot->setPlotCount(1);
    m_plot->setChartTitle(0, "Euler Angles (deg)");
    m_plot->setPlotRange(0, -180, 180);
    layout->addWidget(m_plot);
    setLayout(layout);

    // Update chart at 60Hz
    QTimer* timer = new QTimer(this);
    connect(timer, &QTimer::timeout, m_plot, QOverload<>::of(&ImGuiPlot::update));
    timer->start(1000 / 60);
}

void EulerChart::addEulerData(float roll, float pitch, float yaw)
{
    m_plot->addData(0, index++, roll, pitch, yaw);
}

void EulerChart::setJointNum(int num) {
    if (m_plot) {
        m_plot->setJointNum(num);
    }
}
