#ifndef IMGUIPLOT_H
#define IMGUIPLOT_H

#include <QtGui/QOpenGLExtraFunctions>
#include <QtOpenGLWidgets/QOpenGLWidget>

class QTimer;
class QEvent;

class ImGuiContext;
class ImPlotContext;

class ImGuiPlot : public QOpenGLWidget, protected QOpenGLExtraFunctions {
  Q_OBJECT

 public:
  explicit ImGuiPlot(QWidget* parent = nullptr);
  ~ImGuiPlot();

  void setChartTitle(int index, const QString&);
  QString chartTitle(int index) const;

  void setPlotRange(int index, double y_min, double y_max);
  QPair<double, double> plotRange(int index) const;

  void addData(int index, float x, float y1, float y2, float y3);
  void addData(int index, float x, const QVector3D& vec);

  void setPlotCount(int count);
  int plotCount() const { return m_plotCount; }
  void setJointNum(int num);


 protected:
  void initializeGL() override;
  void paintGL() override;
  void resizeGL(int w, int h) override;
  bool eventFilter(QObject* watched, QEvent* event) override;

 private:
  const static int data_size = 400;
  const static int default_frame_rate = 60;

 private:
  char m_titles[4][64];
  QPair<double, double> m_ranges[4];

  ImGuiContext* ctx;     // ImGui Context
  ImPlotContext* p_ctx;  // ImPlot Context

  bool g_MousePressed[3] = {false, false, false};

  float m_datas[4][4][data_size];
  float mouse_wheel;   // mouse vertical wheel
  float mouse_wheelH;  // mouse horizontal wheel

  int m_plotCount = 4;
  int joint_num;
};

#endif  // IMGUIPLOT_H
