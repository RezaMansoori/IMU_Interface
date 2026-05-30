#include "components/imguiplot.h"

#include <3rdparty/imgui/imgui.h>               /// IMGUI main header include
#include <3rdparty/imgui//backends/imgui_impl_opengl3.h>  /// IMGUI backend include
#include <3rdparty/imgui/imgui_internal.h>  /// IMGUI internal header for ImGui::SetCurrentContext()
#include <3rdparty/implot/implot.h>          /// IMPLOT main header include
#include <3rdparty/implot/implot_internal.h>  /// IMPLOT internal header for ImPlot::SetCurrentContext()

#include <QtCore/QEvent>
#include <QtCore/QTimer>
#include <QtGui/QMouseEvent>
#include <QtGui/QWheelEvent>

ImGuiPlot::ImGuiPlot(QWidget* parent)
    : QOpenGLWidget(parent), ctx(nullptr), p_ctx(nullptr) {
  // set the titles tonull for preventing posibble errors such memory errors

  for (int k = 0; k < 4; ++k) {
    m_titles[k][0] = ' ';
    m_titles[k][1] = '\0';
    m_ranges[k] = {-1, 1};
    // initialize data to a default values
    for (int i = 0; i < data_size; ++i) m_datas[k][0][i] = i;
    for (int j = 1; j < 4; j++)
      std::memset(m_datas[k][j], 0, sizeof(float) * data_size);
  }
  // clear margins and install event filter
  setContentsMargins(5, 5, 5, 5);
  installEventFilter(this);
}

ImGuiPlot::~ImGuiPlot() {
  makeCurrent();  // Ensure the OpenGL context is current

  // Shutdown ImGui OpenGL3 implementation
  ImGui_ImplOpenGL3_Shutdown();

  // Destroy ImPlot context
  if (p_ctx) {
    ImPlot::DestroyContext(p_ctx);
    p_ctx = nullptr;
  }

  // Destroy ImGui context
  if (ctx) {
    ImGui::DestroyContext(ctx);
    ctx = nullptr;
  }

  doneCurrent();  // Release the OpenGL context
}

void ImGuiPlot::setChartTitle(int index, const QString& title) {
  Q_ASSERT(title.length() > 0);
  Q_ASSERT(index >= 0 && index < 4);
  std::strcpy(m_titles[index], title.toStdString().c_str());
}

QString ImGuiPlot::chartTitle(int index) const {
  Q_ASSERT(index >= 0 && index < 4);
  return QString::fromStdString(m_titles[index]);
}

void ImGuiPlot::setPlotRange(int index, double y_min, double y_max) {
  Q_ASSERT(index >= 0 && index < 4);
  Q_ASSERT(y_min < y_max);
  m_ranges[index] = {y_min, y_max};
}

QPair<double, double> ImGuiPlot::plotRange(int index) const {
  Q_ASSERT(index >= 0 && index < 4);
  return m_ranges[index];
}

void ImGuiPlot::addData(int index, float x, float y1, float y2, float y3) {
  Q_ASSERT(index < 4);
  // left shift old data
  for (int i = 0; i < 4; i++)
    std::memmove(m_datas[index][i], m_datas[index][i] + 1,
                 (data_size - 1) * sizeof(float));

  // insert new data at end
  m_datas[index][0][data_size - 1] = x;
  m_datas[index][1][data_size - 1] = y1;
  m_datas[index][2][data_size - 1] = y2;
  m_datas[index][3][data_size - 1] = y3;
}

void ImGuiPlot::addData(int index, float x, const QVector3D& vec) {
  addData(index, x, vec.x(), vec.y(), vec.z());
}

void ImGuiPlot::initializeGL() {
  initializeOpenGLFunctions();

  ctx = ImGui::CreateContext();
  ImGui::SetCurrentContext(ctx);

  // Create ImPlot context and set global styles
  p_ctx = ImPlot::CreateContext();
  ImPlot::SetCurrentContext(p_ctx);
  ImPlot::GetStyle().PlotBorderSize = 0.5f;
  ImPlot::GetStyle().PlotPadding = ImVec2(10, 10);
  ImPlot::GetStyle().LineWeight = 2.0f;

  if (ImGui::GetIO().BackendRendererUserData == nullptr) {
    ImGui_ImplOpenGL3_Init();
    ImGui::GetIO().IniFilename = nullptr;
    ImGui::GetStyle().WindowPadding = ImVec2(0, 0);
  }

  // Set up ImGui style
  ImGui::StyleColorsLight();
  ImGui::GetStyle().ScaleAllSizes(devicePixelRatio());
}

void ImGuiPlot::paintGL() {
  // set current context
  ImGui::SetCurrentContext(ctx);
  ImPlot::SetCurrentContext(p_ctx);
  // Clear the background
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);
  // save a reference to ImGuiIO
  ImGuiIO& io = ImGui::GetIO();
  // create new ImGuiOpenGL Frame
  ImGui_ImplOpenGL3_NewFrame();

  // handle mouse position
  if (isActiveWindow()) {
    const QPoint pos = mapFromGlobal(QCursor::pos());
    io.MousePos = ImVec2(pos.x(), pos.y());
  } else {
    io.MousePos = ImVec2(-1, -1);
  }
  // apply the mouse key's states
  for (int i = 0; i < 3; i++) {
    io.MouseDown[i] = g_MousePressed[i];
  }
  // apply the mouse wheel state
  io.MouseWheelH = mouse_wheelH;
  io.MouseWheel = mouse_wheel;
  mouse_wheelH = mouse_wheel = 0;
  // save the available size in a temporary variable
  ImVec2 size = io.DisplaySize;
  // create new ImGui Frame
  ImGui::NewFrame();

  char buffer[16];
  for (int i = 0; i < m_plotCount; ++i) {
    // set the window location for create the 2x2 grid view
    // i = 0 -> (0  , 0  ) >
    // i = 1 -> (w/2, 0  ) => [  w/2 for odd  i  ]
    // i = 2 -> (0  , h/2) => [  h/2 for j >= 2  ]
    // i = 3 -> (w/2, h/2) >
    if (m_plotCount == 1) {
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(size);
    } else{
        ImGui::SetNextWindowPos(
            ImVec2(i % 2 ? size.x / 2 : 0, i < 2 ? 0 : size.y / 2));
        ImGui::SetNextWindowSize(ImVec2(size.x / 2, size.y / 2));
    }
    snprintf(buffer, 16, "Plot %d", i + 1);
    // Begin Drawing
    ImGui::Begin(buffer, nullptr,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);

    ImPlot::SetNextAxisToFit(ImAxis_X1);
    ImPlot::SetNextAxisLimits(ImAxis_Y1, m_ranges[i].first, m_ranges[i].second,
                              ImPlotCond_Once);
    if (ImPlot::BeginPlot(m_titles[i], ImVec2(-1, -1),
                          ImPlotFlags_Crosshairs)) {
      ImPlot::SetupAxisTicks(ImAxis_X1, nullptr, 0);
        if(m_plotCount==1){
          switch (joint_num) {
          case 0:
              ImPlot::PlotLine("flexion/extension(x)", m_datas[i][0], m_datas[i][1], data_size);
              ImPlot::PlotLine("endo/exo(y)", m_datas[i][0], m_datas[i][2], data_size);
              ImPlot::PlotLine("abduction/adduction(z)", m_datas[i][0], m_datas[i][3], data_size);
              break;
          case 1:
              ImPlot::PlotLine("flexion/extension(x)", m_datas[i][0], m_datas[i][1], data_size);
              ImPlot::PlotLine("endo/exo(y)", m_datas[i][0], m_datas[i][2], data_size);
              ImPlot::PlotLine("abduction/adduction(z)", m_datas[i][0], m_datas[i][3], data_size);
              break;
          case 2:
              ImPlot::PlotLine("flexion/extension(x)", m_datas[i][0], m_datas[i][1], data_size);
              ImPlot::PlotLine("endo/exo(y)", m_datas[i][0], m_datas[i][2], data_size);
              ImPlot::PlotLine("abduction/adduction(z)", m_datas[i][0], m_datas[i][3], data_size);
              break;
          case 3:
              ImPlot::PlotLine("flexion/extension(x)", m_datas[i][0], m_datas[i][1], data_size);
              ImPlot::PlotLine("endo/exo(y)", m_datas[i][0], m_datas[i][2], data_size);
              ImPlot::PlotLine("abduction/adduction(z)", m_datas[i][0], m_datas[i][3], data_size);
              break;
          case 4:
              ImPlot::PlotLine("flexion/extension(x)", m_datas[i][0], m_datas[i][1], data_size);
              ImPlot::PlotLine("endo/exo(y)", m_datas[i][0], m_datas[i][2], data_size);
              ImPlot::PlotLine("abduction/adduction(z)", m_datas[i][0], m_datas[i][3], data_size);
              break;
          case 5:
              ImPlot::PlotLine("flexion/extension(x)", m_datas[i][0], m_datas[i][1], data_size);
              ImPlot::PlotLine("endo/exo(y)", m_datas[i][0], m_datas[i][2], data_size);
              ImPlot::PlotLine("abduction/adduction(z)", m_datas[i][0], m_datas[i][3], data_size);
              break;
          case 6:
              ImPlot::PlotLine("flexion/extension(x)", m_datas[i][0], m_datas[i][1], data_size);
              ImPlot::PlotLine("endo/exo(y)", m_datas[i][0], m_datas[i][2], data_size);
              ImPlot::PlotLine("abduction/adduction(z)", m_datas[i][0], m_datas[i][3], data_size);
              break;
          default:
              ImPlot::PlotLine("x", m_datas[i][0], m_datas[i][1], data_size);
              ImPlot::PlotLine("y", m_datas[i][0], m_datas[i][2], data_size);
              ImPlot::PlotLine("z", m_datas[i][0], m_datas[i][3], data_size);
              break;
          }
        }else{
            ImPlot::PlotLine("x", m_datas[i][0], m_datas[i][1], data_size);
            ImPlot::PlotLine("y", m_datas[i][0], m_datas[i][2], data_size);
            ImPlot::PlotLine("z", m_datas[i][0], m_datas[i][3], data_size);
        }
      ImPlot::EndPlot();
    }

    ImGui::End();
  }
  // End Drawing

  // Render the Data and creates the drawData
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void ImGuiPlot::resizeGL(int w, int h) {
  // resize by device DPI
  const qreal retinaScale = devicePixelRatioF();
  ImGui::SetCurrentContext(ctx);

  ImGuiIO& io = ImGui::GetIO();
  io.DisplaySize = ImVec2(float(w), float(h));
  io.DisplayFramebufferScale = ImVec2(retinaScale, retinaScale);

  glViewport(0, 0, w * retinaScale, h * retinaScale);
}

bool ImGuiPlot::eventFilter(QObject* watched, QEvent* event) {
  if (watched == this) {
    switch (event->type()) {
      case QEvent::MouseButtonDblClick:
      case QEvent::MouseButtonPress:
      case QEvent::MouseButtonRelease: {
        QMouseEvent* mouse_event = static_cast<QMouseEvent*>(event);
        g_MousePressed[0] = mouse_event->buttons() & Qt::LeftButton;
        g_MousePressed[1] = mouse_event->buttons() & Qt::RightButton;
        g_MousePressed[2] = mouse_event->buttons() & Qt::MiddleButton;
        break;
      }
      case QEvent::Wheel: {
        QWheelEvent* wheel_event = static_cast<QWheelEvent*>(event);
        ImGui::SetCurrentContext(ctx);

        // Handle horizontal component
        if (wheel_event->pixelDelta().x()) {
          mouse_wheelH +=
              wheel_event->pixelDelta().x() / (ImGui::GetTextLineHeight());
        } else {
          // Magic number of 120 comes from Qt doc on QWheelEvent::pixelDelta()
          mouse_wheelH += wheel_event->angleDelta().x() / 120.0f;
        }

        // Handle vertical component
        if (wheel_event->pixelDelta().y()) {
          // 5 lines per unit
          mouse_wheel += wheel_event->pixelDelta().y() /
                         (5.0 * ImGui::GetTextLineHeight());
        } else {
          // Magic number of 120 comes from Qt doc on QWheelEvent::pixelDelta()
          mouse_wheel += wheel_event->angleDelta().y() / 120.0f;
        }
      }
      default:
        break;
    }
  }
  return QObject::eventFilter(watched, event);
}

void ImGuiPlot::setPlotCount(int count) {
    if (count < 1) count = 1;
    if (count > 4) count = 4;
    m_plotCount = count;
    update();
}

void ImGuiPlot::setJointNum(int num) {
    joint_num = num;
    update();
}
