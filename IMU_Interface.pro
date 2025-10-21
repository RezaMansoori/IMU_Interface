QT += core gui widgets serialport opengl openglwidgets multimedia multimediawidgets charts printsupport 3dcore 3drender 3dinput 3dextras
CONFIG += c++17

SOURCES += \
    bodywidget.cpp \
    main.cpp \
    cnpy.cpp \
    ImageWithTextWidget.cpp \
    eulerchart.cpp \
    modelCalibrator.cpp \
    modelCalibratorNpose.cpp \
    calibrator_utils.cpp \
    processingWorker.cpp \
    smpl_model.cpp \
    startpage.cpp \
    simulationpage.cpp \
    modelviewerpage.cpp \
    filespage.cpp \
    imusdatapage.cpp \
    loginpage.cpp \
    usbreceiver.cpp \
    imu.cpp \
    recorderservice.cpp \
    object3d.cpp \
    calibrationpage.cpp \
    charts.cpp \
    helpdialog.cpp \
    registerpage.cpp \
    combinedcalibrationviewerpage.cpp \
    postprocesspage.cpp \
    CountdownDialog.cpp \
    components/imguiplot.cpp \
    3rdparty/imgui/imgui.cpp \
    3rdparty/imgui/imgui_draw.cpp \
    3rdparty/imgui/imgui_tables.cpp \
    3rdparty/imgui/imgui_widgets.cpp \
    3rdparty/imgui/imgui_demo.cpp \
    3rdparty/imgui/backends/imgui_impl_opengl3.cpp \
    3rdparty/implot/implot.cpp \
    3rdparty/implot/implot_items.cpp \
    utils.cpp

HEADERS += \
    ImageWithTextWidget.h \
    bodywidget.h \
    cnpy.h \
    ImageWithTextWidget.h \
    eulerchart.h \
    modelCalibrator.h \
    calibratorBase.h \
    modelCalibratorNpose.h \
    calibrator_utils.h \
    processingWorker.h \
    smpl_model.h \
    startpage.h \
    simulationpage.h \
    modelviewerpage.h \
    filespage.h \
    imusdatapage.h \
    loginpage.h \
    usbreceiver.h \
    imu.h \
    recorderservice.h \
    object3d.h \
    calibrationpage.h \
    charts.h \
    helpdialog.h \
    registerpage.h \
    combinedcalibrationviewerpage.h \
    postprocesspage.h \
    CountdownDialog.h \
    components/imguiplot.h \
    3rdparty/imgui/imgui.h \
    3rdparty/imgui/imgui_internal.h \
    3rdparty/imgui/backends/imgui_impl_opengl3.h \
    3rdparty/implot/implot.h \
    3rdparty/implot/implot_internal.h \
    utils.h

INCLUDEPATH += \
    C:/eigen/eigen-master \
    $$PWD/components \
    $$PWD/3rdparty/imgui \
    $$PWD/3rdparty/imgui/backends \
    $$PWD/3rdparty/implot \
    C:/libtorch/include \
    C:/libtorch/include/torch/csrc/api/include

INCLUDEPATH += C:/Users/ACER/Desktop/vcpkg/installed/x64-windows/include

RESOURCES += \
    resources.qrc

RESOURCES_DIR = $$PWD/icons
DISTFILES += \
    $$RESOURCES_DIR/critical_battery.png \
    $$RESOURCES_DIR/low_battery.png \
    $$RESOURCES_DIR/medium_battery.png \
    $$RESOURCES_DIR/high_battery.png \
    $$RESOURCES_DIR/full_battery.png \
    $$RESOURCES_DIR/tick.png

LIBS += -LC:/libtorch/lib -ltorch_cuda -lc10_cuda -ltorch -ltorch_cpu -lc10
# LIBS += -L"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.8\lib\x64" -lcudart
LIBS += C:/Users/ACER/Desktop/vcpkg/installed/x64-windows/lib/libzmq-mt-4_3_5.lib
# LIBS += -LC:/Users/ACER/Desktop/vcpkg/installed/x64-windows/lib -lzmq



win32 {
    LIBS += -lOpenGL32
}
