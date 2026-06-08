#ifndef SIMULATIONPAGE_H
#define SIMULATIONPAGE_H

#include "processingWorker.h"
#include "eulerchart.h"
#include "imu.h"
#include "bodywidget.h"
#include "ImageWithTextWidget.h"
#include "usbreceiver.h"
#include "calibratorBase.h"
#include "modelCalibratorNpose.h"  // برای استفاده از NposeCalibrator
#include "fullKinematics.hpp"
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QProcess>
#include <QOpenGLWidget>
#include <QTimer>
#include <QMessageBox>
#include <QApplication>
#include <QComboBox>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DCore/QBuffer>
#include <Qt3DCore/QAttribute>
#include <Qt3DRender/QGeometryRenderer>

#include <thread>
#include <zmq.hpp>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <fstream>
#include <Eigen/Dense>
#include <deque>  // برای deque
#include <QElapsedTimer>  // برای محاسبه زمان
#include <QVector3D>  // برای موقعیت joints

class Calibrator;
class SMPLModel;
struct MeshData {
    std::vector<float> vertices;
    std::vector<int> faces;
};
namespace torch {
namespace jit {
class Module;
}
}
struct CausalAvgImpl;

class SimulationPage : public QWidget
{
    Q_OBJECT

public:
    explicit SimulationPage(QWidget *parent = nullptr, USBReceiver *usbReceiver = nullptr, bool isDarkTheme = true);
    void applyTheme();
    void sendMeshLoop();
    ~SimulationPage();
    explicit SimulationPage(QWidget *parent = nullptr, USBReceiver *usbReceiver = nullptr, bool isDarkTheme = true, const QVariantMap &imus={});
    int getSelectedIMU(){return selectedIMU;}
    std::vector<Eigen::Vector4f> avg_quat;
    std::vector<torch::Tensor> imuStack;
    torch::Tensor sample = torch::zeros({18, 3, 3});
    int imuCount;
public slots:
    void onIMUData(const IMU &data);
    // void onIMUDataKin(const IMU &data);
    void updateSelectedImus(const QMap<QString, QVariant> &selectedImus);
    void onAllEulerAnglesUpdated(const std::vector<std::array<double, 3>>& eulerAngles,
                                 const std::array<double, 3>& translation);
signals:
    void recordingStarted(const QString &fileName);
    void recordingStopped(const QString &fileName);
    void sendIMUData(const IMU &data);
    void sendIMUDataKin(const IMU &data);
    void sendJointIndex(int index);
    void startWorker();

private slots:
    void openBoth();
    void openMeshSimulation();
    void openMeshSimulationKin();
    void closeMeshSimulation();
    void openOpensimSimulation();
    void transmit();
    void embedPythonWindow();
    void updateRunModelConnection(int index);
    //void onCoordinateTest();
    //void onTPoseTest();
    //void processCoordinateTest();
    void processPoseTest();
    void startCalibrationTests();
    void handleCoordinateTestFinished();
    void handlePoseTestFinished();
    
    void onChooseFileClicked();
    void processOfflineFrame();
    void eulerDataUpdated(double rx, double py, double yz);
    void onJointComboChanged(int comboIndex);
private:
    void initUI();
    void loadModel();
    void updateModel();
    void runModelAndSendMesh();
    void send_mesh(const std::vector<float>& verts, const std::vector<int>& faces);
    void send_selections(const QSet<int>& selections);
    void updateMesh(const std::vector<float> &verts, const std::vector<int> &faces);
    void handleUSBConnection(bool connected);
    void stopWorkerThread();

    QHBoxLayout *mainLayout;
    QOpenGLWidget *plotterPlaceholder;
    QComboBox *selectModel;

    QPushButton *btnOpenMesh;
    QPushButton *btnCloseMesh;
    QPushButton *btnMotion;
    //*********
    //QPushButton *btncoordinate;
    //QPushButton *btntpose;
    QPushButton *btnCalibration; // دکمه جدید برای تست‌های کالیبراسیون
    //*********
    QComboBox* jointComboBox;
    QProcess* pythonProcess = nullptr;
    QLineEdit* data_frame;
    QPixmap m_image;
    QString m_text;
    QTimer* tposeTimer = nullptr;
    QString selectedCsvPath;
    QString selectedTPosePath;
    QTimer *offlineTimer;

    ImageWithTextWidget* m_imageWidget;
    BodyWidget *body;
    EulerChart* eulerChart;
    HWND hwndPython = nullptr;
    zmq::context_t context{1};
    zmq::socket_t socket{context, zmq::socket_type::pub};
    Eigen::MatrixXd offlineData;

    bool isDarkTheme;
    bool running = false;
    bool runningKin = false;
    std::ofstream csvFile;
    std::mutex queueMutex;
    std::condition_variable cv;
    std::queue<MeshData> meshQueue;
    std::thread meshThread;
    bool isSimulating = false;
    int selectedIMU;
    std::shared_ptr<torch::jit::Module>  model;
    lbk::BodyKinematics kinematics_model;
    std::unique_ptr<CalibratorBase> calibrator;
    std::map<int, IMU> lastImuData;
    bool meshInitialized = false;
    SMPLModel* smplModel = nullptr;
    std::shared_ptr<CausalAvgImpl> causal_avg;
    void *avg_state;

    bool isCoordinateTesting = false;
    bool isTPoseTesting = false;
    int currentOfflineRow = 0;

    std::map<std::string, int> selected_imus;
    std::map<std::string, int> selected_imusKin;
    Eigen::MatrixXf coord_data;
    Eigen::MatrixXf pose_data;
    Eigen::MatrixXf pose_dataKin;
    Eigen::Matrix3f RMI;
    std::vector<Eigen::Matrix3f> RSB;
    Eigen::MatrixXf acc_offsets;
    std::vector<Eigen::Matrix3f> RIS;
    std::vector<Eigen::Vector3f> pose_acc;
    int sampleCount = 0;
    Qt3DExtras::Qt3DWindow *view;
    QWidget *container;
    Qt3DCore::QEntity *rootEntity;

    Qt3DCore::QBuffer *vertexBuffer;
    Qt3DCore::QBuffer *indexBuffer;
    Qt3DCore::QAttribute *positionAttribute;
    Qt3DCore::QAttribute *indexAttribute;
    Qt3DRender::QGeometryRenderer *renderer;

    bool isUSBConnected;
    USBReceiver *usbReceiver;
    QString selectedNpzPath;

    QThread* workerThread;
    ProcessingWorker* worker;

    // // برای پخش فریم‌های NPZ
    // QTimer* npzTimer;
    // torch::Tensor npz_imu_acc, npz_imu_ori;
    // int npz_numFrames;
    // int currentNpzFrame;

    int csvFrameCounter = 0;

    QWidget *guideBox;
    QLabel *guideLabel;
    QLabel *guideImage;
    void updateGuideBox(bool isCoordinateTest);

    QComboBox *poseComboBox;

    bool traceEnabled = false;  // flag برای فعال/غیرفعال trace
    int traceDurationMs = 1000;  // 1 ثانیه (قابل تغییر)
    int maxTracePoints = 60;    // حداکثر نقاط trace (برای ~60fps)
    float traceSphereRadius = 0.02f;  // اندازه کره (بزرگ‌تر از 0.05f)
    QColor traceColor = QColor(0, 255, 0, 255);  // سبز (بدون opacity در تعریف پایه)
    std::vector<std::deque<std::pair<QVector3D, QElapsedTimer>>> jointTraces;  // برای هر joint: deque از (position, timer)
    std::vector<Qt3DCore::QEntity*> traceEntities;  // لیست entityهای trace برای delete
    std::vector<Qt3DCore::QEntity*> tracePool;  // pool برای reuse entityها
    void updateJointTraces(const std::vector<QVector3D>& currentJoints);  // تابع آپدیت trace
    void clearTraces();  // برای پاک کردن traceها
    QFile* logFile;  // برای logging
    void logMessage(const QString& msg);  // تابع log
};


#endif // SIMULATIONPAGE_H
