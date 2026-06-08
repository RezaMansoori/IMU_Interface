#include "modelCalibrator.h"
#include "smpl_model.h"
#include "simulationpage.h"
#include "bodywidget.h"
#include "utils.h"
#include "CountdownDialog.h"
#include "modelCalibratorNpose.h"
#include "calibratorBase.h"

#include <torch/script.h>
#include <zmq.hpp>
#include <torch/torch.h>
#include <vector>
#include <cstdlib>   // for rand()
#include <ctime>     // for time()
#include <array>
#include "cnpy.h"

#include <QMessageBox>
#include <QProcess>
#include <QOpenGLWidget>
#include <QTimer>
#include <QComboBox>
#include <QLabel>
#include <QDateTime>
#include <QDir>
#include <QLineEdit>
#include <QFileDialog>

#include <Qt3DRender/QCamera>
#include <Qt3DRender/QMaterial>
#include <Qt3DCore/QGeometry>
#include <Qt3DCore/QAttribute>
#include <Qt3DCore/QBuffer>
#include <Qt3DRender/QGeometryRenderer>
#include <Qt3DRender/QPointLight>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DExtras/QForwardRenderer>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DCore/QEntity>
#include <Qt3dRender/QDirectionalLight>
#include <Qt3DExtras/QSphereMesh>

#ifdef Q_OS_WIN
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winuser.h>
#endif
#include <Windows.h>

#define MODEL_PATH "data/newModel-Online.pt"
#define COORD_CSV_PATH "data/coord_cache.bin"
#define TPOSE_CSV_PATH "data/tpose_cache.bin"

#define MODEL_PATH "newModel_Online.pt"
#define KINEMATICS_PATH "smpl_kinematics.pt"
// #define COORD_CSV_PATH "C:/Users/ASUS/Desktop/Coordinte.bin"
// #define TPOSE_CSV_PATH "C:/Users/ASUS/Desktop/T-pose.bin"
#define COORD_CSV_PATH "coord_cache.bin"
#define TPOSE_CSV_PATH "tpose_cache.bin"

SimulationPage::SimulationPage(QWidget *parent, USBReceiver *usbReceiver, bool isDarkTheme, const QVariantMap &imus)
    : QWidget(parent),
    isDarkTheme(isDarkTheme),
    usbReceiver(usbReceiver),
    selected_imus(),                           // <-- <-- don't try to init from QVariantMap here
    avg_quat(18, Eigen::Vector4f::Zero())
{
    for (auto it = imus.begin(); it != imus.end(); ++it) {
        selected_imus[it.key().toStdString()] = it.value().toInt();
    }
    qDebug() << "Selected IMUs:";
    for (const auto &pair : selected_imus) {
        qDebug() << QString::fromStdString(pair.first) << ":" << pair.second;
    }

    initUI();
    // logging init
    logFile = new QFile("debug_log.txt");
    if (logFile->open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append)) {
        logMessage("SimulationPage constructor started");
    } else {
        qWarning() << "Failed to open log file";
    }

    // share OpenGL برای جلوگیری از Qt3D crash
    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts, true);

    // try-catch دور loadModel
    try {
        loadModel();
        logMessage("Model loaded successfully");
    } catch (const std::exception& e) {
        logMessage("Error loading model: " + QString(e.what()));
        QMessageBox::critical(this, "Error", "Failed to load model: " + QString(e.what()));
        return;  // یا exit برنامه
    } catch (...) {
        logMessage("Unknown error loading model");
        QMessageBox::critical(this, "Error", "Unknown error loading model");
        return;
    }

    // worker setup با delay (برای جلوگیری از race)
    workerThread = new QThread();
    worker = new ProcessingWorker(model, smplModel, causal_avg);
    worker->moveToThread(workerThread);
    connect(workerThread, &QThread::started, worker, &ProcessingWorker::start);
    QTimer::singleShot(500, this, [=]() {
        logMessage("Starting workerThread with delay");
        workerThread->start();  // start بدون آرگومان (default priority)
    });

    applyTheme();
    loadModel();

    jointTraces.resize(6);  // فرض 6 joint بر اساس selected_imus (pelvis, left_knee, etc.)
    tracePool.reserve(maxTracePoints * jointTraces.size());  // reserve برای ~360 entity (60*6، برای جلوگیری از realloc)

    connect(worker, &ProcessingWorker::jointsUpdated, this, &SimulationPage::updateJointTraces);

    // برای دیباگ: اضافه کن اگر لازم
    qDebug() << "Trace system initialized with" << jointTraces.size() << "joints";

    offlineTimer = new QTimer(this);
    connect(offlineTimer, &QTimer::timeout, this, &SimulationPage::processOfflineFrame);



    // Connect USBReceiver signals
    if (usbReceiver) {
        // connect(usbReceiver, &USBReceiver::data, processingThread, &ProcessingThread::processIMUData);
        // connect(usbReceiver, &USBReceiver::data, this, &SimulationPage::onIMUData);
        connect(usbReceiver, &USBReceiver::connectionStatus, this, &SimulationPage::handleUSBConnection);
        connect(usbReceiver->findChild<RecorderService*>(), &RecorderService::recordingStarted, this, &SimulationPage::recordingStarted);
        connect(usbReceiver->findChild<RecorderService*>(), &RecorderService::recordingStopped, this, &SimulationPage::recordingStopped);
        isUSBConnected = usbReceiver->isRunning();
    }

    workerThread = new QThread(this);
    worker = new ProcessingWorker(model, smplModel, causal_avg);

    worker->setCalibrator(std::move(calibrator));

    worker->moveToThread(workerThread);
    connect(workerThread, &QThread::finished, worker, &QObject::deleteLater);

    connect(this, &SimulationPage::sendIMUData,
            worker, &ProcessingWorker::processIMUData,
            Qt::QueuedConnection);
    connect(this, &SimulationPage::sendIMUDataKin,
            worker, &ProcessingWorker::processIMUDataKin,
            Qt::QueuedConnection);

    connect(worker, &ProcessingWorker::meshUpdated,
            this, &SimulationPage::updateMesh,
            Qt::QueuedConnection);

    connect(worker, &ProcessingWorker::jointsUpdated, this, &SimulationPage::updateJointTraces);

    connect(worker, &ProcessingWorker::eulerDataUpdated,
            this, &SimulationPage::eulerDataUpdated,
            Qt::QueuedConnection);

    connect(this, &SimulationPage::sendJointIndex,
            worker, &ProcessingWorker::setJointIndex,
            Qt::QueuedConnection);

    connect(jointComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &SimulationPage::onJointComboChanged);

    connect(this, &SimulationPage::startWorker,
            worker, &ProcessingWorker::start,
            Qt::QueuedConnection);

    connect(worker, &ProcessingWorker::allEulerAnglesUpdated,
            this, &SimulationPage::onAllEulerAnglesUpdated,
            Qt::QueuedConnection);

    workerThread->start();

    coord_data.resize(1000, 4);
    coord_data.setZero();
    pose_data.resize(18 * 1000, 7);
    pose_data.setZero();
    sampleCount = 0;

    logMessage("SimulationPage constructor finished");
}

void SimulationPage::initUI()
{
    qDebug() << "Starting initUI for SimulationPage";
    mainLayout = new QHBoxLayout(this);

    QVBoxLayout* visualizerLayout = new QVBoxLayout();

    // Guide box in top-left corner
    guideBox = new QWidget();
    guideBox->setFixedSize(200, 150); // اندازه مناسب برای باکس
    guideBox->setStyleSheet(isDarkTheme ?
                                "background: #3e3e4e; border-radius: 10px; border: 1px solid #555;" :
                                "background: #f7f9fc; border-radius: 10px; border: 1px solid #d0d5dd;");
    QVBoxLayout *guideLayout = new QVBoxLayout(guideBox);
    guideLayout->setAlignment(Qt::AlignCenter);
    guideLayout->setSpacing(10);

    guideLabel = new QLabel("Coordinate Test");
    guideLabel->setStyleSheet(isDarkTheme ?
                                  "font: bold 14px 'Segoe UI'; color: #f8f8f8;" :
                                  "font: bold 14px 'Segoe UI'; color: #1d2939;");
    guideLabel->setAlignment(Qt::AlignCenter);
    guideLayout->addWidget(guideLabel);

    guideImage = new QLabel();
    QPixmap imuPixmap("icons/imu_icon.png");
    if (!imuPixmap.isNull()) {
        guideImage->setPixmap(imuPixmap.scaled(100, 100, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    } else {
        guideImage->setText("IMU Image Not Found");
    }
    guideImage->setAlignment(Qt::AlignCenter);
    guideLayout->addWidget(guideImage);

    visualizerLayout->addWidget(guideBox, 0, Qt::AlignTop | Qt::AlignLeft);

    // Control buttons
    QHBoxLayout *controls = new QHBoxLayout();
    selectModel = new QComboBox();
    selectModel->addItem("Online Mesh with Neural Networks");
    selectModel->addItem("Offline Mesh with Neural Networks");
    selectModel->addItem("Skeleton with Opensim");
    selectModel->addItem("Online mesh with kinematics");
    controls->addWidget(selectModel);
    connect(selectModel, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &SimulationPage::updateRunModelConnection);

    btnOpenMesh = new QPushButton("Run Model");
    btnCloseMesh = new QPushButton("Stop Model");
    btnOpenMesh->setEnabled(false);
    btnCloseMesh->setEnabled(false);
    btnCalibration = new QPushButton("Calibration Tests");
    controls->addWidget(btnCalibration);
    connect(btnCalibration, &QPushButton::clicked, this, &SimulationPage::startCalibrationTests);

    poseComboBox = new QComboBox();
    poseComboBox->addItem("T-Pose");
    poseComboBox->addItem("N-Pose");
    controls->addWidget(poseComboBox);

    QPushButton* btnTrace = new QPushButton("Toggle Trace");
    controls->addWidget(btnTrace);  // controls فرض layout دکمه‌هاست (بر اساس کدت mainLayout یا QVBoxLayout)
    connect(btnTrace, &QPushButton::clicked, this, [=]() {
        traceEnabled = !traceEnabled;
        if (!traceEnabled) clearTraces();
    });
    btnTrace->setStyleSheet(isDarkTheme ?
                                "QPushButton { background: #232526; color: #43cea2; border-radius: 8px; font: 14pt 'Segoe UI'; }"
                                "QPushButton:hover { background: #43cea2; color: #232526; }" :
                                "QPushButton { background: #e0e0e0; color: black; border: 1px solid #333; border-radius: 8px; font: 14pt 'Segoe UI'; }"
                                "QPushButton:hover { background: #333; color: white; }");

    for (QPushButton *btn : {btnCalibration, btnOpenMesh, btnCloseMesh}) {
        controls->addWidget(btn);
    }

    connect(btnOpenMesh, &QPushButton::clicked, this, &SimulationPage::openMeshSimulation);
    connect(btnCloseMesh, &QPushButton::clicked, this, &SimulationPage::closeMeshSimulation);

    visualizerLayout->addLayout(controls);

    view = new Qt3DExtras::Qt3DWindow();
    container = QWidget::createWindowContainer(view);
    container->setMinimumSize(800, 600);
    visualizerLayout->addWidget(container);

    rootEntity = new Qt3DCore::QEntity();
    view->defaultFrameGraph()->setClearColor(QColor(135, 206, 250));
    qDebug() << "rootEntity:" << (rootEntity != nullptr) << "view:" << (view != nullptr);
    Qt3DRender::QCamera *camera = view->camera();
    camera->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    camera->setPosition(QVector3D(0, 0, 5));
    camera->setViewCenter(QVector3D(0, 0, 0));

    Qt3DExtras::QOrbitCameraController *camController = new Qt3DExtras::QOrbitCameraController(rootEntity);
    camController->setCamera(camera);

    // ساخت هندسه اولیه
    Qt3DCore::QGeometry *geometry = new Qt3DCore::QGeometry(rootEntity);
    vertexBuffer = new Qt3DCore::QBuffer(geometry);
    QByteArray vertexData;
    float vertices[] = {
        // موقعیت (x, y, z) | نرمال (nx, ny, nz) | مختصات بافت (u, v)
        -0.5f, -0.5f, 0.0f,  0.0f, 0.0f, 1.0f,  0.0f, 0.0f, // رأس 1
        0.5f, -0.5f, 0.0f,  0.0f, 0.0f, 1.0f,  1.0f, 0.0f, // رأس 2
        0.0f,  0.5f, 0.0f,  0.0f, 0.0f, 1.0f,  0.5f, 1.0f  // رأس 3
    };
    vertexData = QByteArray((const char*)vertices, sizeof(vertices));
    vertexBuffer->setData(vertexData);

    indexBuffer = new Qt3DCore::QBuffer(geometry);
    QByteArray indexData;
    unsigned int indices[] = {0, 1, 2};
    indexData = QByteArray((const char*)indices, sizeof(indices));
    indexBuffer->setData(indexData);

    positionAttribute = new Qt3DCore::QAttribute(geometry);
    positionAttribute->setName(Qt3DCore::QAttribute::defaultPositionAttributeName());
    positionAttribute->setVertexBaseType(Qt3DCore::QAttribute::Float);
    positionAttribute->setVertexSize(3);
    positionAttribute->setAttributeType(Qt3DCore::QAttribute::VertexAttribute);
    positionAttribute->setBuffer(vertexBuffer);
    positionAttribute->setByteStride(8 * sizeof(float));
    positionAttribute->setByteOffset(0);
    positionAttribute->setCount(3);
    geometry->addAttribute(positionAttribute);

    Qt3DCore::QAttribute *normalAttribute = new Qt3DCore::QAttribute(geometry);
    normalAttribute->setName(Qt3DCore::QAttribute::defaultNormalAttributeName());
    normalAttribute->setVertexBaseType(Qt3DCore::QAttribute::Float);
    normalAttribute->setVertexSize(3); // nx, ny, nz
    normalAttribute->setAttributeType(Qt3DCore::QAttribute::VertexAttribute);
    normalAttribute->setBuffer(vertexBuffer);
    normalAttribute->setByteStride(8 * sizeof(float));
    normalAttribute->setByteOffset(3 * sizeof(float));
    normalAttribute->setCount(3);
    geometry->addAttribute(normalAttribute);

    Qt3DCore::QAttribute *texCoordAttribute = new Qt3DCore::QAttribute(geometry);
    texCoordAttribute->setName(Qt3DCore::QAttribute::defaultTextureCoordinateAttributeName());
    texCoordAttribute->setVertexBaseType(Qt3DCore::QAttribute::Float);
    texCoordAttribute->setVertexSize(2); // u, v
    texCoordAttribute->setAttributeType(Qt3DCore::QAttribute::VertexAttribute);
    texCoordAttribute->setBuffer(vertexBuffer);
    texCoordAttribute->setByteStride(8 * sizeof(float));
    texCoordAttribute->setByteOffset(6 * sizeof(float));
    texCoordAttribute->setCount(3);
    geometry->addAttribute(texCoordAttribute);

    indexAttribute = new Qt3DCore::QAttribute(geometry);
    indexAttribute->setAttributeType(Qt3DCore::QAttribute::IndexAttribute);
    indexAttribute->setVertexBaseType(Qt3DCore::QAttribute::UnsignedInt);
    indexAttribute->setBuffer(indexBuffer);
    indexAttribute->setCount(3);
    geometry->addAttribute(indexAttribute);

    renderer = new Qt3DRender::QGeometryRenderer(rootEntity);
    renderer->setGeometry(geometry);
    renderer->setPrimitiveType(Qt3DRender::QGeometryRenderer::Triangles);

    // افزودن منبع نور
    Qt3DCore::QEntity *lightEntity = new Qt3DCore::QEntity(rootEntity);
    Qt3DRender::QDirectionalLight *light = new Qt3DRender::QDirectionalLight(lightEntity);
    light->setColor(Qt::white);
    light->setIntensity(1.0f);
    light->setWorldDirection(QVector3D(-1.0f, -1.0f, -1.0f)); // نور از بالا سمت چپ
    lightEntity->addComponent(light);

    // Qt3DCore::QTransform *lightTransform = new Qt3DCore::QTransform(lightEntity);
    // lightTransform->setTranslation(QVector3D(0.0f, 0.0f, 10.0f));
    // lightEntity->addComponent(lightTransform);


    // ایجاد موجودیت برای مش
    Qt3DCore::QEntity *meshEntity = new Qt3DCore::QEntity(rootEntity);
    Qt3DExtras::QPhongMaterial *material = new Qt3DExtras::QPhongMaterial(rootEntity);
    material->setAmbient(QColor(50, 50, 50));   // محیطی کم
    material->setDiffuse(QColor(200, 200, 200)); // خاکستری
    material->setSpecular(QColor(255, 255, 255)); // براق
    material->setShininess(50.0f);
    meshEntity->addComponent(renderer);
    meshEntity->addComponent(material);

    view->setRootEntity(rootEntity);

    mainLayout->addLayout(visualizerLayout, 3);

    QVBoxLayout* rightLayout = new QVBoxLayout();

    QLabel* label = new QLabel("Select Joint:");
    rightLayout->addWidget(label);

    body = new BodyWidget();
    QStringList jointNames = {
        "Pelvis", "Hip Left", "Hip Right", "Spine Base",
        "Knee Left", "Knee Right", "Spine Mid", "Ankle Left",
        "Ankle Right", "Spine Shoulder", "Foot Left", "Foot Right",
        "Neck", "Clavicle Left", "Clavicle Right", "Head",
        "Shoulder Left", "Shoulder Right", "Elbow Left", "Elbow Right",
        "Wrist Left", "Wrist Right", "Hand Left", "Hand Right"
    };
    body->setSelectionMode(body->MultiSelection);
    body->setSelectionMode(body->SingleSelection);
    body->setJointNames(jointNames);
    jointComboBox = new QComboBox();
    for (int i = 0; i < jointNames.size(); ++i)
        jointComboBox->addItem(jointNames[i], i);
    connect(body, &BodyWidget::jointSelected, this, [=](int index) {
        jointComboBox->setCurrentIndex(index);
        eulerChart->setJointNum(index);
    });
    connect(jointComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            body, [=](int index) {
                emit body->jointSelected(index);
                eulerChart->setJointNum(index);
            });
    body->setMinimumSize(300,300);
    rightLayout->addWidget(body, 0, Qt::AlignCenter);

    eulerChart = new EulerChart(this);
    eulerChart->setMinimumHeight(300);
    rightLayout->addWidget(eulerChart, 1);

    QHBoxLayout* transmitLayout = new QHBoxLayout();

    data_frame = new QLineEdit();
    data_frame->setPlaceholderText("how many data for chart");
    QString styleLineEdit = R"(
        QLineEdit {
            background-color: #f7f9fc;
            border: 1px solid #d0d5dd;
            border-radius: 8px;
            padding: 6px;
            font: 14px 'Segoe UI';
            color: #1d2939;
        }
        QLineEdit::placeholder {
            color: #98a2b3;
        }
    )";
    data_frame->setStyleSheet(styleLineEdit);
    transmitLayout->addWidget(data_frame, 4);
    btnMotion = new QPushButton("Submit");
    transmitLayout->addWidget(btnMotion, 1);
    connect(btnMotion, &QPushButton::clicked, this, &SimulationPage::transmit);

    rightLayout->addLayout(transmitLayout);

    // QWidget* rightContainer = new QWidget();
    // rightContainer->setLayout(rightLayout);
    // rightContainer->setMinimumWidth(400);

    mainLayout->addLayout(rightLayout, 2);  // Right (2 parts)

}


void SimulationPage::applyTheme()
{
    QString widgetStyle = isDarkTheme ?
                              "SimulationPage { background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #232526, stop:1 #414345); }" :
                              "SimulationPage { background: white; color: black;}";

    QString placeholderStyle = isDarkTheme ?
                                   "QOpenGLWidget { background: #232526; }" :
                                   "QOpenGLWidget { background: #e0e0e0; }";

    QString buttonStyle = isDarkTheme ?
                              "QPushButton { background: #232526; color: #43cea2; border-radius: 8px; font: 14pt 'Segoe UI'; }"
                              "QPushButton:hover { background: #43cea2; color: #232526; }" :
                              "QPushButton { background: #e0e0e0; color: black; border: 1px solid #333; border-radius: 8px; font: 14pt 'Segoe UI'; }"
                              "QPushButton:hover { background: #333; color: white; }";
    QString comboStyle = isDarkTheme ?
                             "QComboBox {"
                             "    background: #3e3e4e;"
                             "    color: #43cea2;"
                             "    border-radius: 8px;"
                             "    font: 15px 'Comic Sans MS';"
                             "    text-align: center;"
                             "}"
                             "QComboBox QAbstractItemView {"
                             "    background: #3e3e4e;"
                             "    color: #43cea2;"
                             "    selection-background-color: #185a9d;"
                             "}" :
                             "QComboBox {"
                             "    background: #e0e0e0;"
                             "    color: black;"
                             "    border: 1px solid #333;"
                             "    border-radius: 8px;"
                             "    font: 15px 'Comic Sans MS';"
                             "    text-align: center;"
                             "}"
                             "QComboBox QAbstractItemView {"
                             "    background: #ffffff;"
                             "    color: black;"
                             "    selection-background-color: #c0c0c0;"
                             "}";


    setStyleSheet(widgetStyle);
    // plotterPlaceholder->setStyleSheet(placeholderStyle);
    for (QPushButton *btn : {btnOpenMesh, btnCloseMesh, btnCalibration}) {
        btn->setStyleSheet(buttonStyle);
    }
    selectModel->setStyleSheet(comboStyle);
    poseComboBox->setStyleSheet(comboStyle);
}
#include "fullKinematics.hpp"
void SimulationPage::loadModel()
{
    std::string npz_path = "data/filtered_model.npz";
    std::string fk_path = "data/traced_smpl.pt";
    smplModel = new SMPLModel(npz_path, fk_path);
    causal_avg = std::make_shared<CausalAvgImpl>(3);
    try {
        model = std::make_shared<torch::jit::script::Module>(torch::jit::load(MODEL_PATH));
        model->eval();
        causal_avg = std::make_shared<CausalAvgImpl>(5);
        smplModel = new SMPLModel("data/smpl_male.npz", "data/fk_model.pt");
        qDebug() << "[Model] Loaded TorchScript model.";
        lbk::BodyKinematics kinematics_model;
        // Eigen::MatrixXf coord_data = loadMatrix(COORD_CSV_PATH);
        // qDebug() << "[Model] Start Loading Calibrator.";

        // Eigen::MatrixXf tpose_data = loadMatrix(TPOSE_CSV_PATH);
        // qDebug() << "[Model] Start Loading Calibrator.";

        // std::map<std::string, int> selected_imus = {
        //     {"pelvis", 5}, {"l_foot", 11}, {"r_foot", 12},
        //     {"head", 8}, {"l_arm", 14}, {"r_arm", 15}
        // };
        // qDebug() << "[Model] Start Loading Calibrator.";
        // calibrator = std::make_unique<Calibrator>(selected_imus, coord_data, tpose_data);

        qDebug() << "[Model] Loaded Calibrator.";
    } catch (const std::exception& e) {
        std::cerr << "Standard exception: " << e.what() << std::endl;
    }
    catch (const std::string& s) {
        std::cerr << "String exception: " << s << std::endl;
    }
    catch (int code) {
        std::cerr << "Integer exception with code: " << code << std::endl;
    }
    catch (const c10::Error &e) {
        qWarning() << "[Model] Failed to load TorchScript model:" << e.what();
        logMessage("Torch error: " + QString(e.what()));
        throw;
    }
    catch (...) {
        std::cerr << "Unknown type of exception!" << std::endl;
    }
}
// void printAvgQuats(const std::vector<Eigen::Vector4f>& avg_quat)
// {
//     for (size_t i = 0; i < avg_quat.size(); ++i)
//     {
//         const auto& q = avg_quat[i];
//         qDebug().nospace()
//             << "avg_quat[" << i << "]: ("
//             << q(0) << ", "
//             << q(1) << ", "
//             << q(2) << ", "
//             << q(3) << ")";
//     }
// }
void SimulationPage::onIMUData(const IMU &data)
{
    // qDebug() << "Received IMU data with ID:" << data.id;

    if (isCoordinateTesting && sampleCount < 1000) {
        int pelvisId = selected_imus.at("pelvis");
        if (data.id == pelvisId) {
            int row = sampleCount;
            coord_data(row, 0) = data.quat[0];
            coord_data(row, 1) = data.quat[1];
            coord_data(row, 2) = data.quat[2];
            coord_data(row, 3) = data.quat[3];
            sampleCount++;
        }
    }

    if (isTPoseTesting && sampleCount < 1000) {
        int id = data.id;
        auto it = std::find_if(selected_imus.begin(), selected_imus.end(),
                               [id](const auto& pair) { return pair.second == id; });
        if (it != selected_imus.end()) {
            int row = id * 1000 + sampleCount;
            if (row >= 0 && row < pose_data.rows()) {
                pose_data(row, 0) = data.lacc[0];
                pose_data(row, 1) = data.lacc[1];
                pose_data(row, 2) = data.lacc[2];
                pose_data(row, 3) = data.quat[0];
                pose_data(row, 4) = data.quat[1];
                pose_data(row, 5) = data.quat[2];
                pose_data(row, 6) = data.quat[3];

                // qDebug() << "Stored quat for ID" << id << "at row" << row << ":"
                //          << data.quat[0] << data.quat[1] << data.quat[2] << data.quat[3];
                // qDebug() << "pose_data Row" << row << ": ["
                //          << pose_data(row, 0) << ", "  // acc_x
                //          << pose_data(row, 1) << ", "  // acc_y
                //          << pose_data(row, 2) << ", "  // acc_z
                //          << pose_data(row, 3) << ", "  // quat_x
                //          << pose_data(row, 4) << ", "  // quat_y
                //          << pose_data(row, 5) << ", "  // quat_z
                //          << pose_data(row, 6) << "]";  // quat_w

                if (id == selected_imus.at("pelvis")) {
                    sampleCount++;
                    // qDebug() << "Incremented sampleCount to" << sampleCount << "for pelvis IMU ID" << id;
                }
            } else {
                // qDebug() << "Invalid row index:" << row << "for IMU ID:" << id;
            }
        } else {
            // qDebug() << "IMU ID" << id << "not in selected_imus";
        }
    }

    // static const std::set<int> selectedIds = {5, 9, 12, 8, 14, 15};
    if (containsId(selected_imus, data.id)) {
        lastImuData[data.id] = data;
        // qDebug() << "IMU ID: " << data.id << ", lacc: [" << lastImuData[data.id].lacc[0] << "," << lastImuData[data.id].lacc[1] << "," << lastImuData[data.id].lacc[2] << "]";
        // qDebug() << "IMU ID: " << data.id << ", quat: [" << lastImuData[data.id].quat[0] << "," << lastImuData[data.id].quat[1] << "," << lastImuData[data.id].quat[2] << "," << lastImuData[data.id].quat[3] << "]";
    } else {
        // qDebug() << "Ignoring IMU ID: " << data.id << " (not in selected IDs)";
    }
    // qDebug() << "data id: " << data.id << "\n";
    // qDebug() << "lastIMUDATA size" << lastImuData.size() << "\n";
    if (isSimulating && lastImuData.size() >= 6){
        // qDebug() << "Selected IMUs: 0";
        // qDebug() << "IMU ID: " << data.id << ", lacc: [" << lastImuData[data.id].lacc[0] << "," << lastImuData[data.id].lacc[1] << "," << lastImuData[data.id].lacc[2] << "]";
        // qDebug() << "IMU ID: " << data.id << ", quat: [" << lastImuData[data.id].quat[0] << "," << lastImuData[data.id].quat[1] << "," << lastImuData[data.id].quat[2] << "," << lastImuData[data.id].quat[3] << "]";
    }/* else {
        qDebug() << "Ignoring IMU ID: " << data.id << " (not in selected IDs)";
    }*/
    // qDebug() << "data id: " << data.id << "\n";
    // qDebug() << "lastIMUDATA size" << lastImuData.size() << "\n";
    // qDebug() << "Selected IMUs: 1 snd runningKin" << runningKin;
    if ((isSimulating || runningKin) && lastImuData.size() >= 0){
        // qDebug() << "Selected IMUs: 3";
        for (const auto& pair : lastImuData) {
            const IMU& imu = pair.second;
            if (std::isnan(imu.lacc[0]) || std::isnan(imu.lacc[1]) || std::isnan(imu.lacc[2]) ||
                std::isnan(imu.quat[0]) || std::isnan(imu.quat[1]) || std::isnan(imu.quat[2]) || std::isnan(imu.quat[3])) {
                // qDebug() << "Error: Invalid IMU data for ID " << pair.first;
                return;
            }
        }
        if (containsId(selected_imus, data.id)) {
            emit sendIMUData(data);
            // qDebug() << "Emitted IMU data for ID: " << data.id;
            // emit sendIMUData(data);
            emit sendIMUDataKin(data);
            // qDebug() << "Emitted IMU data for ID: " << data.quat[0];
        }
    }
}

void SimulationPage::updateMesh(const std::vector<float> &verts, const std::vector<int> &faces) {
    std::vector<QVector3D> normals = computeNormals(verts, faces);
    std::vector<float> extendedVerts;
    extendedVerts.reserve(verts.size() * 8 / 3);

    for (size_t i = 0; i < verts.size() / 3; ++i) {
        extendedVerts.push_back(verts[3*i + 0]); // x
        extendedVerts.push_back(verts[3*i + 1]); // y
        extendedVerts.push_back(verts[3*i + 2]); // z

        extendedVerts.push_back(normals[i].x());
        extendedVerts.push_back(normals[i].y());
        extendedVerts.push_back(normals[i].z());

        extendedVerts.push_back(0.0f);
        extendedVerts.push_back(0.0f);
    }



    QByteArray vertexBufferBytes;
    vertexBufferBytes.resize(extendedVerts.size() * sizeof(float));
    memcpy(vertexBufferBytes.data(), extendedVerts.data(), vertexBufferBytes.size());
    vertexBuffer->setData(vertexBufferBytes);

    QByteArray indexBufferBytes;
    indexBufferBytes.resize(faces.size() * sizeof(int));
    memcpy(indexBufferBytes.data(), faces.data(), indexBufferBytes.size());
    indexBuffer->setData(indexBufferBytes);

    positionAttribute->setCount(verts.size() / 3);
    indexAttribute->setCount(faces.size());
}

void SimulationPage::onAllEulerAnglesUpdated(const std::vector<std::array<double, 3>>& eulerAngles,
                                             const std::array<double, 3>& translation) {
    if (!csvFile.is_open()) {
        qWarning() << "CSV file not open for writing Euler angles";
        return;
    }

    for (size_t jointIndex = 0; jointIndex < eulerAngles.size(); ++jointIndex) {
        const auto& euler = eulerAngles[jointIndex];
        if (csvFile.is_open())
            csvFile << euler[0] << "," << euler[1] << "," << euler[2] << ",";
    }
    if (csvFile.is_open()){
        csvFile << translation[0] << "," << translation[1] << "," << translation[2] << "\n";
        // csvFile << "\n";
    }
    csvFile.flush();
    csvFrameCounter++;
}

void SimulationPage::updateModel()
{
    // using torch::tensor;
    std::vector<torch::Tensor> accs, rots;

    for (int id : {5, 11, 12, 8, 14, 15}) {
        const IMU &imu = lastImuData[id];
        accs.push_back(torch::tensor({imu.lacc[0], imu.lacc[1], imu.lacc[2]}).reshape({1, 3}));
        rots.push_back(torch::tensor({imu.quat[0], imu.quat[1], imu.quat[2], imu.quat[3]}).reshape({1, 4}));
    }

    auto acc = torch::cat(accs, 0) * 9.81;
    acc = acc.contiguous();
    qDebug() << "acc to eigen\n";
    Eigen::MatrixXf acc_eigen = Eigen::Map<Eigen::MatrixXf>(acc.data_ptr<float>(), acc.size(0), acc.size(1));
    qDebug() << "quat to rot mat\n";

    auto rotations = Calibrator::quat_batch_to_rotmats(torch::cat(rots, 0));

    torch::Tensor calibAcc, calibRot;
    Eigen::MatrixXf cal_acc;
    std::vector<Eigen::Matrix3f> cal_rot;
    calibrator->calibrate(acc_eigen, rotations, cal_acc, cal_rot);

    auto options = torch::TensorOptions();
    calibAcc = torch::from_blob(
                   cal_acc.data(),
                   {cal_acc.rows(), cal_acc.cols()},
                   options
                   ).clone();  // <— clone() makes it own its data

    std::vector<torch::Tensor> rot_tensors;
    rot_tensors.reserve(cal_rot.size());
    for (auto &M : cal_rot) {
        // from_blob wraps the 3×3 data; then clone to own its memory
        rot_tensors.push_back(
            torch::from_blob(
                const_cast<float*>(M.data()),  // data ptr is const, so cast away
                {3, 3},
                torch::TensorOptions()
                ).clone()
            );
    }

    // Stack along a new first dimension → shape [N, 3, 3]
    calibRot = torch::stack(rot_tensors, /*dim=*/0);
    // QString rowStr = QString("calib acc: [%1, %2, %3]").arg(i)
    //                      .arg(V(i,0)).arg(V(i,1)).arg(V(i,2));
    // qDebug() << rowStr;
    // qDebug() << "Type of calibAcc before forward(): " << calibAcc.dtype();
    // qDebug() << "Type of calibRot before forward(): " << calibRot.dtype();
    // qDebug() << calibAcc.size();
    qDebug() << "start loading model\n";
    try{
        auto output = model->forward({calibAcc, calibRot}).toTuple();
        auto pose = output->elements()[0].toTensor();   // [1, 24, 3, 3]
        auto trans = output->elements()[1].toTensor();  // [1, 3]
        qDebug() << "model returned datas\n";

        int jointIndex = jointComboBox->currentData().toInt();
        torch::Tensor R = pose[0][jointIndex].to(torch::kFloat64);
        torch::Tensor R_tensor_d = R.to(torch::kFloat64).contiguous();
        const double (*R_array)[3] = reinterpret_cast<const double (*)[3]>(R_tensor_d.data_ptr<double>());
        std::array<double, 3> euler = rotm2eul_XYZ(R_array);
        eulerChart->addEulerData(euler[0], euler[1], euler[2]);

        for (int var = 0; var < 24; ++var) {
            torch::Tensor R = pose[0][var].to(torch::kFloat64);
            torch::Tensor R_tensor_d = R.to(torch::kFloat64).contiguous();
            const double (*R_array)[3] = reinterpret_cast<const double (*)[3]>(R_tensor_d.data_ptr<double>());
            std::array<double, 3> euler = rotm2eul_XYZ(R_array);
            if (csvFile.is_open())
                csvFile << euler[0] << "," << euler[1] << "," << euler[2] << ",";
        }
        if (csvFile.is_open()){
            csvFile << "\n";
        }
        qDebug() << "saved CSV FILE\n";

        std::vector<torch::Tensor> p_pose_list;
        std::vector<torch::Tensor> p_trans_list;
        p_pose_list.push_back(pose.clone());
        p_trans_list.push_back(trans.clone());
        qDebug() << "start Sampl Model\n";

        auto [verts, faces, joints] = smplModel->view_motion(p_pose_list, p_trans_list);
        qDebug() << "Finished Sampl Model\n";

        torch::Tensor* state_ptr = static_cast<torch::Tensor*>(avg_state);
        c10::optional<torch::Tensor> opt_state;
        if (state_ptr)
            opt_state = *state_ptr;
        auto [pooled_verts, new_state] = causal_avg->forward(verts.unsqueeze(0), opt_state);
        verts = pooled_verts;
        if (!avg_state)
            avg_state = new torch::Tensor(new_state);
        else
            *static_cast<torch::Tensor*>(avg_state) = new_state;
        verts = verts.contiguous().to(torch::kCPU);
        std::vector<float> vertices(verts.data_ptr<float>(), verts.data_ptr<float>() + verts.numel());

        faces = faces.contiguous().to(torch::kCPU);
        std::vector<int> face_indices;
        if (faces.scalar_type() == torch::kInt64) {
            std::vector<int64_t> tmp(faces.data_ptr<int64_t>(), faces.data_ptr<int64_t>() + faces.numel());
            face_indices.assign(tmp.begin(), tmp.end());
        } else {
            face_indices.assign(faces.data_ptr<int>(), faces.data_ptr<int>() + faces.numel());
        }
        updateMesh(vertices, face_indices);
    } catch (const std::exception& e) {
        std::cerr << "Standard exception: " << e.what() << std::endl;
    }
    catch (const std::string& s) {
        std::cerr << "String exception: " << s << std::endl;
    }
    catch (int code) {
        std::cerr << "Integer exception with code: " << code << std::endl;
    }
    catch (...) {
        std::cerr << "Unknown type of exception!" << std::endl;
    }

    // send_mesh(vertices, face_indices);
}

bool is_tpose_satisfied(int i/*const torch::Tensor& joints*/) {
    if (i > 350 && i < 370)
        return false;
}

void SimulationPage::runModelAndSendMesh()
{
    // run_python_viewer();
    // QTimer::singleShot(0, this, &SimulationPage::embedPythonWindow);
    // Start mesh sender thread
    // std::thread meshThread(&SimulationPage::sendMeshLoop, this);

    std::cout << "detaching";
    // meshThread.detach();
    std::cout << "detached";
    std::string model_path = "data/newModel-Online.pt";
    std::string npz_path   =  "data/s1_walking1.npz";
    std::string fk_path = "data/traced_smpl.pt";
    std::string npz_smpl_path = "data/filtered_model.npz";

    // selected_imus = {
    //     {"Pelvis", 5},
    //     {"ThighL", 15}, {"ShankL", 11}, {"FootL", 13},
    //     {"ThighR", 10}, {"ShankR", 12}, {"FootR", 14}
    // };
    std::map<std::string, int> Kin = {
        {"pelvis", 5}, {"back", 9}, /*{"Head", 15},*/
        /*{"ArmL", 14},*/ {"r_arm", 15}, /*{"ForearmL", 16},*/ {"r_forearm", 11},
        /*{"HandL", 18},*/ /*{"HandR", 14},*/ /*{"ThighL", 9},*/ {"r_thigh", 10},
        /*{"ShankL", 11},*/ {"r_shank", 12}/*, {"FootL", 13}, {"FootR", 12}*/
    };

    try {
        Eigen::MatrixXf coord_data = loadMatrix(COORD_CSV_PATH);
        Eigen::MatrixXf tpose_data = loadMatrix(TPOSE_CSV_PATH);

        // std::unique_ptr<Calibrator> calibrator = std::make_unique<Calibrator>(selected_imus, coord_data, tpose_data);

        torch::NoGradGuard no_grad;

        // Load input data
        cnpy::npz_t npz = cnpy::npz_load(npz_path);
        auto getOr = [&](const std::string& a, const std::string& b) -> cnpy::NpyArray& {
            if (npz.count(a)) return npz[a];
            if (npz.count(b)) return npz[b];
            throw std::runtime_error("Neither " + a + " nor " + b + " found in npz");
        };

        torch::Tensor imu_acc = npzToTensor(getOr("imu_acc", "vacc"), torch::kFloat32);
        torch::Tensor imu_ori = npzToTensor(getOr("imu_ori", "vrot"), torch::kFloat32);
        torch::Tensor uwb;
        bool has_uwb = npz.count("uwb");
        if (has_uwb) uwb = npzToTensor(npz["uwb"], torch::kFloat32);

        int64_t T = imu_acc.size(0);
        torch::Tensor avg_state;

        for (int64_t t = 0; t < T; ++t) {
            if (!running) { break; }
            std::vector<torch::jit::IValue> inputs{
                imu_acc[t].to(torch::kCPU),
                imu_ori[t].to(torch::kCPU)
            };
            if (has_uwb) inputs.push_back(uwb[t].to(torch::kCPU));
            auto out = model->forward(inputs).toTuple();
            torch::Tensor pose_t = out->elements()[0].toTensor();
            torch::Tensor trans_t;
            if (out->elements().size() > 1 && out->elements()[1].isTensor())
                trans_t = out->elements()[1].toTensor();

            {
            int jointIndex = jointComboBox->currentData().toInt();
            torch::Tensor R = pose_t[0][jointIndex].to(torch::kFloat64);
            torch::Tensor R_tensor_d = R.to(torch::kFloat64).contiguous();
            const double (*R_array)[3] = reinterpret_cast<const double (*)[3]>(R_tensor_d.data_ptr<double>());
            std::array<double, 3> euler = rotm2eul_XYZ(R_array);
            eulerChart->addEulerData(euler[0], euler[1], euler[2]);

            for (int var = 0; var < 24; ++var) {
                torch::Tensor R = pose_t[0][var].to(torch::kFloat64);
                torch::Tensor R_tensor_d = R.to(torch::kFloat64).contiguous();
                const double (*R_array)[3] = reinterpret_cast<const double (*)[3]>(R_tensor_d.data_ptr<double>());
                std::array<double, 3> euler = rotm2eul_XYZ(R_array);
                if (csvFile.is_open())
                    csvFile << euler[0] << "," << euler[1] << "," << euler[2] << ",";
            }
            if (csvFile.is_open()){
                csvFile << "\n";
            }
            }

            std::vector<torch::Tensor> p_pose_list;
            std::vector<torch::Tensor> p_trans_list;
            p_pose_list.push_back(pose_t.clone());
            p_trans_list.push_back(trans_t.clone());
            auto [verts, faces, joints] = smplModel->view_motion(p_pose_list, p_trans_list);
            auto [pooled_verts, new_state] = causal_avg->forward(verts.unsqueeze(0), avg_state);
            verts = pooled_verts;
            avg_state = new_state;

            verts = verts.contiguous().to(torch::kCPU);
            std::vector<float> vertices(verts.data_ptr<float>(), verts.data_ptr<float>() + verts.numel());

            faces = faces.contiguous().to(torch::kCPU);
            std::vector<int> face_indices;
            if (faces.scalar_type() == torch::kInt64) {
                std::vector<int64_t> tmp(faces.data_ptr<int64_t>(), faces.data_ptr<int64_t>() + faces.numel());
                face_indices.assign(tmp.begin(), tmp.end());
            } else {
                face_indices.assign(faces.data_ptr<int>(), faces.data_ptr<int>() + faces.numel());
            }

            updateMesh(vertices, face_indices);
        }
        csvFile.close();
    } catch (const std::exception& e) {
        std::cerr << "[openOpenSim ERROR] " << e.what() << std::endl;
        QMessageBox::warning(nullptr, "Error", QString::fromStdString(e.what()));
    }
}

void SimulationPage::openMeshSimulation() {
    if (!isSimulating) {
        isSimulating = true;
        btnOpenMesh->setEnabled(false);
        btnCloseMesh->setEnabled(true);
        selectModel->setEnabled(false);
    }
    running = true;

    int modelIndex = selectModel->currentIndex();
    if (modelIndex == 0) {
    QDir().mkpath("data");
        csvFile.open(QString("data/joints_angles_%1.csv").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss")).toStdString());
    if(csvFile.is_open()){
        for (int var = 0; var < 24; ++var) {
            csvFile << "Roll" << var << "," << "Pitch" << var << "," << "Yaw" << var << ",";
        }
            csvFile << "trans_x,trans_y,trans_z\n";
    }
        int currentJoint = jointComboBox->currentData().toInt();
        emit sendJointIndex(currentJoint);
        csvFrameCounter = 0;

        emit startWorker();
        connect(usbReceiver, &USBReceiver::data, this, &SimulationPage::onIMUData);

    } else if (modelIndex == 1) {
        if (offlineData.rows() == 0) {
            QMessageBox::warning(this, "Error", "No offline data loaded!");
            return;
        }
        currentOfflineRow = 0;
        offlineTimer->start(16);  // ~60Hz
    }
}

void SimulationPage::openMeshSimulationKin() {
    qDebug() << "=== openMeshSimulationKin CALLED ===";
    if (!isSimulating) {
        isSimulating = true;
        btnOpenMesh->setEnabled(false);
        btnCloseMesh->setEnabled(true);
        selectModel->setEnabled(false);
    }
    qDebug() << "runningKin before" << runningKin;
    runningKin = true;
    qDebug() << "runningKin after" << runningKin;
    int modelIndex = selectModel->currentIndex();
    if (modelIndex == 0) {
        QDir().mkpath("data");
        csvFile.open(QString("data/joints_anglesKin_%1.csv").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss")).toStdString());
        if(csvFile.is_open()){
            for (int var = 0; var < 24; ++var) {
                csvFile << "Roll" << var << "," << "Pitch" << var << "," << "Yaw" << var << ",";
            }
            csvFile << "trans_x,trans_y,trans_z\n";
        }
        int currentJoint = jointComboBox->currentData().toInt();
        emit sendJointIndex(currentJoint);
        csvFrameCounter = 0;

        emit startWorker();
        connect(usbReceiver, &USBReceiver::data, this, &SimulationPage::onIMUData);

    } else if (modelIndex == 1) {
        if (offlineData.rows() == 0) {
            QMessageBox::warning(this, "Error", "No offline data loaded!");
            return;
        }
        currentOfflineRow = 0;
        offlineTimer->start(16);  // ~60Hz
    }
}
#include <iostream>
#include <Eigen/Dense>

void printPoseData(const Eigen::MatrixXf &pose_data, int imuCount, int rowsToPrint = 5) {
    int T = pose_data.rows();
    int colsPerIMU = 7;

    int printRows = std::min(rowsToPrint, T);

    for (int t = 0; t < printRows; ++t) {
        std::cout << "Time step " << t << ":\n";
        for (int j = 0; j < imuCount; ++j) {
            int colStart = j * colsPerIMU;
            std::cout << "  IMU " << j << " -> Acc: ["
                      << pose_data(t, colStart + 0) << ", "
                      << pose_data(t, colStart + 1) << ", "
                      << pose_data(t, colStart + 2) << "] "
                      << "Quat: ["
                      << pose_data(t, colStart + 3) << ", "
                      << pose_data(t, colStart + 4) << ", "
                      << pose_data(t, colStart + 5) << ", "
                      << pose_data(t, colStart + 6) << "]\n";
        }
        std::cout << "--------------------------\n";
    }
}

void SimulationPage::updateRunModelConnection(int index)
{
    // Disconnect any existing connection to avoid multiple connections
    disconnect(btnOpenMesh, &QPushButton::clicked, this, nullptr);
    disconnect(btnCalibration, &QPushButton::clicked, this, nullptr);
    // Reconnect based on the selected model
    if (index == 0) { // "Online Mesh with NN"
        connect(btnOpenMesh, &QPushButton::clicked, this, &SimulationPage::openMeshSimulation);
        btnCalibration->setText("Calibration");
        connect(btnCalibration, &QPushButton::clicked, this, &SimulationPage::startCalibrationTests);
    } else if(index == 1){ // "Offline Mesh with NN"
        connect(btnOpenMesh, &QPushButton::clicked, this, &SimulationPage::openMeshSimulation);
        btnCalibration->setText("Choose File");
        connect(btnCalibration, &QPushButton::clicked, this, &SimulationPage::onChooseFileClicked);
    }else if (index == 2) { // "Skeleton with Opensim"
        worker->setUseKinematics(false);
        connect(btnOpenMesh, &QPushButton::clicked, this, &SimulationPage::openOpensimSimulation);
    }else if (index == 3) { // "online kinematics"
        qDebug() << "Setting up kinematics mode - index 3";
        qDebug() << "btnOpenMesh pointer:" << btnOpenMesh;
        qDebug() << "btnOpenMesh is enabled:" << btnOpenMesh->isEnabled();
        // selected_imus = {
        //     {"Pelvis", 5},
        //     {"ThighL", 15}, {"ShankL", 11}, {"FootL", 13},
        //     {"ThighR", 10}, {"ShankR", 12}, {"FootR", 14}
        // };
        std::vector<Eigen::Vector4f> avg_quat_scaled;
        avg_quat_scaled.reserve(avg_quat.size());
        btnOpenMesh->setEnabled(true);
        qDebug() << "btnOpenMesh is enabled:" << btnOpenMesh->isEnabled();
        for (const auto& q : avg_quat) {
            avg_quat_scaled.push_back(q / sampleCount);
        }
        runningKin = true;
        worker->calibrateKin(selected_imus, avg_quat_scaled);
        // worker->calibrateKin(selected_imus, avg_quat);
        // qDebug() << "Setting up Online kinematics";
        // connect(btnOpenMesh, &QPushButton::clicked, this, &SimulationPage::openMeshSimulationKin);
        // qDebug() << "Setting up Online kinematics 2";
        qDebug() << "Connecting to openMeshSimulationKin (index 3)";
        connect(btnOpenMesh, &QPushButton::clicked, this, [this]() {
            qDebug() << "LAMBDA TEST: Button clicked for Kinematics (index 3)!";
            this->openMeshSimulationKin();
        });
        // بررسی کنید که آیا این دکمه با دکمه روی صفحه یکی هست
        qDebug() << "btnOpenMesh address:" << btnOpenMesh;
        qDebug() << "btnOpenMesh text:" << btnOpenMesh->text();
        qDebug() << "btnOpenMesh objectName:" << btnOpenMesh->objectName();
    }
}

// void SimulationPage::onCalibrationClicked()
// {
//     selectedTPosePath = QFileDialog::getOpenFileName(this,
//                                                    tr("Choose Bin File"),
//                                                    "C:/Users/ACER/Downloads/Telegram Desktop/IMU_Interfacev54/IMU_Interface/data/",
//                                                    tr("Bin Files (*.bin);;All Files (*)"));

//     if (!selectedTPosePath.isEmpty()) {
//         try {
//             Eigen::MatrixXf coord_data = loadMatrix(COORD_CSV_PATH);
//             Eigen::MatrixXf tpose_data = loadMatrix(TPOSE_CSV_PATH);

//             std::map<std::string, int> selected_imus = {
//                 {"pelvis", 5}, {"l_foot", 11}, {"r_foot", 12},
//                 {"head", 8}, {"l_arm", 14}, {"r_arm", 15}
//             };
//             calibrator = std::make_unique<Calibrator>(selected_imus, coord_data, tpose_data);

//             QMessageBox::information(this, "File Selected", "Selected bin: " + selectedTPosePath);
//             btnOpenMesh->setEnabled(true);
//         } catch (const std::exception& e) {
//             QMessageBox::warning(this, "Error", "Failed to load CSV: " + QString::fromStdString(e.what()));
//         }
//     } else {
//         qDebug() << "No file selected.";
//     }
// }

void SimulationPage::onChooseFileClicked()
{
    selectedCsvPath = QFileDialog::getOpenFileName(this,
                                                   tr("Choose CSV File"),
                                                   ".",
                                                   tr("CSV Files (*.csv);;All Files (*)"));

    if (!selectedCsvPath.isEmpty()) {
        try {
            offlineData = loadEulerCsv(selectedCsvPath.toStdString());
            qDebug() << "Loaded offline data:" << offlineData.rows() << "frames";
            QMessageBox::information(this, "File Selected", "Selected CSV: " + selectedCsvPath + "\nFrames: " + QString::number(offlineData.rows()));
            btnOpenMesh->setEnabled(true);
        } catch (const std::exception& e) {
            QMessageBox::warning(this, "Error", "Failed to load CSV: " + QString::fromStdString(e.what()));
        }
    } else {
        qDebug() << "No file selected.";
    }
}

void SimulationPage::processOfflineFrame() {
    if (currentOfflineRow >= offlineData.rows()) {
        offlineTimer->stop();
        closeMeshSimulation();
        return;
    }
    qDebug() << "start process Offline";
    Eigen::RowVectorXd row = offlineData.row(currentOfflineRow);
    std::vector<float> pose_vec(216);
    for (int i = 0; i < 216; ++i) {
        pose_vec[i] = static_cast<float>(row(i+1));
    }
    torch::Tensor pose_t = torch::from_blob(pose_vec.data(), {1, 24, 3, 3},
                                            torch::TensorOptions().dtype(torch::kFloat32)).clone();
    qDebug() << "loaded pose matrices";

    float trans_data[3] = {
        static_cast<float>(row(217)),
        static_cast<float>(row(218)),
        static_cast<float>(row(219))
    };
    torch::Tensor trans_t = torch::from_blob(trans_data, {1, 3},
                                             torch::TensorOptions().dtype(torch::kFloat32)).clone();
    qDebug() << "load translations";

    std::vector<torch::Tensor> p_pose_list = {pose_t};
    std::vector<torch::Tensor> p_trans_list = {trans_t};

    qDebug() << "send for view motion";
    auto [verts, faces, joints] = smplModel->view_motion(p_pose_list, p_trans_list);
    qDebug() << "sent for view motion";

    torch::Tensor* state_ptr = static_cast<torch::Tensor*>(avg_state);
    c10::optional<torch::Tensor> opt_state;
    if (state_ptr)
        opt_state = *state_ptr;
    auto [pooled_verts, new_state] = causal_avg->forward(verts.unsqueeze(0), opt_state);
    verts = pooled_verts;
    if (!avg_state)
        avg_state = new torch::Tensor(new_state);
    else
        *static_cast<torch::Tensor*>(avg_state) = new_state;

    verts = verts.contiguous().to(torch::kCPU);
    std::vector<float> vertices(verts.data_ptr<float>(), verts.data_ptr<float>() + verts.numel());

    faces = faces.contiguous().to(torch::kCPU);
    std::vector<int> face_indices(faces.data_ptr<int>(), faces.data_ptr<int>() + faces.numel());

    updateMesh(vertices, face_indices);

    int jointIndex = jointComboBox->currentData().toInt();
    torch::Tensor R_tensor = pose_t[0][jointIndex].clone();  // [3,3] float
    torch::Tensor R_double = R_tensor.to(torch::kFloat64).contiguous();
    const double (*R_array)[3] = reinterpret_cast<const double (*)[3]>(R_double.data_ptr<double>());
    std::array<double, 3> euler = rotm2eul_XYZ(R_array);
    eulerChart->addEulerData(euler[0], euler[1], euler[2]);

    currentOfflineRow++;
}

void SimulationPage::onJointComboChanged(int comboIndex) {
    int jointIndex = jointComboBox->itemData(comboIndex).toInt();
    emit sendJointIndex(jointIndex);
}

void SimulationPage::eulerDataUpdated(double rx, double py, double yz) {
    qDebug() << "eulerDataSignal";
    eulerChart->addEulerData(rx, py, yz);
}

void SimulationPage::send_selections(const QSet<int>& selections) {
    std::cout << "[C++] Initializing ZeroMQ for selections..." << std::endl;
    zmq::context_t context(1);
    zmq::socket_t controlPubSocket(context, zmq::socket_type::pub);
    controlPubSocket.bind("tcp://*:5557");

    // Give SUB time to connect
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::vector<int> nums(selections.begin(), selections.end());
    std::sort(nums.begin(), nums.end());


    std::ostringstream msg_stream;
    msg_stream << data_frame->text().toStdString() <<",";
    bool first = true;
    for (int num : nums) {
        if (num >= 0 && num <= 23) {
            if (!first) msg_stream << ",";
            msg_stream << num;
            first = false;
        }
    }

    std::string msg = msg_stream.str();
    if (msg.empty()) {
        std::cout << "[C++] No valid selections to send." << std::endl;
        return;
    }

    zmq::message_t message(msg.data(), msg.size());
    controlPubSocket.send(message, zmq::send_flags::none);
    std::cout << "[C++] Sent selections: " << msg << std::endl;

    // Keep socket alive briefly to ensure delivery
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    controlPubSocket.close();
    context.close();
    std::cout << "[C++] ZeroMQ for selections cleaned up." << std::endl;

}

void SimulationPage::transmit()
{
    qDebug() << "[C++] Open Motion.";

    if (pythonProcess) {
        qDebug() << "[C++] Python viewer already running.";
    }else{
        QString pythonScriptPath = "data/createChart.py";
        QString pythonExe = "python";

        pythonProcess = new QProcess(this);
        connect(pythonProcess, &QProcess::finished, this, [=]() {
            qDebug() << "[C++] Python viewer finished.";
            pythonProcess->deleteLater();
            pythonProcess = nullptr;
            hwndPython = nullptr;
        });
        connect(pythonProcess, &QProcess::readyReadStandardOutput, this, [=]() {
            QByteArray data = pythonProcess->readAllStandardOutput();
            qDebug().noquote() << "[PYTHON STDOUT]" << QString::fromUtf8(data);
        });
        connect(pythonProcess, &QProcess::readyReadStandardError, this, [=]() {
            QByteArray data = pythonProcess->readAllStandardError();
            qDebug().noquote() << "[PYTHON STDERR]" << QString::fromUtf8(data);
        });

        pythonProcess->start(pythonExe, QStringList() << "-u" << pythonScriptPath);

        if (!pythonProcess->waitForStarted(3000)) {
            QMessageBox::warning(this, "Python", "Failed to launch Python viewer.");
            delete pythonProcess;
            pythonProcess = nullptr;
        } else {
            std::cout << "[C++] Python viewer launched." << std::endl;
        }
    }
    QTimer::singleShot(2000, this, [=]() {
        QSet<int> selectedJoints = body->getSelectedJoints();
        send_selections(selectedJoints);

        QTimer::singleShot(500, this, [=]() { // extra send to avoid first drop
            send_selections(selectedJoints);
        });
    });
}

void SimulationPage::openOpensimSimulation(){

}

void SimulationPage::openBoth()
{
    // QString pythonPath = "C:/Program Files/Python38/python.exe";
    // QString pythonScriptPath = "C:/Users/User/OneDrive/Desktop/Python/Edit/imu_csv_recorder/imu_csv_recorder.py";
    // QString pythonDir = "C:/Users/User/OneDrive/Desktop/Python/Edit/imu_csv_recorder";
    // QString motionPath = "C:/Users/rcc/Desktop/opemsim_2/IMU_Simulation_OpenSim_new/main.py";
    // QString motionDir = "C:/Users/rcc/Desktop/opemsim_2/IMU_Simulation_OpenSim_new";

    // اجرای Python script
    // QProcess::startDetached(pythonPath, QStringList() << pythonScriptPath, pythonDir);

    // اجرای Motion
    // bool success = QProcess::startDetached(pythonPath, QStringList() << motionPath, motionDir);
    // if (success) {
    //     QTimer::singleShot(2000, this, &SimulationPage::embedPythonWindow);  // تاخیر برای اجرای جاسازی
    // } else {
    //     QMessageBox::warning(this, "Error", "Failed to launch Motion.");
    // }
}

void SimulationPage::stopWorkerThread() {
    if (workerThread && workerThread->isRunning()) {
        worker->stop();  // Signal worker to stop processing
        workerThread->quit();  // Request thread to exit
        workerThread->wait();  // Wait for thread to finish
    }
}

void SimulationPage::closeMeshSimulation() {
    isSimulating = false;

    if (offlineTimer) {
        offlineTimer->stop();
    }
    // if (npzTimer) {
    //     npzTimer->stop();
    // }

    if (worker) {
        worker->stop();
    }

    currentOfflineRow = 0;
    // currentNpzFrame = 0;
    lastImuData.clear();
    avg_state = nullptr;

    btnOpenMesh->setEnabled(true);
    btnCloseMesh->setEnabled(false);
    selectModel->setEnabled(true);

    disconnect(usbReceiver, &USBReceiver::data, this, &SimulationPage::onIMUData);

    if (pythonProcess) {
        pythonProcess->kill();
        pythonProcess->waitForFinished(1000);
        delete pythonProcess;
        pythonProcess = nullptr;
    }
#ifdef Q_OS_WIN
    if (hwndPython) {
        PostMessage(hwndPython, WM_CLOSE, 0, 0);
        hwndPython = nullptr;
    }
#endif
    if (csvFile.is_open()) {
        csvFile.flush();
        csvFile.close();
    }
}

SimulationPage::~SimulationPage()
{
    if (logFile) {
        logMessage("SimulationPage destructor called");
        logFile->close();
        delete logFile;
    }
    running = false;
    cv.notify_all();

    clearTraces();

    // delete all entityها در pool (فقط در destructor، برای cleanup نهایی)
    for (auto entity : tracePool) {
        if (entity) {
            entity->deleteLater();  // safe delete
        }
    }
    tracePool.clear();

    if (offlineTimer) {
        offlineTimer->stop();
        delete offlineTimer;
        offlineTimer = nullptr;
    }
    // if (npzTimer) {
    //     npzTimer->stop();
    //     delete npzTimer;
    //     npzTimer = nullptr;
    // }

    stopWorkerThread();

    if (pythonProcess) {
        pythonProcess->kill();
        pythonProcess->waitForFinished(1000);
        delete pythonProcess;
        pythonProcess = nullptr;
    }

#ifdef Q_OS_WIN
    if (hwndPython) {
        PostMessage(hwndPython, WM_CLOSE, 0, 0);
        hwndPython = nullptr;
    }
#endif

    if (csvFile.is_open()) {
        csvFile.close();
    }
}

void SimulationPage::handleUSBConnection(bool connected)
{
    isUSBConnected = connected;
}

void SimulationPage::embedPythonWindow()
{
#ifdef Q_OS_WIN
    const wchar_t* possibleTitles[] = {
        L"OpenSim 4.5: python (MASI_HMaleMuscle)",
        L"OpenSim",
        L"Simbody Visualizer",
        L"Real-time Mesh"
    };

    hwndPython = nullptr;
    while (!hwndPython) {
        for (const wchar_t* title : possibleTitles) {
            hwndPython = FindWindowW(nullptr, title);
            if (hwndPython) break;
        }
    }

    if (!hwndPython) {
        QMessageBox::warning(this, "Error", "Failed to find Python visualizer window.");
        return;
    }

    HWND hwndQt = reinterpret_cast<HWND>(plotterPlaceholder->winId());
    SetParent(hwndPython, hwndQt);
    SetWindowLong(hwndPython, GWL_STYLE, WS_VISIBLE);
    RECT rect;
    GetClientRect(hwndQt, &rect);
    MoveWindow(hwndPython, 0, 0, (rect.right - rect.left), (rect.bottom - rect.top), TRUE);
#endif
}
/*
void SimulationPage::onCoordinateTest() {
    CountdownDialog* dialog = new CountdownDialog("Hold Y axis upward",isDarkTheme, this);
    connect(dialog, &CountdownDialog::countdownStarted, [&]() {
        isCoordinateTesting = true;
        sampleCount = 0;
    });
    connect(dialog, &CountdownDialog::countdownFinished, this, &SimulationPage::processCoordinateTest);
    dialog->exec();
    updateGuideBox(false);
}
*/

/*
void SimulationPage::onTPoseTest() {
    CountdownDialog* dialog = new CountdownDialog("Please hold T-pose position",isDarkTheme, this);
    connect(dialog, &CountdownDialog::countdownStarted, [&]() {
        isTPoseTesting = true;
        sampleCount = 0;
    });
    connect(dialog, &CountdownDialog::countdownFinished, this, &SimulationPage::processTPoseTest);
    dialog->exec();
    updateGuideBox(false);
}
*/
void SimulationPage::processPoseTest() {
    QString selectedPose = poseComboBox->currentText();
    int numSamples = (selectedPose == "N-Pose") ? 800 : 1000;

    if (pose_data.rows() < numSamples * 6) {  // 6 joints
        QMessageBox::critical(this, "Error", QString("Insufficient data for %1 test. Required: at least %2 rows.").arg(selectedPose).arg(numSamples * 6));
        return;
    }

    if (coord_data.rows() < numSamples) {
        QMessageBox::critical(this, "Error", QString("Insufficient coordinate data. Required: %1 rows.").arg(numSamples));
        return;
    }

    // ساخت calibrator بر اساس pose انتخابی
    try {
        if (selectedPose == "N-Pose") {

            calibrator = std::make_unique<NposeCalibrator>(selected_imus, coord_data, pose_data);
        } else {
            // qDebug() << "pose_data shape: rows=" << pose_data.rows() << ", cols=" << pose_data.cols();
            // for (int i = 4000; i < 4300; ++i) {
            //     qDebug() << "Row" << i << ": ["
            //              << pose_data(i, 0) << ", "  // quat_w
            //              << pose_data(i, 1) << ", "  // quat_x
            //              << pose_data(i, 2) << ", "  // quat_y
            //              << pose_data(i, 3) << "]";  // quat_z
            // }
            // for (int i = 7000; i < 7300; ++i) {
            //     qDebug() << "Row" << i << ": ["
            //              << pose_data(i, 0) << ", "  // quat_w
            //              << pose_data(i, 1) << ", "  // quat_x
            //              << pose_data(i, 2) << ", "  // quat_y
            //              << pose_data(i, 3) << "]";  // quat_z
            // }
            // for (int i = 8000; i < 8300; ++i) {
            //     qDebug() << "Row" << i << ": ["
            //              << pose_data(i, 0) << ", "  // quat_w
            //              << pose_data(i, 1) << ", "  // quat_x
            //              << pose_data(i, 2) << ", "  // quat_y
            //              << pose_data(i, 3) << "]";  // quat_z
            // }
            // for (int i = 9000; i < 9300; ++i) {
            //     qDebug() << "Row" << i << ": ["
            //              << pose_data(i, 0) << ", "  // quat_w
            //              << pose_data(i, 1) << ", "  // quat_x
            //              << pose_data(i, 2) << ", "  // quat_y
            //              << pose_data(i, 3) << "]";  // quat_z
            // }
            // for (int i = 10000; i < 10300; ++i) {
            //     qDebug() << "Row" << i << ": ["
            //              << pose_data(i, 0) << ", "  // quat_w
            //              << pose_data(i, 1) << ", "  // quat_x
            //              << pose_data(i, 2) << ", "  // quat_y
            //              << pose_data(i, 3) << "]";  // quat_z
            // }
            // for (int i = 11000; i < 11300; ++i) {
            //     qDebug() << "Row" << i << ": ["
            //              << pose_data(i, 0) << ", "  // quat_w
            //              << pose_data(i, 1) << ", "  // quat_x
            //              << pose_data(i, 2) << ", "  // quat_y
            //              << pose_data(i, 3) << "]";  // quat_z
            // }
            // qDebug() << "coord_data shape: rows=" << coord_data.rows() << ", cols=" << coord_data.cols();
            // for (int i = 0; i < 300; ++i) {
            //     qDebug() << "Row" << i << ": ["
            //              << coord_data(i, 0) << ", "  // quat_w
            //              << coord_data(i, 1) << ", "  // quat_x
            //              << coord_data(i, 2) << ", "  // quat_y
            //              << coord_data(i, 3) << "]";  // quat_z
            // }

            
            // calibrator = std::make_unique<Calibrator>(selected_imus, coord_data, pose_data);
            std::vector<Eigen::Vector4f> avg_quat_scaled;
            avg_quat_scaled.reserve(avg_quat.size());

            for (const auto& q : avg_quat) {
                avg_quat_scaled.push_back(q / sampleCount);
            }

            calibrator = std::make_unique<Calibrator>(selected_imus, coord_data, pose_data, avg_quat_scaled);
        }
    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Error", QString("Calibration failed: %1").arg(e.what()));
        return;
    }

    worker->setCalibrator(std::move(calibrator));

    QMessageBox::information(this, selectedPose + " Test", selectedPose + " test completed. Calibration matrices calculated.");
}

void SimulationPage::updateSelectedImus(const QMap<QString, QVariant> &selectedImus) {
    qDebug() << "updateSelectedImus called with" << selectedImus.size() << "items";
    selected_imus.clear();
    int max_id = 0;
    for (auto it = selectedImus.constBegin(); it != selectedImus.constEnd(); ++it) {
        int id = it.value().toInt();
        selected_imus[it.key().toLower().toStdString()] = id;
        if (id > max_id) max_id = id;
    }
    // تغییر اندازه tpose_data بر اساس بزرگ‌ترین id
    pose_data.resize((max_id + 1) * 1000, 7);
    qDebug() << "Resized tpose_data to" << pose_data.rows() << "rows";
    qDebug() << "Updated Selected IMUs:";
    for (const auto& pair : selected_imus) {
        qDebug() << QString::fromStdString(pair.first) << ":" << pair.second;
    }
}

void SimulationPage::updateGuideBox(bool isCoordinateTest) {
    QString selectedPose = poseComboBox->currentText();
    if (isCoordinateTest) {
        guideLabel->setText("Coordinate Test");
        QPixmap imuPixmap("icons/imu_icon.png");
        if (!imuPixmap.isNull()) {
            guideImage->setPixmap(imuPixmap.scaled(100, 100, Qt::KeepAspectRatio, Qt::SmoothTransformation));
        } else {
            guideImage->setText("IMU Image Not Found");
        }
    } else {
        QString poseText = (selectedPose == "N-Pose") ? "N-Pose Test" : "T-Pose Test";
        guideLabel->setText(poseText);
        QString poseIconPath = (selectedPose == "N-Pose") ? "icons/npose_icon.png" : "icons/tpose_icon.png";
        QPixmap posePixmap(poseIconPath);
        if (!posePixmap.isNull()) {
            guideImage->setPixmap(posePixmap.scaled(100, 100, Qt::KeepAspectRatio, Qt::SmoothTransformation));
        } else {
            guideImage->setText(poseText + " Image Not Found");
        }
    }
}

// void SimulationPage::startCalibrationTests() {
//     updateGuideBox(true); // نمایش باکس راهنمای Coordinate Test
//     CountdownDialog* dialog = new CountdownDialog("Hold Y axis upward", isDarkTheme, this);
//     connect(dialog, &CountdownDialog::countdownStarted, this, [&]() {
//         isCoordinateTesting = true;
//         sampleCount = 0;
//     });
//     connect(dialog, &CountdownDialog::countdownFinished, this, &SimulationPage::handleCoordinateTestFinished);
//     dialog->exec();
// }
void SimulationPage::startCalibrationTests() {
    updateGuideBox(true); // نمایش باکس راهنمای Coordinate Test
    CountdownDialog* dialog = new CountdownDialog("Hold Y axis upward", isDarkTheme, this);
    connect(dialog, &CountdownDialog::countdownStarted, this, [&]() {
        isCoordinateTesting = true;
        sampleCount = 0;
    });
    connect(dialog, &CountdownDialog::countdownFinished, this, &SimulationPage::handleCoordinateTestFinished);
    connect(usbReceiver, &USBReceiver::data, this, &SimulationPage::onIMUData);
    dialog->exec();
}
void SimulationPage::handleCoordinateTestFinished() {
    updateGuideBox(false); // تغییر به باکس راهنمای Pose Test
    QString selectedPose = poseComboBox->currentText();
    QString countdownMessage = (selectedPose == "N-Pose") ? "Please hold N-pose position" : "Please hold T-pose position";
    CountdownDialog* dialog = new CountdownDialog(countdownMessage, isDarkTheme, this);
    connect(dialog, &CountdownDialog::countdownStarted, this, [&]() {
        isTPoseTesting = true;
        isCoordinateTesting = false;
        sampleCount = 0;
    });
    connect(dialog, &CountdownDialog::countdownFinished, this, &SimulationPage::handlePoseTestFinished);
    dialog->exec();
}

void SimulationPage::handlePoseTestFinished() {
    processPoseTest(); // اجرای پردازش Pose Test
    btnOpenMesh->setEnabled(true);
}

void SimulationPage::updateJointTraces(const std::vector<QVector3D>& currentJoints) {
    // logMessage("updateJointTraces called with " + QString::number(currentJoints.size()) + " joints");
    if (!traceEnabled || currentJoints.empty() || currentJoints.size() != jointTraces.size()) {
        clearTraces();  // اگر mismatch، clear کن
        return;
    }

    // برای هر joint
    for (size_t i = 0; i < jointTraces.size(); ++i) {
        auto& trace = jointTraces[i];

        // اضافه کردن موقعیت جدید
        QElapsedTimer timer;
        timer.start();
        trace.push_back({currentJoints[i], timer});

        // حذف قدیمی‌ها
        while (!trace.empty() && (trace.size() > maxTracePoints || trace.front().second.elapsed() > traceDurationMs)) {
            trace.pop_front();
        }
    }

    for (auto entity : traceEntities) {
        if (entity) {
            entity->setEnabled(false);  // hide بدون delete
            tracePool.push_back(entity);  // back to pool برای reuse بعدی
        }
    }
    traceEntities.clear();

    // رندر trace جدید برای همه joints
    for (size_t i = 0; i < jointTraces.size(); ++i) {
        const auto& trace = jointTraces[i];
        for (size_t j = 0; j < trace.size(); ++j) {
            const auto& pos = trace[j].first;
            float opacity = 1.0f - (static_cast<float>(trace[j].second.elapsed()) / traceDurationMs);
            if (opacity <= 0.0f) continue;

            Qt3DCore::QEntity* traceEntity;
            if (!tracePool.empty()) {
                traceEntity = tracePool.back();  // reuse از pool
                tracePool.pop_back();
                traceEntity->setEnabled(true);  // show کن
            } else {
                // اگر pool خالی، new entity بساز
                traceEntity = new Qt3DCore::QEntity(rootEntity);
                Qt3DExtras::QSphereMesh* sphere = new Qt3DExtras::QSphereMesh();
                sphere->setRadius(traceSphereRadius);
                sphere->setSlices(10);
                sphere->setRings(10);
                traceEntity->addComponent(sphere);

                // داخل else (ایجاد entity جدید)
                Qt3DExtras::QPhongMaterial* material = new Qt3DExtras::QPhongMaterial();
                material->setDiffuse(QColor(255, 0, 0, static_cast<int>(opacity * 255)));  // alpha در QColor
                // حذف: material->setAlphaBlendingEnabled(true);
                traceEntity->addComponent(material);

                Qt3DCore::QTransform* transform = new Qt3DCore::QTransform();
                traceEntity->addComponent(transform);
            }

            // آپدیت material و transform (چون reuse، باید update کنی)
            auto material = qobject_cast<Qt3DExtras::QPhongMaterial*>(traceEntity->componentsOfType<Qt3DExtras::QPhongMaterial>()[0]);
            if (material) {
                material->setDiffuse(QColor(255, 0, 0, static_cast<int>(opacity * 255)));
            }

            auto transform = qobject_cast<Qt3DCore::QTransform*>(traceEntity->componentsOfType<Qt3DCore::QTransform>()[0]);
            if (transform) {
                transform->setTranslation(pos);
            }

            traceEntities.push_back(traceEntity);
        }
    }
}

void SimulationPage::clearTraces() {
    for (auto entity : traceEntities) {
        if (entity) {
            entity->setEnabled(false);  // hide
            tracePool.push_back(entity);  // back to pool
        }
    }
    traceEntities.clear();

    for (auto& trace : jointTraces) {
        trace.clear();
    }
}

void SimulationPage::logMessage(const QString& msg) {
    if (logFile && logFile->isOpen()) {
        QTextStream out(logFile);
        out << QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss") << " - " << msg << "\n";
        out.flush();
    }
    qDebug() << msg;  // هم به console
}
