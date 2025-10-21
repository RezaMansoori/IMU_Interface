#include "object3d.h"
#include <QMatrix4x4>
#include <QVector3D>
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QMouseEvent>
#include <QWheelEvent>
#include <cmath>

Object3D::Object3D(QWidget *parent)
    : QOpenGLWidget(parent),
      modelProgram(nullptr),
      axesProgram(nullptr),
      modelVbo(QOpenGLBuffer::VertexBuffer),
      modelEbo(QOpenGLBuffer::IndexBuffer),
      axesVbo(QOpenGLBuffer::VertexBuffer),
      axesEbo(QOpenGLBuffer::IndexBuffer),
      modelVertexCount(0),
      axesVertexCount(0),
      useFallbackSphere(false),
      cameraDistance(3.0f),
      cameraAzimuth(0.0f),
      cameraElevation(0.0f)
{
    setMouseTracking(true);
}

Object3D::~Object3D()
{
    modelVao.destroy();
    modelVbo.destroy();
    modelEbo.destroy();
    axesVao.destroy();
    axesVbo.destroy();
    axesEbo.destroy();
    delete modelProgram;
    delete axesProgram;
}

void Object3D::setRotation(const QQuaternion &rotation)
{
    m_rotation = rotation;
    update();
}

QQuaternion Object3D::getRotation() const
{
    return m_rotation;
}

void Object3D::resetCamera()
{
    cameraDistance = 3.0f;
    cameraAzimuth = 0.0f;
    cameraElevation = 0.0f;
    updateViewMatrix();
    update();
}

void Object3D::loadOBJ(const QString &filePath)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Failed to open OBJ file:" << filePath;
        useFallbackSphere = true;
        createFallbackSphere();
        return;
    }

    useFallbackSphere = false;
    QVector<QVector3D> tempVertices;
    QVector<QVector3D> tempNormals;
    modelVertices.clear();
    modelIndices.clear();

    QTextStream in(&file);
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        QStringList tokens = line.split(' ', Qt::SkipEmptyParts);

        if (tokens.isEmpty()) continue;

        if (tokens[0] == "v") {
            if (tokens.size() >= 4) {
                float x = tokens[1].toFloat();
                float y = tokens[2].toFloat();
                float z = tokens[3].toFloat();
                tempVertices.append(QVector3D(x, y, z));
            }
        } else if (tokens[0] == "vn") {
            if (tokens.size() >= 4) {
                float x = tokens[1].toFloat();
                float y = tokens[2].toFloat();
                float z = tokens[3].toFloat();
                tempNormals.append(QVector3D(x, y, z));
            }
        } else if (tokens[0] == "f") {
            for (int i = 1; i < tokens.size(); ++i) {
                QStringList faceData = tokens[i].split('/');
                if (faceData.size() >= 3) {
                    int vertexIndex = faceData[0].toInt() - 1;
                    int normalIndex = faceData[2].toInt() - 1;
                    if (vertexIndex >= 0 && vertexIndex < tempVertices.size() && normalIndex >= 0 && normalIndex < tempNormals.size()) {
                        modelVertices.append(tempVertices[vertexIndex].x());
                        modelVertices.append(tempVertices[vertexIndex].y());
                        modelVertices.append(tempVertices[vertexIndex].z());
                        modelVertices.append(tempNormals[normalIndex].x());
                        modelVertices.append(tempNormals[normalIndex].y());
                        modelVertices.append(tempNormals[normalIndex].z());
                        modelIndices.append(modelIndices.size());
                    }
                }
            }
        }
    }

    file.close();
    modelVertexCount = modelIndices.size();
}

void Object3D::createFallbackSphere()
{
    modelVertices.clear();
    modelIndices.clear();
    const int stacks = 20;
    const int slices = 20;
    const float radius = 0.5f;

    for (int i = 0; i <= stacks; ++i) {
        float phi = M_PI * i / stacks;
        float sinPhi = sin(phi);
        float cosPhi = cos(phi);
        for (int j = 0; j <= slices; ++j) {
            float theta = 2 * M_PI * j / slices;
            float sinTheta = sin(theta);
            float cosTheta = cos(theta);
            float x = radius * sinPhi * cosTheta;
            float y = radius * sinPhi * sinTheta;
            float z = radius * cosPhi;
            modelVertices.append(x);
            modelVertices.append(y);
            modelVertices.append(z);
            modelVertices.append(x);
            modelVertices.append(y);
            modelVertices.append(z);
        }
    }

    for (int i = 0; i < stacks; ++i) {
        for (int j = 0; j < slices; ++j) {
            int first = i * (slices + 1) + j;
            int second = first + slices + 1;
            modelIndices.append(first);
            modelIndices.append(second);
            modelIndices.append(first + 1);
            modelIndices.append(second);
            modelIndices.append(second + 1);
            modelIndices.append(first + 1);
        }
    }

    modelVertexCount = modelIndices.size();
}

void Object3D::createAxes()
{
    axesVertices.clear();
    axesIndices.clear();

    // X-axis (red)
    axesVertices.append(0.0f); axesVertices.append(0.0f); axesVertices.append(0.0f); // Position
    axesVertices.append(1.0f); axesVertices.append(0.0f); axesVertices.append(0.0f); // Color (Red)
    axesVertices.append(1.0f); axesVertices.append(0.0f); axesVertices.append(0.0f); // Position
    axesVertices.append(1.0f); axesVertices.append(0.0f); axesVertices.append(0.0f); // Color (Red)

    // Y-axis (green)
    axesVertices.append(0.0f); axesVertices.append(0.0f); axesVertices.append(0.0f); // Position
    axesVertices.append(0.0f); axesVertices.append(1.0f); axesVertices.append(0.0f); // Color (Green)
    axesVertices.append(0.0f); axesVertices.append(1.0f); axesVertices.append(0.0f); // Position
    axesVertices.append(0.0f); axesVertices.append(1.0f); axesVertices.append(0.0f); // Color (Green)

    // Z-axis (blue)
    axesVertices.append(0.0f); axesVertices.append(0.0f); axesVertices.append(0.0f); // Position
    axesVertices.append(0.0f); axesVertices.append(0.0f); axesVertices.append(1.0f); // Color (Blue)
    axesVertices.append(0.0f); axesVertices.append(0.0f); axesVertices.append(1.0f); // Position
    axesVertices.append(0.0f); axesVertices.append(0.0f); axesVertices.append(1.0f); // Color (Blue)

    axesIndices.append(0); axesIndices.append(1);
    axesIndices.append(2); axesIndices.append(3);
    axesIndices.append(4); axesIndices.append(5);

    axesVertexCount = axesIndices.size();
}

void Object3D::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(35.0f / 255.0f, 37.0f / 255.0f, 38.0f / 255.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);

    loadOBJ("C:/Users/ACER/Documents/Code/IMU_Interfacevfinal_mohandes/IMU_Interfacevfinal/build/Desktop_Qt_6_9_2_MSVC_64_bit-Release/icons/imu-model.obj");
    createAxes();

    modelProgram = new QOpenGLShaderProgram(this);
    modelProgram->addShaderFromSourceCode(QOpenGLShader::Vertex,
        "#version 330 core\n"
        "in vec3 inPos;\n"
        "in vec3 inNorm;\n"
        "uniform mat4 proj;\n"
        "uniform mat4 view;\n"
        "uniform mat4 model;\n"
        "out vec3 fragPos;\n"
        "out vec3 fragNorm;\n"
        "void main() {\n"
        "    fragPos = vec3(model * vec4(inPos, 1.0));\n"
        "    fragNorm = mat3(transpose(inverse(model))) * inNorm;\n"
        "    gl_Position = proj * view * model * vec4(inPos, 1.0);\n"
        "}\n");
    modelProgram->addShaderFromSourceCode(QOpenGLShader::Fragment,
        "#version 330 core\n"
        "in vec3 fragPos;\n"
        "in vec3 fragNorm;\n"
        "uniform vec3 objectColor;\n"
        "uniform vec3 lightPos;\n"
        "uniform vec3 viewPos;\n"
        "const vec3 lightColor = vec3(0.0, 0.2, 0.8);\n"
        "out vec4 fragColor;\n"
        "void main() {\n"
        "    float ambientStrength = 0.05;\n"
        "    vec3 ambient = ambientStrength * lightColor;\n"
        "    vec3 norm = normalize(fragNorm);\n"
        "    vec3 lightDir = normalize(lightPos - fragPos);\n"
        "    float diff = max(dot(norm, lightDir), 0.0);\n"
        "    vec3 diffuse = diff * lightColor;\n"
        "    vec3 viewDir = normalize(viewPos - fragPos);\n"
        "    vec3 reflectDir = reflect(-lightDir, norm);\n"
        "    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);\n"
        "    vec3 specular = 0.5 * spec * lightColor;\n" // تطابق با specular=0.5 در پایتون
        "    vec3 result = (ambient + diffuse + specular) * objectColor;\n"
        "    fragColor = vec4(result, 1.0);\n"
        "}\n");
    modelProgram->link();

    axesProgram = new QOpenGLShaderProgram(this);
    axesProgram->addShaderFromSourceCode(QOpenGLShader::Vertex,
        "#version 330 core\n"
        "in vec3 inPos;\n"
        "in vec3 inColor;\n"
        "uniform mat4 proj;\n"
        "uniform mat4 view;\n"
        "uniform mat4 model;\n"
        "out vec3 color;\n"
        "void main() {\n"
        "    color = inColor;\n"
        "    gl_Position = proj * view * model * vec4(inPos, 1.0);\n"
        "}\n");
    axesProgram->addShaderFromSourceCode(QOpenGLShader::Fragment,
        "#version 330 core\n"
        "in vec3 color;\n"
        "out vec4 fragColor;\n"
        "void main() {\n"
        "    fragColor = vec4(color, 1.0);\n"
        "}\n");
    axesProgram->link();

    modelVao.create();
    modelVao.bind();
    modelVbo.create();
    modelVbo.bind();
    modelVbo.allocate(modelVertices.constData(), modelVertices.size() * sizeof(float));
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    modelEbo.create();
    modelEbo.bind();
    modelEbo.allocate(modelIndices.constData(), modelIndices.size() * sizeof(unsigned int));
    modelVao.release();
    modelVbo.release();
    modelEbo.release();

    axesVao.create();
    axesVao.bind();
    axesVbo.create();
    axesVbo.bind();
    axesVbo.allocate(axesVertices.constData(), axesVertices.size() * sizeof(float));
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    axesEbo.create();
    axesEbo.bind();
    axesEbo.allocate(axesIndices.constData(), axesIndices.size() * sizeof(unsigned int));
    axesVao.release();
    axesVbo.release();
    axesEbo.release();

    updateViewMatrix();
}

void Object3D::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
    projection.setToIdentity();
    projection.perspective(45.0f, (float)w / h, 0.1f, 100.0f);
}

void Object3D::updateViewMatrix()
{
    view.setToIdentity();
    float radAzimuth = cameraAzimuth * M_PI / 180.0f;
    float radElevation = cameraElevation * M_PI / 180.0f;
    float x = cameraDistance * cos(radElevation) * sin(radAzimuth);
    float y = cameraDistance * cos(radElevation) * cos(radAzimuth);
    float z = cameraDistance * sin(radElevation);
    view.lookAt(QVector3D(x, y, z), QVector3D(0.0f, 0.0f, 0.0f), QVector3D(0.0f, 0.0f, 1.0f));
}

void Object3D::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // رندر مدل
    modelProgram->bind();
    modelVao.bind();
    modelMatrix.setToIdentity();
    modelMatrix.translate(0.0f, 0.0f, 0.0f);

    // چرخش اصلاحی اولیه برای مدل (جبران 90 درجه به چپ حول محور Y)
    QQuaternion correction = QQuaternion::fromAxisAndAngle(0, 0, 1, -270);
    modelMatrix.rotate(correction);

    // سپس اعمال چرخش دریافتی از IMU
    modelMatrix.rotate(m_rotation);

    modelProgram->setUniformValue("model", modelMatrix);
    modelProgram->setUniformValue("view", view);
    modelProgram->setUniformValue("proj", projection);
    modelProgram->setUniformValue("objectColor", QVector3D(0.263f, 0.808f, 0.635f)); // رنگ #43cea2
    modelProgram->setUniformValue("lightPos", QVector3D(0.0f, 0.0f, 5.0f));
    modelProgram->setUniformValue("viewPos", QVector3D(0.0f, -3.0f, 0.0f));
    glDrawElements(GL_TRIANGLES, modelVertexCount, GL_UNSIGNED_INT, 0);
    modelVao.release();
    modelProgram->release();

    // رندر محورها با ضخامت مشابه پایتون
    glLineWidth(2.0f); // ضخامت خطوط معادل شعاع سیلندر 0.02 در پایتون
    axesProgram->bind();
    axesVao.bind();
    axesMatrix.setToIdentity();
    axesProgram->setUniformValue("model", axesMatrix);
    axesProgram->setUniformValue("view", view);
    axesProgram->setUniformValue("proj", projection);
    glDrawElements(GL_LINES, axesVertexCount, GL_UNSIGNED_INT, 0);
    axesVao.release();
    axesProgram->release();
    glLineWidth(1.0f); // بازگرداندن ضخامت پیش‌فرض
}

void Object3D::mousePressEvent(QMouseEvent *event)
{
    lastMousePos = event->pos();
}

void Object3D::mouseMoveEvent(QMouseEvent *event)
{
    if (event->buttons() & Qt::LeftButton) {
        int dx = event->x() - lastMousePos.x();
        int dy = event->y() - lastMousePos.y();
        cameraAzimuth -= dx * 0.5f;
        cameraElevation -= dy * 0.5f;
        cameraElevation = qBound(-89.0f, cameraElevation, 89.0f);
        updateViewMatrix();
        update();
    } else if (event->buttons() & Qt::MiddleButton) {
        int dx = event->x() - lastMousePos.x();
        int dy = event->y() - lastMousePos.y();
        float sensitivity = 0.01f * cameraDistance;
        QMatrix4x4 invView = view.inverted();
        QVector3D right = QVector3D(invView(0,0), invView(1,0), invView(2,0)).normalized();
        QVector3D up = QVector3D(invView(0,1), invView(1,1), invView(2,1)).normalized();
        QVector3D translation = -right * dx * sensitivity + up * dy * sensitivity;
        view.translate(translation);
        update();
    } else if (event->buttons() & Qt::RightButton) {
        int dy = event->y() - lastMousePos.y();
        cameraDistance += dy * 0.05f;
        cameraDistance = qBound(0.1f, cameraDistance, 100.0f);
        updateViewMatrix();
        update();
    }
    lastMousePos = event->pos();
}

void Object3D::wheelEvent(QWheelEvent *event)
{
    float delta = event->angleDelta().y() / 120.0f;
    cameraDistance -= delta * 0.2f * cameraDistance;
    cameraDistance = qBound(0.1f, cameraDistance, 100.0f);
    updateViewMatrix();
    update();
}
