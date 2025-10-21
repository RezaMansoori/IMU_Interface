#ifndef OBJECT3D_H
#define OBJECT3D_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLTexture>
#include <QQuaternion>
#include <QVector>
#include <QMatrix4x4>
#include <QPoint>
#include <QString>

class Object3D : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit Object3D(QWidget *parent = nullptr);
    ~Object3D();
    void setRotation(const QQuaternion &rotation);
    QQuaternion getRotation() const;
    void resetCamera();

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    void loadOBJ(const QString &filePath);
    void createFallbackSphere();
    void createCheckerboardFloor();
    void createAxes();
    void updateViewMatrix();

    QQuaternion m_rotation;
    QOpenGLShaderProgram *modelProgram;
    QOpenGLShaderProgram *floorProgram;
    QOpenGLShaderProgram *axesProgram;
    QOpenGLBuffer modelVbo;
    QOpenGLBuffer modelEbo;
    QOpenGLVertexArrayObject modelVao;
    QOpenGLBuffer floorVbo;
    QOpenGLBuffer floorEbo;
    QOpenGLVertexArrayObject floorVao;
    QOpenGLBuffer axesVbo;
    QOpenGLBuffer axesEbo;
    QOpenGLVertexArrayObject axesVao;
    QOpenGLTexture *checkerboardTexture;
    QMatrix4x4 projection;
    QMatrix4x4 view;
    QMatrix4x4 modelMatrix;
    QMatrix4x4 floorMatrix;
    QMatrix4x4 axesMatrix;
    QVector<float> modelVertices;
    QVector<unsigned int> modelIndices;
    QVector<float> floorVertices;
    QVector<unsigned int> floorIndices;
    QVector<float> axesVertices;
    QVector<unsigned int> axesIndices;
    int modelVertexCount;
    int floorVertexCount;
    int axesVertexCount;
    bool useFallbackSphere;
    QPoint lastMousePos;
    float cameraDistance;
    float cameraAzimuth;
    float cameraElevation;
};

#endif // OBJECT3D_H
