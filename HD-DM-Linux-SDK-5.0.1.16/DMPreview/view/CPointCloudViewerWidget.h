#ifndef CPOINTCLOUDVIEWERWIDGET_H
#define CPOINTCLOUDVIEWERWIDGET_H
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QMutex>
#include <vector>
#include "FPSCalculator.h"
#include "ArcBall.h"

#define DOUBLE_CLICK_INTERVAL (500)

class CVideoDeviceController;
class CPointCloudViewerWidget : public QOpenGLWidget,
                                protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit CPointCloudViewerWidget(CVideoDeviceController *pVideoDeviceController,
                                     QWidget *parent = 0);
    ~CPointCloudViewerWidget();

    virtual void mouseDoubleClickEvent(QMouseEvent *event);
    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseReleaseEvent(QMouseEvent *event);
    virtual void mouseMoveEvent(QMouseEvent *event);
    virtual void wheelEvent(QWheelEvent *event);

    virtual void initializeGL();
    virtual void resizeGL(int w, int h);
    virtual void paintGL();

    void updateData(std::vector<float> &vertices, std::vector<unsigned char> &colors);

    void initShaders();

    void PrepareCircleData();

    void drawBackground();
    void darwPointCloud();
    void drawAxis();

    float GetFPS(){ return m_fpsCalculator.GetFPS(); }

private:
    CVideoDeviceController *m_pVideoDeviceController;

    QMatrix4x4 m_rotateMatrix;
    QMatrix4x4 m_modelMatrix;
    QMatrix4x4 m_viewMatrix;
    QMatrix4x4 m_perspectiveMatrix;
    QMatrix4x4 m_orthoMartix;

    CArcBall m_arcBall;
    float m_fScrollZ;
    float m_fMoveX, m_fMoveY;
    int m_nLastMouseX, m_nLastMouseY;

    QOpenGLShaderProgram m_program;
    QOpenGLBuffer m_verticesBuffer;
    QOpenGLBuffer m_colorsBuffer;

    std::vector<float> m_vertices;
    std::vector<unsigned char> m_colors;

    std::vector<float> m_circleVertices;
    std::vector<unsigned char> m_circleColors;

    FPSCalculator m_fpsCalculator;

    QMutex m_mutex;
};

#endif // CPOINTCLOUDVIEWERWIDGET_H
