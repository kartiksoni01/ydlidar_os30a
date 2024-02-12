#ifndef CIMUDATAVIEWERWIDGET_H
#define CIMUDATAVIEWERWIDGET_H
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QQuaternion>
#include "CIMUModel.h"
#include "IMU/IMU_Calibration/CIMUCalibration.h"

struct CVertexData{
    float x;
    float y;
    float z;
    float r;
    float g;
    float b;
    float a;
};

class IMUFilter;
class CIMUDataViewerWidget : public QOpenGLWidget,
                             protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit CIMUDataViewerWidget(QWidget *parent = 0);
    ~CIMUDataViewerWidget();

    virtual void initializeGL();
    virtual void resizeGL(int w, int h);
    virtual void paintGL();

    void initShaders();
    void PrepareAxisData();
    void PrepareDeviceBoxData();

    void drawAxis();
    void drawDeviceBox();

    QMatrix4x4 GetRotateMatrix();

    void updateData(IMUData *pIMUData, CIMUModel::TYPE imuType);

private:
    QMatrix4x4 m_rotateMatrix;
    QMatrix4x4 m_modelMatrix;
    QMatrix4x4 m_viewMatrix;
    QMatrix4x4 m_perspectiveMatrix;
    QMatrix4x4 m_orthoMartix;

    QOpenGLShaderProgram m_program;
    QOpenGLBuffer m_xAxisBuffer;
    QOpenGLBuffer m_yAxisBuffer;
    QOpenGLBuffer m_zAxisBuffer;
    QOpenGLBuffer m_deviceBoxBuffer;

    std::vector<float> m_vertices;
    std::vector<unsigned char> m_colors;

    IMUFilter *m_pIMUfilter;
    QQuaternion m_quaternion;

    CIMUCalibration m_pIMUCalibration;
};

#endif // CIMUDATAVIEWERWIDGET_H
