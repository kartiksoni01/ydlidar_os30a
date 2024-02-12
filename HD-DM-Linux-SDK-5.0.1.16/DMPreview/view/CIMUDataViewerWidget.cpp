#include "CIMUDataViewerWidget.h"
#include "OpenGLShaderCore.h"
#include <math.h>
#if !defined(Q_PROCESSOR_ARM)
#include <GL/gl.h>
#include <GL/glext.h>
#endif
#include <QTime>
#include "IMU/IMU_Filter/IMUFilter.h"
#include "IMU/IMU_Filter/IMUFilter_AHRS.h"
#include "IMU/IMU_Filter/IMUFilter_Complementary.h"
#include "IMU/IMU_Filter/IMUFilter_EYS3D_AHRS.h"
#include "IMU/IMU_Filter/Quaternion1.h"

CIMUDataViewerWidget::CIMUDataViewerWidget(QWidget *parent):
QOpenGLWidget(parent),
m_xAxisBuffer(QOpenGLBuffer::VertexBuffer),
m_yAxisBuffer(QOpenGLBuffer::VertexBuffer),
m_zAxisBuffer(QOpenGLBuffer::VertexBuffer),
m_deviceBoxBuffer(QOpenGLBuffer::VertexBuffer)
{
    QSurfaceFormat newFormat = format();
    newFormat.setSamples(4);
    newFormat.setDepthBufferSize(32);
    setFormat(newFormat);

    m_pIMUfilter = new IMUFilter_AHRS();
//    m_pIMUfilter = new IMUFilter_Complementary();
//    m_pIMUfilter = new IMUFilter_EYS3D_AHRS();
}

CIMUDataViewerWidget::~CIMUDataViewerWidget()
{
    makeCurrent();
    m_xAxisBuffer.destroy();
    m_yAxisBuffer.destroy();
    m_zAxisBuffer.destroy();
    m_deviceBoxBuffer.destroy();
    doneCurrent();

    if (m_pIMUfilter) delete m_pIMUfilter;
}

void CIMUDataViewerWidget::initializeGL()
{
    initializeOpenGLFunctions();
    initShaders();

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    #ifdef TI_EVM
    glEnable(GL_MULTISAMPLES_NV);
    #else
    glEnable(GL_MULTISAMPLE);
    #endif

    PrepareAxisData();
    PrepareDeviceBoxData();
}

void CIMUDataViewerWidget::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);

    m_orthoMartix.setToIdentity();
    int nZPlane = w < h ? w : h;
    m_orthoMartix.ortho( -w, w, -h, h, -nZPlane, nZPlane);
}

void CIMUDataViewerWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawAxis();
    drawDeviceBox();
}

void CIMUDataViewerWidget::initShaders()
{
    // Compile vertex shader
    if (!m_program.addShaderFromSourceCode(QOpenGLShader::Vertex, imuShaderVert))
        close();

    // Compile fragment shader
    if (!m_program.addShaderFromSourceCode(QOpenGLShader::Fragment, imuShaderFragment))
        close();

    // Link shader pipeline
    if (!m_program.link()){
        printf("%s\n", m_program.log().toLocal8Bit().data());
        close();
    }
    // Bind shader pipeline for use
    if (!m_program.bind())
        close();
}

void CIMUDataViewerWidget::PrepareAxisData()
{
    const float fAxisLength = 2.0f;
    const float fAxisWidth = 0.12f;
    const float fAxisArrowLength = 0.2f;
    const float fAxisArrowWidth = 0.16f;

    CVertexData axisData[] = {
        //Tail
        {0.0f, -fAxisWidth / 2.0, -fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {0.0f, -fAxisWidth / 2.0, fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {0.0f, fAxisWidth / 2.0, -fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {0.0f, fAxisWidth / 2.0, -fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {0.0f, -fAxisWidth / 2.0, fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {0.0f, fAxisWidth / 2.0, fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},

        {0.0f, fAxisWidth / 2.0, fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, fAxisWidth / 2.0, fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, fAxisWidth / 2.0, -fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, fAxisWidth / 2.0, -fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {0.0f, fAxisWidth / 2.0, -fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {0.0f, fAxisWidth / 2.0, fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},

        {0.0f, fAxisWidth / 2.0, fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {0.0f, -fAxisWidth / 2.0, fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, fAxisWidth / 2.0, fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, fAxisWidth / 2.0, fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {0.0f, -fAxisWidth / 2.0, fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, -fAxisWidth / 2.0, fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},

        {fAxisLength, -fAxisWidth / 2.0, fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {0.0f, -fAxisWidth / 2.0, fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, -fAxisWidth / 2.0, -fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, -fAxisWidth / 2.0, -fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {0.0f, -fAxisWidth / 2.0, fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {0.0f, -fAxisWidth / 2.0, -fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},

        {0.0f, -fAxisWidth / 2.0, -fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {0.0f, fAxisWidth / 2.0, -fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, -fAxisWidth / 2.0, -fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, -fAxisWidth / 2.0, -fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {0.0f, fAxisWidth / 2.0, -fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, fAxisWidth / 2.0, -fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},

        {fAxisLength, -fAxisWidth / 2.0, -fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, fAxisWidth / 2.0, -fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, fAxisWidth / 2.0, fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, fAxisWidth / 2.0, fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, -fAxisWidth / 2.0, fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, fAxisWidth / 2.0, -fAxisWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},

        //Arrow
        {fAxisLength, -fAxisArrowWidth / 2.0, -fAxisArrowWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, -fAxisArrowWidth / 2.0, fAxisArrowWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, fAxisArrowWidth / 2.0, -fAxisArrowWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, fAxisArrowWidth / 2.0, -fAxisArrowWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, -fAxisArrowWidth / 2.0, fAxisArrowWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, fAxisArrowWidth / 2.0, fAxisArrowWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},

        {fAxisLength, fAxisArrowWidth / 2.0, fAxisArrowWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength + fAxisArrowLength, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, -fAxisArrowWidth / 2.0, fAxisArrowWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},

        {fAxisLength, fAxisArrowWidth / 2.0, fAxisArrowWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength + fAxisArrowLength, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, fAxisArrowWidth / 2.0, -fAxisArrowWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},

        {fAxisLength, -fAxisArrowWidth / 2.0, -fAxisArrowWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength + fAxisArrowLength, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, fAxisArrowWidth / 2.0, -fAxisArrowWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},

        {fAxisLength, -fAxisArrowWidth / 2.0, -fAxisArrowWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength + fAxisArrowLength, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f},
        {fAxisLength, -fAxisArrowWidth / 2.0, fAxisArrowWidth / 2.0, 0.0f, 0.0f, 0.0f, 1.0f}

    };

    for (CVertexData &vertexData : axisData){
        vertexData.r = 1.0f;
        vertexData.g = 0.0f;
        vertexData.b = 0.0f;
        vertexData.a = 1.0f;
    }

    m_xAxisBuffer.create();
    m_xAxisBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);

    m_xAxisBuffer.bind();
#if !defined(Q_PROCESSOR_ARM)
    if((unsigned int)m_xAxisBuffer.size() < sizeof(axisData)){
        m_xAxisBuffer.allocate(sizeof(axisData));
    }
    auto pXAxisVertices = m_xAxisBuffer.map(QOpenGLBuffer::WriteOnly);
    memcpy(pXAxisVertices, &axisData[0], sizeof(axisData));
    m_xAxisBuffer.unmap();
#else
    m_xAxisBuffer.allocate(&axisData[0], sizeof(axisData));
#endif
    m_xAxisBuffer.release();

    for (CVertexData &vertexData : axisData){
        vertexData.r = 0.0f;
        vertexData.g = 1.0f;
        vertexData.b = 0.0f;
        vertexData.a = 1.0f;
    }

    m_yAxisBuffer.create();
    m_yAxisBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_yAxisBuffer.bind();
#if !defined(Q_PROCESSOR_ARM)
    if((unsigned int)m_yAxisBuffer.size() < sizeof(axisData)){
        m_yAxisBuffer.allocate(sizeof(axisData));
    }
    auto pYAxisVertices = m_yAxisBuffer.map(QOpenGLBuffer::WriteOnly);
    memcpy(pYAxisVertices, &axisData[0], sizeof(axisData));
    m_yAxisBuffer.unmap();
#else
    m_yAxisBuffer.allocate(&axisData[0], sizeof(axisData));
#endif
    m_yAxisBuffer.release();

    for (CVertexData &vertexData : axisData){
        vertexData.r = 0.0f;
        vertexData.g = 0.0f;
        vertexData.b = 1.0f;
        vertexData.a = 1.0f;
    }

    m_zAxisBuffer.create();
    m_zAxisBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_zAxisBuffer.bind();
#if !defined(Q_PROCESSOR_ARM)
    if((unsigned int)m_zAxisBuffer.size() < sizeof(axisData)){
        m_zAxisBuffer.allocate(sizeof(axisData));
    }
    auto pzAxisVertices = m_zAxisBuffer.map(QOpenGLBuffer::WriteOnly);
    memcpy(pzAxisVertices, &axisData[0], sizeof(axisData));
    m_zAxisBuffer.unmap();
#else
    m_zAxisBuffer.allocate(&axisData[0], sizeof(axisData));
#endif
    m_zAxisBuffer.release();
}

void CIMUDataViewerWidget::PrepareDeviceBoxData()
{
    const float fBoxSize = 1.0f;
    const float BOX_ALPHA = 1.0f;
    const float fColorIntensity = 1.0f;

    CVertexData boxData[] = {
        // front
        { -fBoxSize / 2, -fBoxSize / 2, fBoxSize / 2, 0.0f, fColorIntensity, 0.0f, BOX_ALPHA},
        { fBoxSize / 2, -fBoxSize / 2, fBoxSize / 2, 0.0f, fColorIntensity, 0.0f, BOX_ALPHA},
        { fBoxSize / 2, fBoxSize / 2, fBoxSize / 2, 0.0f, fColorIntensity, 0.0f, BOX_ALPHA},
        { fBoxSize / 2, fBoxSize / 2, fBoxSize / 2, 0.0f, fColorIntensity, 0.0f, BOX_ALPHA},
        { -fBoxSize / 2, fBoxSize / 2, fBoxSize / 2, 0.0f, fColorIntensity, 0.0f, BOX_ALPHA},
        { -fBoxSize / 2, -fBoxSize / 2, fBoxSize / 2, 0.0f, fColorIntensity, 0.0f, BOX_ALPHA},

        // bottom
        { -fBoxSize / 2, -fBoxSize / 2, fBoxSize / 2, 0.0f, fColorIntensity, fColorIntensity, BOX_ALPHA},
        { -fBoxSize / 2, -fBoxSize / 2, -fBoxSize / 2, 0.0f, fColorIntensity, fColorIntensity, BOX_ALPHA},
        { fBoxSize / 2, -fBoxSize / 2, -fBoxSize / 2, 0.0f, fColorIntensity, fColorIntensity, BOX_ALPHA},
        { fBoxSize / 2, -fBoxSize / 2, -fBoxSize / 2, 0.0f, fColorIntensity, fColorIntensity, BOX_ALPHA},
        { fBoxSize / 2, -fBoxSize / 2, fBoxSize / 2, 0.0f, fColorIntensity, fColorIntensity, BOX_ALPHA},
        { -fBoxSize / 2, -fBoxSize / 2, fBoxSize / 2, 0.0f, fColorIntensity, fColorIntensity, BOX_ALPHA},

        // left
        { -fBoxSize / 2, -fBoxSize / 2, fBoxSize / 2, fColorIntensity, 0.0f, fColorIntensity, BOX_ALPHA},
        { -fBoxSize / 2, fBoxSize / 2, fBoxSize / 2, fColorIntensity, 0.0f, fColorIntensity, BOX_ALPHA},
        { -fBoxSize / 2, fBoxSize / 2, -fBoxSize / 2, fColorIntensity, 0.0f, fColorIntensity, BOX_ALPHA},
        { -fBoxSize / 2, fBoxSize / 2, -fBoxSize / 2, fColorIntensity, 0.0f, fColorIntensity, BOX_ALPHA},
        { -fBoxSize / 2, -fBoxSize / 2, -fBoxSize / 2, fColorIntensity, 0.0f, fColorIntensity, BOX_ALPHA},
        { -fBoxSize / 2, -fBoxSize / 2, fBoxSize / 2, fColorIntensity, 0.0f, fColorIntensity, BOX_ALPHA},

        // right
        { fBoxSize / 2, fBoxSize / 2, -fBoxSize / 2, fColorIntensity, fColorIntensity, 0.0f, BOX_ALPHA},
        { fBoxSize / 2, fBoxSize / 2, fBoxSize / 2, fColorIntensity, fColorIntensity, 0.0f, BOX_ALPHA},
        { fBoxSize / 2, -fBoxSize / 2, fBoxSize / 2, fColorIntensity, fColorIntensity, 0.0f, BOX_ALPHA},
        { fBoxSize / 2, -fBoxSize / 2, fBoxSize / 2, fColorIntensity, fColorIntensity, 0.0f, BOX_ALPHA},
        { fBoxSize / 2, -fBoxSize / 2, -fBoxSize / 2, fColorIntensity, fColorIntensity, 0.0f, BOX_ALPHA},
        { fBoxSize / 2, fBoxSize / 2, -fBoxSize / 2, fColorIntensity, fColorIntensity, 0.0f, BOX_ALPHA},

        // back
        { fBoxSize / 2, fBoxSize / 2, -fBoxSize / 2, 0.0f, 0.0f, fColorIntensity, BOX_ALPHA},
        { fBoxSize / 2, -fBoxSize / 2, -fBoxSize / 2, 0.0f, 0.0f, fColorIntensity, BOX_ALPHA},
        { -fBoxSize / 2, -fBoxSize / 2, -fBoxSize / 2, 0.0f, 0.0f, fColorIntensity, BOX_ALPHA},
        { -fBoxSize / 2, -fBoxSize / 2, -fBoxSize / 2, 0.0f, 0.0f, fColorIntensity, BOX_ALPHA},
        { -fBoxSize / 2, fBoxSize / 2, -fBoxSize / 2, 0.0f, 0.0f, fColorIntensity, BOX_ALPHA},
        { fBoxSize / 2, fBoxSize / 2, -fBoxSize / 2, 0.0f, 0.0f, fColorIntensity, BOX_ALPHA},

        // top
        { fBoxSize / 2, fBoxSize / 2, -fBoxSize / 2, fColorIntensity, 0.0f, 0.0f, BOX_ALPHA},
        { -fBoxSize / 2, fBoxSize / 2, -fBoxSize / 2, fColorIntensity, 0.0f, 0.0f, BOX_ALPHA},
        { -fBoxSize / 2, fBoxSize / 2, fBoxSize / 2, fColorIntensity, 0.0f, 0.0f, BOX_ALPHA},
        { -fBoxSize / 2, fBoxSize / 2, fBoxSize / 2, fColorIntensity, 0.0f, 0.0f, BOX_ALPHA},
        { fBoxSize / 2, fBoxSize / 2, fBoxSize / 2, fColorIntensity, 0.0f, 0.0f, BOX_ALPHA},
        { fBoxSize / 2, fBoxSize / 2, -fBoxSize / 2, fColorIntensity, 0.0f, 0.0f, BOX_ALPHA},
    };

    m_deviceBoxBuffer.create();
    m_deviceBoxBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);

    m_deviceBoxBuffer.bind();
#if !defined(Q_PROCESSOR_ARM)
    if((unsigned int)m_deviceBoxBuffer.size() < sizeof(boxData)){
        m_deviceBoxBuffer.allocate(sizeof(boxData));
    }
    auto pDeviceBoxVertices = m_deviceBoxBuffer.map(QOpenGLBuffer::WriteOnly);
    memcpy(pDeviceBoxVertices, &boxData[0], sizeof(boxData));
    m_deviceBoxBuffer.unmap();
#else
    m_deviceBoxBuffer.allocate(&boxData[0], sizeof(boxData));
#endif
    m_deviceBoxBuffer.release();
}

QMatrix4x4 CIMUDataViewerWidget::GetRotateMatrix()
{
    QMatrix4x4 rotate;
    rotate.rotate(-90.0, QVector3D(1.0f, 0.0f, 0.0f));
    rotate.rotate(m_quaternion);
    return rotate;
}

void CIMUDataViewerWidget::drawAxis()
{
    QMatrix4x4 model;
    model.scale(width() < height() ? width() / 3: height() / 3);

    quintptr offset = 0;
    int vertexLocation = m_program.attributeLocation("a_position");
    int colorLocation = m_program.attributeLocation("a_color");
    m_program.enableAttributeArray(vertexLocation);
    m_program.enableAttributeArray(colorLocation);

    m_program.setUniformValue("mvp_matrix", m_orthoMartix * GetRotateMatrix() * model);
    m_xAxisBuffer.bind();
    m_program.setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(CVertexData));
    offset = sizeof(float) * 3;
    m_program.setAttributeBuffer(colorLocation, GL_FLOAT, offset, 4, sizeof(CVertexData));
    m_xAxisBuffer.release();
    glDrawArrays(GL_TRIANGLES ,0, 54);

    offset = 0;
    QMatrix4x4 rotateToYAxis;
    rotateToYAxis.rotate(90.0f, 0.0f, 0.0f, 1.0f);
    m_program.setUniformValue("mvp_matrix", m_orthoMartix * GetRotateMatrix() * rotateToYAxis * model);
    m_yAxisBuffer.bind();
    m_program.setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(CVertexData));
    offset = sizeof(float) * 3;
    m_program.setAttributeBuffer(colorLocation, GL_FLOAT, offset, 4, sizeof(CVertexData));
    m_yAxisBuffer.release();
    glDrawArrays(GL_TRIANGLES ,0, 54);

    offset = 0;
    QMatrix4x4 rotateToZAxis;
    rotateToZAxis.rotate(-90.0f, 0.0f, 1.0f, 0.0f);

    m_program.setUniformValue("mvp_matrix", m_orthoMartix * GetRotateMatrix() * rotateToZAxis * model);
    m_zAxisBuffer.bind();
    m_program.setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(CVertexData));
    offset = sizeof(float) * 3;
    m_program.setAttributeBuffer(colorLocation, GL_FLOAT, offset, 4, sizeof(CVertexData));
    m_zAxisBuffer.release();
    glDrawArrays(GL_TRIANGLES ,0, 54);
}

void CIMUDataViewerWidget::drawDeviceBox()
{
    QMatrix4x4 model;
    model.scale(width() < height() ? width() / 3: height() / 3);
    model.scale(3.75f, 1.25f, 1.0f);
    quintptr offset = 0;
    int vertexLocation = m_program.attributeLocation("a_position");
    int colorLocation = m_program.attributeLocation("a_color");
    m_program.enableAttributeArray(vertexLocation);
    m_program.enableAttributeArray(colorLocation);

    m_program.setUniformValue("mvp_matrix", m_orthoMartix * GetRotateMatrix() * model);
    m_deviceBoxBuffer.bind();
    m_program.setAttributeBuffer(vertexLocation, GL_FLOAT, offset, 3, sizeof(CVertexData));
    offset = sizeof(float) * 3;
    m_program.setAttributeBuffer(colorLocation, GL_FLOAT, offset, 4, sizeof(CVertexData));
    m_deviceBoxBuffer.release();
    glDrawArrays(GL_TRIANGLES ,0, 36);
}

void CIMUDataViewerWidget::updateData(IMUData *pIMUData, CIMUModel::TYPE imuType)
{
    const float fGravity = 9.8f;
    const float degreeToArc = M_PI / 180.0;

    static QTime lastTime = QTime::currentTime();
    QTime currentTime = QTime::currentTime();
    float dt =(currentTime.msecsSinceStartOfDay() - lastTime.msecsSinceStartOfDay())/1000.0;
    lastTime = currentTime;

    switch (imuType){
        case CIMUModel::IMU_6_AXIS:
        {
            m_pIMUCalibration.DoCalib(*pIMUData);
            m_pIMUfilter->update(pIMUData->_accelX * fGravity, pIMUData->_accelY * fGravity, pIMUData->_accelZ * fGravity,
                                     pIMUData->_gyroScopeX * degreeToArc, pIMUData->_gyroScopeY * degreeToArc, pIMUData->_gyroScopeZ * degreeToArc,
                                     dt);
            double q0, q1, q2, q3;
            m_pIMUfilter->getOrientation(q0, q1, q2, q3);
            m_quaternion = QQuaternion(q0, q1, q2, q3);
            break;
        }
        case CIMUModel::IMU_9_AXIS:
            m_quaternion = QQuaternion(pIMUData->_quaternion[0],
                                       pIMUData->_quaternion[1],
                                       pIMUData->_quaternion[2],
                                       pIMUData->_quaternion[3]);
            break;
        default: return;
    }
#if defined(QT_DEBUG)
    QVector3D eulerAngles = m_quaternion.toEulerAngles();
    printf("Roll[%f], Pitch[%f], Yaw[%f]\n", eulerAngles.x(), eulerAngles.y(), eulerAngles.z());
#endif


    update();
}
