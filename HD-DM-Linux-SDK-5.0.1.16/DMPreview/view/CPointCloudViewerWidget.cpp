#include "CPointCloudViewerWidget.h"
#include "OpenGLShaderCore.h"
#include <QVector3D>
#include "CVideoDeviceController.h"
#include <QMouseEvent>
#include <QWheelEvent>
#include <math.h>
#if !defined(Q_PROCESSOR_ARM)
#include <GL/gl.h>
#include <GL/glext.h>
#endif
CPointCloudViewerWidget::CPointCloudViewerWidget(CVideoDeviceController *pVideoDeviceController,
                                                 QWidget *parent):
QOpenGLWidget(parent),
m_pVideoDeviceController(pVideoDeviceController),
m_verticesBuffer(QOpenGLBuffer::VertexBuffer),
m_colorsBuffer(QOpenGLBuffer::VertexBuffer),
m_fScrollZ(0.0f),
m_fMoveX(0.0f), m_fMoveY(0.0f),
m_nLastMouseX(0), m_nLastMouseY(0)
{
    m_rotateMatrix.setToIdentity();
    m_modelMatrix.setToIdentity();
    m_viewMatrix.setToIdentity();
    m_perspectiveMatrix.setToIdentity();
}

CPointCloudViewerWidget::~CPointCloudViewerWidget()
{
    makeCurrent();
    m_verticesBuffer.destroy();
    m_colorsBuffer.destroy();
    doneCurrent();
}

void CPointCloudViewerWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
    if(Qt::LeftButton == event->buttons()){
        m_arcBall.Reset();
        m_fScrollZ = 0.0f;
        m_fMoveY = 0.0f;
        m_fMoveX = 0.0f;
    }
}

void CPointCloudViewerWidget::mousePressEvent(QMouseEvent *event)
{
    if(Qt::LeftButton == event->buttons()){
        m_arcBall.OnMouseDown({(float)event->x(), (float)event->y()});
    }else if(Qt::MiddleButton == event->buttons()){
        m_nLastMouseX = event->x();
        m_nLastMouseY = event->y();
    }
}

void CPointCloudViewerWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if(Qt::LeftButton == event->buttons()){
        m_arcBall.OnMouseUp({(float)event->x(), (float)event->y()});
    }
}

void CPointCloudViewerWidget::mouseMoveEvent(QMouseEvent *event)
{
    if(Qt::LeftButton == event->buttons()){
        m_arcBall.OnMouseMove({(float)event->x(), (float)event->y()}, ROTATE);
    }else if(Qt::MiddleButton == event->buttons()){
        m_fMoveX += event->x() - m_nLastMouseX;
        m_nLastMouseX = event->x();

        m_fMoveY += event->y() - m_nLastMouseY;
        m_nLastMouseY = event->y();
    }
}

void CPointCloudViewerWidget::wheelEvent(QWheelEvent *event)
{
    m_fScrollZ += event->delta() / 10.0f;
}

void CPointCloudViewerWidget::initializeGL()
{
    initializeOpenGLFunctions();

    initShaders();

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    m_rotateMatrix.setToIdentity();
    m_rotateMatrix.rotate(180.0f, 0.0f, 1.0f, 0.0f);
    m_rotateMatrix.rotate(180.0f, 0.0f, 0.0f, 1.0f);

    m_verticesBuffer.create();
    m_verticesBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);

    m_colorsBuffer.create();
    m_colorsBuffer.setUsagePattern(QOpenGLBuffer::StaticDraw);

    PrepareCircleData();
}

void CPointCloudViewerWidget::PrepareCircleData()
{
    const float fRadio = 0.5f;
    const int nTotalCirclePoint = 500;
    const float fOffset = (2 * M_PI) / (nTotalCirclePoint - 1);
    const float nTotalVerticesValueCount = nTotalCirclePoint * 3;
    m_circleVertices.resize(nTotalVerticesValueCount * 3, 0.0f);
    m_circleColors.resize(nTotalVerticesValueCount * 3, 0);
    for (float r = 0.0f, i = 0; i < nTotalCirclePoint ; ++i, r += fOffset){
        m_circleVertices[i * 3] = cos(r) * fRadio;
        m_circleVertices[i * 3 + 1] = sin(r) * fRadio;
        m_circleVertices[i * 3 + 2] = 0.0f;
        m_circleColors[i * 3] = 163;
        m_circleColors[i * 3 + 1] = 163;
        m_circleColors[i * 3 + 2] = 234;

        m_circleVertices[i * 3 + nTotalVerticesValueCount] = 0;
        m_circleVertices[i * 3 + 1 + nTotalVerticesValueCount] = cos(r) * fRadio;
        m_circleVertices[i * 3 + 2 + nTotalVerticesValueCount] = sin(r) * fRadio;
        m_circleColors[i * 3 + nTotalVerticesValueCount] = 230;
        m_circleColors[i * 3 + 1 + nTotalVerticesValueCount] = 161;
        m_circleColors[i * 3 + 2 + nTotalVerticesValueCount] = 161;

        m_circleVertices[i * 3 + (nTotalVerticesValueCount * 2)] =  sin(r) * fRadio;
        m_circleVertices[i * 3 + 1 + (nTotalVerticesValueCount * 2)] = 0;
        m_circleVertices[i * 3 + 2 + (nTotalVerticesValueCount * 2)] = cos(r) * fRadio;
        m_circleColors[i * 3 + (nTotalVerticesValueCount * 2)] = 179;
        m_circleColors[i * 3 + 1 + (nTotalVerticesValueCount * 2)] = 244;
        m_circleColors[i * 3 + 2 + (nTotalVerticesValueCount * 2)] = 185;
    }
}

void CPointCloudViewerWidget::updateData(std::vector<float> &vertices, std::vector<unsigned char> &colors)
{
    m_mutex.lock();
    if(vertices.size() != m_vertices.size()){
        m_vertices.resize(vertices.size());
    }
    m_vertices.assign(vertices.begin(), vertices.end());
    m_mutex.unlock();

    m_mutex.lock();
    if(colors.size() != m_colors.size()){
        m_colors.resize(colors.size());
    }
    m_colors.assign(colors.begin(), colors.end());
    m_mutex.unlock();

    m_fpsCalculator.clock();

    parentWidget()->update();
    update();
}

void CPointCloudViewerWidget::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
    m_perspectiveMatrix.setToIdentity();
    double dblFOV = m_pVideoDeviceController ? m_pVideoDeviceController->GetVideoDeviceModel()->GetCameraFOV() :
                                               75.0f;
    m_perspectiveMatrix.perspective(dblFOV, (float)w / h, 0.01f, 16384.0f);

    m_orthoMartix.setToIdentity();
    int nZPlane = w < h ? w : h;
    m_orthoMartix.ortho( -w / 2, w / 2, -h / 2, h / 2, -nZPlane / 2, nZPlane / 2);
    m_arcBall.Resize(w, h);
    QOpenGLWidget::resizeGL(w, h);
}

void CPointCloudViewerWidget::paintGL()
{
    // Clear color and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    drawBackground();
    darwPointCloud();
    drawAxis();
}

void CPointCloudViewerWidget::drawBackground()
{
    static float backgroundVertices[] = {
                                          -1.0f, -1.0f, 0.0f,
                                          1.0f, -1.0f, 0.0f,
                                          -1.0f, 1.0f, 0.0f,

                                          -1.0f, 1.0f, 0.0f,
                                          1.0f, -1.0f, 0.0f,
                                          1.0f, 1.0f, 0.0f
                                        };

    static unsigned char backgroundColors[] = {
                                                157, 157, 237,
                                                157, 157, 237,
                                                0, 0, 0,

                                                0, 0, 0,
                                                157, 157, 237,
                                                0, 0, 0

                                               };

    m_program.setUniformValue("mvp_matrix",
                              QMatrix4x4());

    m_verticesBuffer.bind();
#if !defined(Q_PROCESSOR_ARM)
    if((unsigned int)m_verticesBuffer.size() < sizeof(backgroundVertices)){
        m_verticesBuffer.allocate(sizeof(backgroundVertices));
    }
    auto pVertices = m_verticesBuffer.map(QOpenGLBuffer::WriteOnly);
    memcpy(pVertices, &backgroundVertices[0], sizeof(backgroundVertices));
    m_verticesBuffer.unmap();
#else
    m_verticesBuffer.allocate(&backgroundVertices[0], sizeof(backgroundVertices));
#endif

    int vertexLocation = m_program.attributeLocation("a_position");
    m_program.enableAttributeArray(vertexLocation);
    m_program.setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);
    m_verticesBuffer.release();

    m_colorsBuffer.bind();
#if !defined(Q_PROCESSOR_ARM)
    if((unsigned int)m_colorsBuffer.size() < sizeof(backgroundColors)){
        m_colorsBuffer.allocate(sizeof(backgroundColors));
    }
    auto pColors = m_colorsBuffer.map(QOpenGLBuffer::WriteOnly);
    memcpy(pColors, &backgroundColors[0], sizeof(backgroundColors));
    m_colorsBuffer.unmap();
#else
    m_colorsBuffer.allocate(&backgroundColors[0], sizeof(backgroundColors));
#endif

    // Tell OpenGL programmable pipeline how to locate vertex texture coordinate data
    int colorLocation = m_program.attributeLocation("a_color");
    m_program.enableAttributeArray(colorLocation);
    m_program.setAttributeBuffer(colorLocation, GL_UNSIGNED_BYTE, 0, 3);
    m_colorsBuffer.release();

    m_program.setUniformValue("bSingleColor", false);

    glDisable(GL_DEPTH_TEST);
    glDrawArrays(GL_TRIANGLES ,0, 6);
    glEnable(GL_DEPTH_TEST);
}

void CPointCloudViewerWidget::drawAxis()
{
    QMatrix4x4 model;
    model.scale(width() < height() ? width() : height());
    m_program.setUniformValue("mvp_matrix",
                              m_orthoMartix * model * m_arcBall.GetTransformation());

    m_verticesBuffer.bind();
#if !defined(Q_PROCESSOR_ARM)
    if((unsigned int)m_verticesBuffer.size() < m_circleVertices.size() * sizeof(float)){
        m_verticesBuffer.allocate(m_circleVertices.size() * sizeof(float));
    }
    auto pVertices = m_verticesBuffer.map(QOpenGLBuffer::WriteOnly);
    memcpy(pVertices, &m_circleVertices[0], m_circleVertices.size() * sizeof(float));
    m_verticesBuffer.unmap();
#else
    m_verticesBuffer.allocate(&m_circleVertices[0], m_circleVertices.size() * sizeof(float));
#endif

    int vertexLocation = m_program.attributeLocation("a_position");
    m_program.enableAttributeArray(vertexLocation);
    m_program.setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);
    m_verticesBuffer.release();

    m_colorsBuffer.bind();
#if !defined(Q_PROCESSOR_ARM)
    if((unsigned int)m_colorsBuffer.size() < m_circleColors.size() * sizeof(unsigned char)){
        m_colorsBuffer.allocate(m_circleColors.size() * sizeof(unsigned char));
    }
    auto pColors = m_colorsBuffer.map(QOpenGLBuffer::WriteOnly);
    memcpy(pColors, &m_circleColors[0], m_circleColors.size() * sizeof(unsigned char));
    m_colorsBuffer.unmap();
#else
    m_colorsBuffer.allocate(&m_circleColors[0], m_circleColors.size() * sizeof(unsigned char));
#endif

    // Tell OpenGL programmable pipeline how to locate vertex texture coordinate data
    int colorLocation = m_program.attributeLocation("a_color");
    m_program.enableAttributeArray(colorLocation);
    m_program.setAttributeBuffer(colorLocation, GL_UNSIGNED_BYTE, 0, 3);
    m_colorsBuffer.release();

    m_program.setUniformValue("bSingleColor", false);

    glLineWidth(1);
    glDrawArrays(GL_LINE_STRIP , 0, m_circleVertices.size() / 9);
    glDrawArrays(GL_LINE_STRIP , m_circleVertices.size() / 9, m_circleVertices.size() / 9);
    glDrawArrays(GL_LINE_STRIP , m_circleVertices.size() * 2 / 9, m_circleVertices.size() / 9);
}

void CPointCloudViewerWidget::darwPointCloud()
{
    // Set modelview-projection matrix
    int nZNear, nZFar;
    m_pVideoDeviceController->GetPreviewOptions()->GetZRange(nZNear, nZFar);
    int nExpand = (nZFar - nZNear) * 0.2;
    nZNear -= nExpand;
    nZFar  += nExpand;
    m_modelMatrix.setToIdentity();
    m_modelMatrix.translate(m_fMoveX, -m_fMoveY, -m_fScrollZ);
    m_modelMatrix.translate(0,0, -(nZFar - nZNear) / 2);
    m_modelMatrix *= m_arcBall.GetTransformation();
    m_modelMatrix.translate(0,0, (nZFar - nZNear) / 2);
    m_modelMatrix *= m_rotateMatrix;
    m_program.setUniformValue("mvp_matrix",
                              m_perspectiveMatrix * m_viewMatrix * m_modelMatrix);

    m_mutex.lock();
    m_verticesBuffer.bind();
#if !defined(Q_PROCESSOR_ARM)
    if((unsigned int)m_verticesBuffer.size() < m_vertices.size() * sizeof(float)){
        m_verticesBuffer.allocate(m_vertices.size() * sizeof(float));
    }
    auto pVertices = m_verticesBuffer.map(QOpenGLBuffer::WriteOnly);
    memcpy(pVertices, &m_vertices[0], m_vertices.size() * sizeof(float));
    m_verticesBuffer.unmap();
#else
    m_verticesBuffer.allocate(&m_vertices[0], m_vertices.size() * sizeof(float));
#endif

    // Tell OpenGL programmable pipeline how to locate vertex position data
    int vertexLocation = m_program.attributeLocation("a_position");
    m_program.enableAttributeArray(vertexLocation);
    m_program.setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3);
    m_verticesBuffer.release();
    m_mutex.unlock();

    m_mutex.lock();
    m_colorsBuffer.bind();
#if !defined(Q_PROCESSOR_ARM)
    if((unsigned int)m_colorsBuffer.size() < m_colors.size() * sizeof(unsigned char)){
        m_colorsBuffer.allocate(m_colors.size() * sizeof(unsigned char));
    }
    auto pColors = m_colorsBuffer.map(QOpenGLBuffer::WriteOnly);
    memcpy(pColors, &m_colors[0], m_colors.size() * sizeof(unsigned char));
    m_colorsBuffer.unmap();
#else
    m_colorsBuffer.allocate(&m_colors[0], m_colors.size() * sizeof(unsigned char));
#endif

    // Tell OpenGL programmable pipeline how to locate vertex texture coordinate data
    int colorLocation = m_program.attributeLocation("a_color");
    m_program.enableAttributeArray(colorLocation);
    m_program.setAttributeBuffer(colorLocation, GL_UNSIGNED_BYTE, 0, 3);
    m_colorsBuffer.release();
    m_mutex.unlock();

    m_program.setUniformValue("bSingleColor",
                              PreviewOptions::POINT_CLOUDE_VIEWER_OUTPUT_SINGLE_COLOR == m_pVideoDeviceController->GetPreviewOptions()->GetPointCloudViewOutputFormat());
    #ifndef TI_EVM
    glPointSize(m_pVideoDeviceController->GetPreviewOptions()->GetPointCloudSize());
    #endif
    glDrawArrays(GL_POINTS ,0, m_vertices.size() / 3);
}

void CPointCloudViewerWidget::initShaders()
{
    // Compile vertex shader
    if (!m_program.addShaderFromSourceCode(QOpenGLShader::Vertex, shaderVert))
        close();

    // Compile fragment shader
    if (!m_program.addShaderFromSourceCode(QOpenGLShader::Fragment, shaderFragment))
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
