#pragma once
#include "ArcBallMatrix.h"
#include "OGLBasic.h"
#include <QMatrix4x4>
#include <QVector2D>
class CArcBall
{
private:
	const float m_fEpsilon = 1.0e-5f;
    const QMatrix4x4 m_mIdentity;
    QVector3D m_vClickVector;
    QVector3D m_vDragVector;
	float m_fWidth;
	float m_fHeight;
	int m_iWidthCanvas;
	int m_iHeightCanvas;
    QVector3D mapToSphere(QVector2D point);
protected:
	CArcBallMatrix m_mLastTranf;
	CArcBallMatrix m_mThisTranf;
    QVector2D m_MouseStart;
public:
	CArcBall();
	~CArcBall();
	void Resize(float NewWidth, float NewHeight);
	void Reset();
    const QMatrix4x4 & GetTransformation(){ return m_mThisTranf.GetMatrix(); }
    void OnMouseDown(QVector2D point);
    void OnMouseMove(QVector2D point, MOUSE_OP action);
    void OnMouseUp(QVector2D point);
};

