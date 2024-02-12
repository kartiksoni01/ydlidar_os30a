#pragma once

#include <QMatrix4x4>
#include <QVector>
class CArcBallMatrix
{
private:
    QMatrix4x4 m_Matrix;
	float m_fScale;
    QVector3D m_vPan;
public:
	CArcBallMatrix();
	~CArcBallMatrix();
    const QMatrix4x4 & GetMatrix(){ return m_Matrix; }	//a reference
    void SetMatrix(const QMatrix4x4 & matrix){ m_Matrix = matrix; }
	void SetScale(float fScale);
    void SetPan(QVector3D vPan);
    void SetRotation(const QVector4D & vquat);
	void Reset();
};

