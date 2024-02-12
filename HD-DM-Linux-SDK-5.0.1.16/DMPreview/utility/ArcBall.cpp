#include "ArcBall.h"
#include <math.h>

CArcBall::CArcBall()
{
	m_fWidth = 1;
	m_fHeight = 1;
	m_iWidthCanvas = 1;
	m_iHeightCanvas = 1;
}


CArcBall::~CArcBall()
{
}

void CArcBall::Resize(float NewWidth, float NewHeight)
{
	//Set adjustment factor for width/height
	m_iWidthCanvas = (int)NewWidth;
	m_iHeightCanvas = (int)NewHeight;
	m_fWidth = 1.0f / ((NewWidth - 1.0f) * 0.5f);
	m_fHeight = 1.0f / ((NewHeight - 1.0f) * 0.5f);
}

void CArcBall::Reset()
{
	m_mLastTranf.Reset();
	m_mThisTranf.Reset();
}

QVector3D CArcBall::mapToSphere(QVector2D point)
{
    QVector3D vector;
    QVector2D tempPoint((float)point.x(), (float)point.y());
	//glm::vec2 tempPoint = glm::vec2(4.f,5.f);

	//Adjust point coords and scale down to range of [-1 ... 1]
    tempPoint.setX((tempPoint.x() * m_fWidth) - 1.0f);
    tempPoint.setY(1.0f - (tempPoint.y() * m_fHeight));

	//Compute square of the length of the vector from this point to the center
	//float length = (tempPoint.x * tempPoint.x) + (tempPoint.y * tempPoint.y);
    float length = QVector2D::dotProduct(tempPoint, tempPoint);
	//tempPoint

	//If the point is mapped outside the sphere... (length > radius squared)
	if (length > 1.0f)
	{
		//Compute a normalizing factor (radius / sqrt(length))
        float norm = (float)(1.0 / sqrt(length));

		//Return the "normalized" vector, a point on the sphere
        vector.setX(tempPoint.x() * norm);
        vector.setY(tempPoint.y() * norm);
        vector.setZ(0.0f);
	}
	//Else it's inside
	else
	{
		//Return a vector to a point mapped inside the sphere sqrt(radius squared - length)
        vector.setX(tempPoint.x());
        vector.setY(tempPoint.y());
        vector.setZ(sqrt(1.0f - length));
	}
	return vector;
}

void CArcBall::OnMouseDown(QVector2D point)
{
	m_mLastTranf = m_mThisTranf;
	m_vClickVector = mapToSphere(point);
	m_MouseStart = point;
}

void CArcBall::OnMouseMove(QVector2D point, MOUSE_OP action)
{
	m_vDragVector = mapToSphere(point);
	//glm::vec3 perp = glm::cross(m_vClickVector, m_vDragVector);
    QVector3D perp = QVector3D::crossProduct(m_vDragVector, m_vClickVector);
    QVector4D vNewRot;
    if (perp.length() > m_fEpsilon)
	{
        vNewRot.setX(perp.x());
        vNewRot.setY(perp.y());
        vNewRot.setZ(perp.z());
        vNewRot.setW(QVector3D::dotProduct(m_vClickVector, m_vDragVector));
	}
	//vNewRot is the quaternion equivalent to rotation
/*	if (action == TRANSLATE)
	{
		float x = (float)(point.x - m_MouseStart.x) / (float)m_iWidthCanvas;
		float y = (float)(point.y - m_MouseStart.y) / (float)m_iHeightCanvas;
		float z = 0.0f;
		glm::vec4 vNewRot2(0);
		m_mThisTranf.SetPan(glm::vec3(x, y, z));
		m_mThisTranf.SetScale(1.0f);
		m_mThisTranf.SetRotation(vNewRot2);
		m_mThisTranf.SetMatrix(m_mThisTranf.GetMatrix() * m_mLastTranf.GetMatrix());
	}
	else*/ 
		if (action == SCALE)
	{
        double a = m_MouseStart.x() * m_MouseStart.x() + m_MouseStart.y() * m_MouseStart.y();
        double b = point.x() * point.x() + point.y() * point.y();
        double len = sqrt(a) / sqrt(b);
        QVector4D vNewRot2;
        QVector3D vNewPan;
        m_mThisTranf.SetPan(vNewPan);
		m_mThisTranf.SetScale((float)len);
		m_mThisTranf.SetRotation(vNewRot2);
		m_mThisTranf.SetMatrix(m_mThisTranf.GetMatrix() * m_mLastTranf.GetMatrix());// Accumulate Last Rotation Into This One
	}
	else if (action == ROTATE)
	{
        QVector3D vNewPan;
        m_mThisTranf.SetPan(vNewPan);
		m_mThisTranf.SetScale(1.0f);
		m_mThisTranf.SetRotation(vNewRot);
		m_mThisTranf.SetMatrix(m_mThisTranf.GetMatrix() * m_mLastTranf.GetMatrix());
	}
}

void CArcBall::OnMouseUp(QVector2D point)
{

}
