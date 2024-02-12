#include "ArcBallMatrix.h"

CArcBallMatrix::CArcBallMatrix()
{
    m_Matrix.setToIdentity();
	m_fScale = 1.f;
}

CArcBallMatrix::~CArcBallMatrix()
{
	Reset();
}

void CArcBallMatrix::SetScale(float fScale)
{
	m_fScale = fScale;
}

void CArcBallMatrix::SetPan(QVector3D vPan)
{
	m_vPan = vPan;
}

void CArcBallMatrix::SetRotation(const QVector4D & vquat)
{
	float n, s;
	float xs, ys, zs;
	float wx, wy, wz;
	float xx, xy, xz;
	float yy, yz, zz;

	//n = (vquat.x * vquat.x) + (vquat.y * vquat.y) + (vquat.z * vquat.z) + (vquat.w * vquat.w);
    n = QVector4D::dotProduct(vquat, vquat);
	s = (n > 0.0f) ? 2.0f / n : 0.0f;

    xs = vquat.x() * s;
    ys = vquat.y() * s;
    zs = vquat.z() * s;
    wx = vquat.w() * xs;
    wy = vquat.w() * ys;
    wz = vquat.w() * zs;
    xx = vquat.x() * xs;
    xy = vquat.x() * ys;
    xz = vquat.x() * zs;
    yy = vquat.y() * ys;
    yz = vquat.y() * zs;
    zz = vquat.z() * zs;

    // rotate
    m_Matrix.setColumn(0, {1.0f - (yy + zz), xy - wz, xz + wy, 0.0f});
    m_Matrix.setColumn(1, {xy + wz, 1.0f - (xx + zz), yz - wx, 0.0f});
    m_Matrix.setColumn(2, {xz - wy, yz + wx, 1.0f - (xx + yy), 0.0f});
    m_Matrix.setColumn(3, {0.0f, 0.0f, 0.0f, 1.0f});
	
    //scale
    m_Matrix.scale(m_fScale);

    m_Matrix.translate(m_vPan.x(), m_vPan.y());

}

void CArcBallMatrix::Reset()
{
    m_Matrix.setToIdentity();
	m_fScale = 1.f;
    m_vPan.setX(0.0f);
    m_vPan.setY(0.0f);
    m_vPan.setZ(0.0f);
}
