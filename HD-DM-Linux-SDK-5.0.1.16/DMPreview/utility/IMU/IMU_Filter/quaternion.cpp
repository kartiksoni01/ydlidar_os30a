#include <cmath>
#include "quaternion.h"
using namespace std;
#define PI 3.14159265358979

/**
 * Compute quaternion product r = a * b, according to Equation (4) of
 * http://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf .
 * Args:
 *     a: 4-element vector, representing the quaternion [a[0], a[1], a[2], a[3]]
 *     b: 4-element vector, representing the quaternion [b[0], b[1], b[2], b[3]]
 *     r: the resulting quaternion, [r[0], r[1], r[2], r[3]]
*/
std::vector<double> quaternionProduct(const vector<double> &a, const vector<double> &b)
{
    vector<double> prod(4);
    prod[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    prod[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    prod[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    prod[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
    return prod;
}

vector<double> quaternionConjugate(const vector<double> &q)
{
    vector<double> conj = q;
    conj[1] *= -1;
    conj[2] *= -1;
    conj[3] *= -1;
    return conj;
}

EulerAngles quaternionToEulerAngles(const vector<double> &q)
{

    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q[0] * q[1] + q[2] * q[3]);
    double cosr_cosp = +1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
    angles.roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q[0] * q[2] - q[3] * q[1]);
    if (fabs(sinp) >= 1)
        angles.pitch = copysign(PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q[0] * q[3] + q[1] * q[2]);
    double cosy_cosp = +1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);
    angles.yaw = atan2(siny_cosp, cosy_cosp);

    angles.pitch *= 180.0/PI;
    angles.roll *= 180.0/PI;
    angles.yaw *= 180.0/PI;

    return angles;
}
