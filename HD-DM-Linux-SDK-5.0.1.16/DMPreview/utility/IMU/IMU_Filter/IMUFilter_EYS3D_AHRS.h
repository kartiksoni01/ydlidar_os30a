//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

#include "IMUFilter.h"
#include <cmath>

class IMUFilter_EYS3D_AHRS : public IMUFilter{
public:

    IMUFilter_EYS3D_AHRS();
    virtual ~IMUFilter_EYS3D_AHRS(){}

    virtual void getOrientation(double& q0, double& q1, double& q2, double& q3)
    {
        q0 = this->q0;
        q1 = this->q1;
        q2 = this->q2;
        q3 = this->q3;

        // perform precise normalization of the output, using 1/sqrt()
        // instead of the fast invSqrt() approximation. Without this,
        // TF2 complains that the quaternion is not normalized.
        double recipNorm = 1 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }

    virtual void setOrientation(double q0, double q1, double q2, double q3)
    {
        this->q0 = q0;
        this->q1 = q1;
        this->q2 = q2;
        this->q3 = q3;
    }

    virtual void update(double ax, double ay, double az,
                        double gx, double gy, double gz,
                        double mx, double my, double mz,
                        double dt);
    virtual void update(double ax, double ay, double az,
                        double gx, double gy, double gz,
                        double dt);

private:

    volatile double beta;	// algorithm gain
    volatile double q0, q1, q2, q3;
    double samplingFreq;
};


#endif
//=====================================================================================================
// End of file
//=====================================================================================================
