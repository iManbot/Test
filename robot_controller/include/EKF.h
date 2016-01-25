#ifndef EKF_H
#define EKF_H
#include <filter/extendedkalmanfilter.h>

#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>

#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>

#include <iostream>

/* The purpose of this program is to construct a kalman filter for the problem
  of localisation of a mobile robot equipped with an ultrasonic sensor.
  In this case the orientation is known, which simplifies the model considerably:
  The system model will become linear.
  The ultrasonic measures the distance to the wall (it can be switched off:
  see mobile_robot_wall_cts.h)

  The necessary SYSTEM MODEL is:

  x_k      = x_{k-1} + v_{k-1} * cos(theta) * delta_t
  y_k      = y_{k-1} + v_{k-1} * sin(theta) * delta_t

  The used MEASUREMENT MODEL:
  measuring the (perpendicular) distance z to the wall y = ax + b

  set WALL_CT = 1/sqrt(pow(a,2) + 1)
  z = WALL_CT * a * x - WALL_CT * y + WALL_CT * b + GAUSSIAN_NOISE
  or Z = H * X_k + J * U_k

  where

  H = [ WALL_CT * a       - WALL_CT      0 ]
  and GAUSSIAN_NOISE = N((WALL_CT * b), SIGMA_MEAS_NOISE)

*/

    
namespace ift
{
    using namespace MatrixWrapper;
    using namespace BFL;
    using namespace std;
class EKF
{

public:
    EKF(unsigned int state_dim, unsigned int control_dim): _A(Matrix(state_dim, state_dim)),
        _B(Matrix(state_dim, control_dim)), _sysNoise_Mu(ColumnVector(state_dim)),
        _sysNoise_Cov(state_dim),
        _system_Uncertainty(Gaussian(_sysNoise_Mu, _sysNoise_Cov)),
        _AB(2),
        _sys_pdf(LinearAnalyticConditionalGaussian(_AB, _system_Uncertainty)),
        _sys_model(LinearAnalyticSystemModelGaussianUncertainty(&_sys_pdf)),
        _H(control_dim, state_dim),
        _measNoise_Mu(control_dim),
        _measNoise_Cov(control_dim),
        _measurement_Uncertainty(_measNoise_Mu, _measNoise_Cov),
        _meas_pdf(_H, _measurement_Uncertainty),
        _meas_model(&_meas_pdf),
        _prior_Mu(state_dim),
        _prior_Cov(state_dim),
        _prior(_prior_Mu, _prior_Cov),
        _filter(&_prior) {
        initialize();
    }

    Matrix _A;
    Matrix _B;
    vector<Matrix> _AB;

    // create gaussian
    ColumnVector _sysNoise_Mu;
    SymmetricMatrix _sysNoise_Cov;
    Gaussian _system_Uncertainty;

    // create the model
    LinearAnalyticConditionalGaussian _sys_pdf;
    LinearAnalyticSystemModelGaussianUncertainty _sys_model;

    // create matrix H for linear measurement model
    Matrix _H;

    // Construct the measurement noise (a scalar in this case)
    ColumnVector _measNoise_Mu;
    SymmetricMatrix _measNoise_Cov;
    Gaussian _measurement_Uncertainty;

    // create the model
    LinearAnalyticConditionalGaussian _meas_pdf;
    LinearAnalyticMeasurementModelGaussianUncertainty _meas_model;


    // Continuous Gaussian prior
    ColumnVector _prior_Mu;
    SymmetricMatrix _prior_Cov;
    Gaussian _prior;

    // filter
    ExtendedKalmanFilter _filter;
    
    // prepare matrix and distributions
    void initialize() {
        _A = 0.0;
        _A(1, 1) = 1.0;
        _A(2, 2) = 1.0;
        _A(3, 3) = 1.0;

        _B = 0.0;
        _B(1, 1) = 0.2;
        _B(2, 2) = 0.2;
        _B(3, 3) = 0.2;

        _AB[0] = _A;
        _AB[1] = _B;

        // create gaussian
        ColumnVector sysNoise_Mu(3);
        sysNoise_Mu = 0.0;
        SymmetricMatrix sysNoise_Cov(3);
        sysNoise_Cov = 0.0;
        sysNoise_Cov(1, 1) = pow(0.1,2);
        sysNoise_Cov(2, 2) = pow(0.1,2);
        sysNoise_Cov(3, 3) = pow(0.1,2);
	Gaussian system_Uncertainty(sysNoise_Mu, sysNoise_Cov);

        // create the model
        _sys_pdf = LinearAnalyticConditionalGaussian(_AB, system_Uncertainty);

        // create matrix H for linear measurement model
        _H = 0.0;
        _H(1, 1) = 1.0;
        _H(2, 2) = 1.0;
        _H(3, 3) = 1.0;


        // Construct the measurement noise (a scalar in this case)
        ColumnVector measNoise_Mu(3);
        measNoise_Mu = 0.0;
        SymmetricMatrix measNoise_Cov(3);
        measNoise_Cov = 0.0;
        measNoise_Cov(1, 1) = pow(0.025,2);
        measNoise_Cov(2, 2) = pow(0.025,2);
        measNoise_Cov(3, 3) = pow(0.1,2);
        Gaussian measurement_Uncertainty(measNoise_Mu, measNoise_Cov);
	
        // create the model
        _meas_pdf = LinearAnalyticConditionalGaussian(_H, measurement_Uncertainty);
        _prior_Mu = 0.0;
        _prior_Cov = 0.0;
        _prior_Cov(1, 1) = pow(10,2);
        _prior_Cov(2, 2) = pow(10,2);
        _prior_Cov(3, 3) = pow(10,2);
        _prior = Gaussian(_prior_Mu, _prior_Cov);
    }

    void setA(float dt) {
        _A(1, 1) = dt;
        _A(2, 2) = dt;
        _A(3, 3) = dt;
        _sys_model.ASet(_A);
    }

    void setB(float dt) {
        _B(1, 1) = dt;
        _B(2, 2) = dt;
        _B(3, 3) = dt;
        _sys_model.BSet(_B);
    }
    void setAB() {
        _sys_model.ASet(_A);
        _sys_model.BSet(_B);
    }
    
    void update(ColumnVector input, ColumnVector measurement) {
        _filter.Update(&_sys_model, input, &_meas_model, measurement);
    }

    ColumnVector getExpected() {
        Pdf<ColumnVector> *posterior = _filter.PostGet();
	return posterior->ExpectedValueGet();
	
    }


}; // end of class EKF
}; // end of package ift


#endif // EKF_H
