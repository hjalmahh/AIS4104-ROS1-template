#include <Eigen/Dense>
#include <string>

#pragma once

class KalmanFilter {
public:
    KalmanFilter(
        double dt,
        const Eigen::MatrixXd& A, //system dynamics
        const Eigen::MatrixXd& C, //output matrix
        const Eigen::MatrixXd& Q,//Process noise covariance
        const Eigen::MatrixXd& R,//Measurment noise covariance
        const Eigen::MatrixXd& P//Estimate erro covariance
    );

    KalmanFilter();

    void init();

    void init(double t0, const Eigen::VectorXd& x0);




    //fra chat
    // New: Prediction step
    void predict(double dt, const Eigen::MatrixXd& A);

    // Measurement update step
    void update(const Eigen::VectorXd& y);

    //----------------------------
    // void update(const Eigen::VectorXd& y);
    //void update(const Eigen::VectorXd& y,double dt,const Eigen::MatrixXd& input);

    Eigen::VectorXd state() const {return x_hat; };
    double time () const {return t;};

private:
    Eigen::MatrixXd input, output, proc_noise, meas_noise, est_error, K, est_error0;

    int m , n;

    double t0,  t;

    double dt;

    bool initialized;

    Eigen::MatrixXd I;

    Eigen::VectorXd x_hat, x_hat_new;

};