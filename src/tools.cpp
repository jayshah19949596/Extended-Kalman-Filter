#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
    // ===========================
    // Calculate the RMSE here.
    // ===========================
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if(estimations.size() != ground_truth.size() || estimations.size() == 0)
    {
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    // ============================
    // Accumulate squared residuals
    // ============================
    for(unsigned int i=0; i < estimations.size(); ++i)
    {
        VectorXd residual = estimations[i] - ground_truth[i];
        // cout << residual << endl;
        // cout << "=================" << endl;

        //coefficient-wise multiplication
        residual = residual.array()*residual.array();
        // cout << residual << endl;
        // cout << "=================" << endl;
        rmse += residual;
    }

    // ============================
    // Calculate the mean
    // ============================
    rmse = (1.0/estimations.size()) * rmse;

    // ===========================
    // Calculate the squared root
    // ===========================
    rmse = rmse.array().sqrt();

    // ===================
    // return the result
    // ===================
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
{
    // ============================
    // Calculate a Jacobian here.
    // ============================
    float px, py, vx, vy, c00, c12, c32;
    MatrixXd Jacobian(3, 4);

    px = x_state(0);
    py = x_state(1);
    vx = x_state(2);
    vy = x_state(3);

    c00 = px*px + py*py;
    c12 = sqrt(c00);
    c32 = (c00*c12);

    Jacobian << 0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0;

    if (fabs(c00) < 0.0001)
    {
        std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
        return Jacobian;
    }

    Jacobian(0, 0) = px / c12;
    Jacobian(0, 1) = py / c12;

    Jacobian(1, 0) = -py / c00;
    Jacobian(1, 1) = px / c00;

    Jacobian(2, 0) = py*(vx*py - vy*px) / c32;
    Jacobian(2, 1) = px*(vy*px - vx*py) / c32;
    Jacobian(2, 2) = px / c12;
    Jacobian(2, 3) = py / c12;

    return Jacobian;
}
