#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    // Set the initial error to 0
    this->p_error = 0.0;
    this->i_error = 0.0;
    this->d_error = 0.0;
    // Set multiplier (tau) for Proportional, Integral and Derivative errors
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void PID::UpdateError(double cte) {
    // Calculate the d_error by taking difference of current cte and previous cte (i.e. p_error)
    this->d_error = cte - this->p_error;
    // Set p_error after d_error so that we can use its previous value above
    this->p_error = cte;
    // Set i_error
    this->i_error = this->i_error + cte;
}

double PID::TotalError() {
    // Calculate and return the total error as per PID formulae
    // -tau_p * CTE - tau_d * diff_CTE - tau_i * int_CTE
    return (-1 * this->Kp * this->p_error) - this->Kd * this->d_error - this->Ki * this->i_error;
}
