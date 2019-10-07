#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;  // y = z-Hx'  (VectorXd)
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si; // K = P'Ht-Si  (MatrixXd) 

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}



VectorXd Cart2Polar(const VectorXd &x_state){
  // convert radar (x, y, vx, vy) to (rho, phi, rho_dot)
  double px, py, vx, vy;
  px = x_state[0];
  py = x_state[1];
  vx = x_state[2];
  vy = x_state[3];

  double rho, phi, rho_dot;
  rho = sqrt(px*px + py*py);
  if (rho < 0.000001)
  {
    rho = 0.000001;
  }  
  phi = atan2(py,px);
  rho_dot = (px * vx + py * vy)/ rho;

  VectorXd z_pred = VectorXd(3);
  z_pred << rho, phi, rho_dot;

  return z_pred;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * update the state by using Extended Kalman Filter equations
   */
  
  VectorXd z_pred = Cart2Polar(x_);  
  VectorXd y = z - z_pred;  // y = z-Hx'  (VectorXd)
  // make anlge between -pi and pi
  if (y(1) > M_PI)
  {
    y(1) = y(1) - 2*M_PI;
  }
  else if (y(1) < -M_PI)
  {
    y(1) = y(1) + 2*M_PI;
  }

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si; // K = P'Ht-Si  (MatrixXd) 

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
