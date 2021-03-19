#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.5;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  
  // first measurement (if)
  is_initialized_ = false;

  // state dimensions
  n_x_ = 5;
  n_aug_ = 7; 
  lambda_ = 3 - n_aug_;
  Xsig_dim = 2*n_aug_ + 1;
  // stochastic matrices
  Xsig_pred_ = MatrixXd(n_x_,2*n_aug_+1);
  // Measurement noise covariance matrices initialization
  R_radar_ = MatrixXd(3,3);
  R_radar_ << std_radr_*std_radr_, 0, 0,
              0,std_radphi_*std_radphi_,0,
              0,0,std_radrd_*std_radrd_;

  R_laser_ = MatrixXd(2,2);
  R_laser_ <<std_laspx_*std_laspx_, 0,
              0, std_laspy_*std_laspy_;
  // Initialize weights vector
  weights_ = VectorXd(2*n_aug_ + 1);
  weights_(0) = lambda_/(lambda_ + n_aug_);
  for (int wt_i = 1; wt_i < 2*n_aug_ + 1; wt_i++)
  {
    weights_(wt_i) = 0.5/(lambda_ + n_aug_);
  }

  // Initialize previous time
  previous_timestamp_ = 0;
  NIS_ = 0;

}

UKF::~UKF() {}

void UKF::NormalizeAngle(double& phi)
{
  while(phi > M_PI) phi -= 2.*M_PI;
  while(phi< -M_PI) phi += 2.*M_PI;

}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  if(!is_initialized_)
  {
    is_initialized_ = true;
    previous_timestamp_ = meas_package.timestamp_;
    double first_element  = meas_package.raw_measurements_(0);
    double second_element = meas_package.raw_measurements_(1);

    if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
     
      // coordinates conversion from polar to cartesian coords

      double px = first_element*cos(second_element);
      double py = first_element*sin(second_element);
      double velocity = meas_package.raw_measurements_(2);  
      x_[0] = px;
      x_[1] = py;
      x_[2] = velocity;
      R_ = R_radar_;
      P_ << std_radr_*std_radr_, 0, 0, 0, 0,
            0, std_radr_*std_radr_, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
      
    } // RADAR condition
  
    else if(meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      x_[0] = meas_package.raw_measurements_(0);
      x_[1] = meas_package.raw_measurements_(1);
      if (fabs(x_[0]) < 0.001 && fabs(x_[1]) < 0.001) 
      {
        x_[0] = 0.001;
        x_[1] = 0.001;
      }
      R_ = R_laser_;
      P_ << pow(std_laspx_,2), 0, 0, 0, 0,
            0, pow(std_laspy_,2), 0, 0, 0,
            0, 0, 2, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;

      
    }  // LIDAR condition
    return;
    } // is not initialized condition

    double dt = (meas_package.timestamp_ - previous_timestamp_)/1000000.0;
    previous_timestamp_ = meas_package.timestamp_;
    Prediction(dt);
    if (meas_package.sensor_type_ == MeasurementPackage::LASER)
      UpdateLidar(meas_package);
    else if(meas_package.sensor_type_ = MeasurementPackage::RADAR)
      UpdateRadar(meas_package);

 } // for the function end.




void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  VectorXd x_aug_ = VectorXd(n_aug_);
  x_aug_.head(5) = x_;
  x_aug_(5) = 0.0;
  x_aug_(6) = 0.0;

  // Including the variance of th noises in Covariance Matrix
  MatrixXd P_aug_ = MatrixXd(n_aug_, n_aug_);
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(5,5) = P_;
  P_aug_.bottomRightCorner(2,2) << pow(std_a_,2), 0,
                                    0, pow(std_yawdd_,2);

  // creating a matrix for sigma points
  
  MatrixXd Xsig = MatrixXd(n_aug_, Xsig_dim);
  Xsig.fill(0.0);
  Xsig.col(0) = x_aug_;
  MatrixXd A = P_aug_.llt().matrixL();

  for (int i = 0; i < n_aug_; i++)
  {
    Xsig.col(i+1) = x_aug_ + sqrt(lambda_ + n_aug_)* A.col(i);
    Xsig.col(i+1+n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_)* A.col(i);
  }

  // Plug-in the sigma points into the process model to get the predicted sigma points
  Xsig_pred_ = MatrixXd(n_x_, Xsig_dim);
  for (int i=0; i < Xsig_dim; i++)
  {
    double v = Xsig(2,i);
    double si = Xsig(3,i);
    double si_dot = Xsig(4,i);
    double nu_a = Xsig(5,i);
    double nu_yawdd = Xsig(6,i);
    VectorXd diff_term = VectorXd(n_x_);
    if (fabs(si_dot) > 0.001)
    {
      diff_term<< (v/si_dot)*(sin(si + si_dot*delta_t) - sin(si)),
                  (v/si_dot)*(-cos(si+ si_dot*delta_t) + cos(si)),
                  0,
                  si_dot* delta_t,
                  0;
    }
    else {
      diff_term << v*cos(si)*delta_t,
                   v*sin(si)*delta_t,
                   0,
                   si_dot*delta_t,
                   0; 
    }
    VectorXd noise_influence = VectorXd(n_x_);
    noise_influence << 0.5* pow(delta_t,2)*cos(si)*nu_a,
                        0.5* pow(delta_t,2)*sin(si)*nu_a,
                        delta_t*nu_a,
                        0.5*pow(delta_t,2)* nu_yawdd,
                        delta_t*nu_yawdd;

    Xsig_pred_.col(i) = Xsig.col(i).head(5) + diff_term + noise_influence;  
  } // end of the for loop with sigma points definition

x_.fill(0.0);
for (int i = 0; i < Xsig_dim; i++)
{
  x_ = x_ + weights_(i)*Xsig_pred_.col(i);
}
P_.fill(0.0);
for (int i=0; i< Xsig_dim; i++)
{
  VectorXd diff_term = Xsig_pred_.col(i) - x_;
  NormalizeAngle(diff_term(3));
  P_ = P_ + weights_(i)*diff_term*diff_term.transpose(); 
}
} // end of the function

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  MatrixXd H = MatrixXd(2,n_x_);
  H << 1, 0, 0, 0, 0,
       0, 1, 0, 0, 0;

  VectorXd z_pred = VectorXd(2,1);
  z_pred = H*x_;

  VectorXd y = VectorXd(2,1);
  y = meas_package.raw_measurements_ - z_pred;

  MatrixXd S = MatrixXd(2,2);
  S = H*P_*H.transpose() + R_laser_;

  MatrixXd K_gain = MatrixXd(n_x_,2);
  K_gain = P_*H.transpose()*S.inverse();

  VectorXd dx_ = K_gain * y;
  x_ = x_ + dx_;

  MatrixXd I = MatrixXd::Identity(5,5);
  P_ = (I-K_gain * H) *P_;

}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  MatrixXd Zsig = MatrixXd(3,Xsig_dim);
  for (int i= 0; i< Xsig_dim; i++)
  {
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double si = Xsig_pred_(3,i);
    double dist = sqrt(px*px + py*py);
    Zsig.col(i) << dist,
                   atan2(py,px),
                   (px*cos(si)*v + py*sin(si)*v)/dist;         
  }
  VectorXd z_pred = VectorXd(3);
  z_pred.fill(0.0);
  for(int j=0; j < Xsig_dim; j++)
  {
    z_pred = z_pred + weights_(j)*Zsig.col(j);
  }
  MatrixXd S = MatrixXd(3,3);
  S.fill(0.0);
  for (int i =0; i < Xsig_dim; i++)
  {
    VectorXd diff_term = Zsig.col(i) - z_pred;
    NormalizeAngle(diff_term(1));
    S = S + weights_(i)*diff_term*diff_term.transpose();
  }
  S(0,0) += pow(std_radr_,2);
  S(1,1) += pow(std_radphi_,2);
  S(2,2) += pow(std_radrd_,2);

  MatrixXd T_ = MatrixXd(n_x_, 3);
  T_.fill(0.0);
  for (size_t i = 0; i < Xsig_dim; i++)
  {
    VectorXd diff_X = Xsig_pred_.col(i) - x_;
    NormalizeAngle(diff_X(3));
    VectorXd diff_Z = Zsig.col(i) - z_pred;
    NormalizeAngle(diff_Z(1));
    T_ = T_ + weights_(i)*diff_X*diff_Z.transpose(); 
  }
  MatrixXd K_gain = MatrixXd(n_x_,3);
  K_gain = T_*S.inverse();
  VectorXd dz_ = (meas_package.raw_measurements_ - z_pred);
  NormalizeAngle(dz_(1));
  x_ = x_ + K_gain * dz_;
  P_ = P_ - K_gain*S*K_gain.transpose();
}