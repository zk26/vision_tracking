#ifndef VISION_TRACKING_KALMAN_FILTER_HPP
#define VISION_TRACKING_KALMAN_FILTER_HPP

#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter(double dt, double process_noise_cov = 0.01, double measurement_noise_cov = 0.1)
        : dt_(dt), initialized_(false) {
        
        // 状态向量维度 (x, y, vx, vy)
        int n = 4;
        
        // 状态转移矩阵
        F_ = Eigen::MatrixXd::Identity(n, n);
        F_(0, 2) = dt;
        F_(1, 3) = dt;
        
        // 测量矩阵 (只测量位置)
        H_ = Eigen::MatrixXd::Zero(2, n);
        H_(0, 0) = 1;
        H_(1, 1) = 1;
        
        // 过程噪声协方差矩阵
        Q_ = Eigen::MatrixXd::Zero(n, n);
        Q_(0, 0) = Q_(1, 1) = pow(dt, 4.0) / 4.0;
        Q_(0, 2) = Q_(2, 0) = Q_(1, 3) = Q_(3, 1) = pow(dt, 3.0) / 2.0;
        Q_(2, 2) = Q_(3, 3) = pow(dt, 2.0);
        Q_ *= process_noise_cov;
        
        // 测量噪声协方差矩阵
        R_ = Eigen::MatrixXd::Identity(2, 2) * measurement_noise_cov;
        
        // 状态协方差矩阵
        P_ = Eigen::MatrixXd::Identity(n, n);
        
        // 单位矩阵
        I_ = Eigen::MatrixXd::Identity(n, n);
    }
    
    void initialize(const Eigen::Vector2d& z0) {
        x_ = Eigen::VectorXd::Zero(4);
        x_(0) = z0(0);
        x_(1) = z0(1);
        initialized_ = true;
    }
    
    void update(const Eigen::Vector2d& z) {
        if (!initialized_) {
            initialize(z);
            return;
        }
        
        // 预测步骤
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;
        
        // 更新步骤
        Eigen::VectorXd z_pred = H_ * x_;
        Eigen::VectorXd y = Eigen::VectorXd(2);
        y(0) = z(0) - z_pred(0);
        y(1) = z(1) - z_pred(1);
        
        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
        
        // 更新状态和协方差
        x_ = x_ + K * y;
        P_ = (I_ - K * H_) * P_;
    }
    
    Eigen::Vector2d getState() const {
        return Eigen::Vector2d(x_(0), x_(1));
    }
    
private:
    double dt_;
    bool initialized_;
    Eigen::VectorXd x_;      // 状态向量
    Eigen::MatrixXd F_;      // 状态转移矩阵
    Eigen::MatrixXd H_;      // 测量矩阵
    Eigen::MatrixXd Q_;      // 过程噪声协方差
    Eigen::MatrixXd R_;      // 测量噪声协方差
    Eigen::MatrixXd P_;      // 状态协方差
    Eigen::MatrixXd I_;      // 单位矩阵
};

#endif // VISION_TRACKING_KALMAN_FILTER_HPP