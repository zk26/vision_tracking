#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include <Eigen/Dense>

class KalmanFilter {
public:
    KalmanFilter(double dt) : dt_(dt), is_initialized_(false) {
        // 状态维度：2 (x, y)
        x_ = Eigen::Vector2d::Zero();
        P_ = Eigen::Matrix2d::Identity() * 1.0;
        F_ = Eigen::Matrix2d::Identity();
        Q_ = Eigen::Matrix2d::Identity() * 0.01; // 过程噪声
        R_ = Eigen::Matrix2d::Identity() * 0.1;  // 测量噪声
        H_ = Eigen::Matrix2d::Identity();
    }

    void update(const Eigen::Vector2d& z) {
        if (!is_initialized_) {
            x_ = z;
            is_initialized_ = true;
            return;
        }
        // 预测
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;

        // 更新
        Eigen::Vector2d y = z - H_ * x_;
        Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;
        Eigen::Matrix2d K = P_ * H_.transpose() * S.inverse();

        x_ = x_ + K * y;
        P_ = (Eigen::Matrix2d::Identity() - K * H_) * P_;
    }

    Eigen::Vector2d getState() const {
        return x_;
    }

private:
    double dt_;
    bool is_initialized_;
    Eigen::Vector2d x_;
    Eigen::Matrix2d P_, F_, Q_, R_, H_;
};

#endif // KALMAN_FILTER_HPP_
