#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

// https://jp.mathworks.com/help/nav/ref/imufilter-system-object.html
class IMUPoseFilter {

public:
    struct Parameter {
        float Ts; // サンプリング周期 [s]

        float beta = 1e-12;  // ジャイロドリフトノイズ分散((rad/s)^2)
        float eta = 1e-5;    // ジャイロノイズ分散((rad/s)^2)
        float lambda = 1e-3; // 加速度センサノイズ分散((m/s^2)^2)

        float xi = 1e-2; // 線形加速度推定ノイズ分散((m/s^2)^2)

        float nu = 0.5; // 加速度のローパスフィルタ係数(0.0 < nu < 1.0)
        // int decimation_factor = 1; // サンプリングレートの減少係数
    };

    IMUPoseFilter(const Parameter params) : params_(params) { reset(); }

    void reset(Eigen::Quaternionf init_q = Eigen::Quaternionf::Identity()) {
        state_.resize(9);
        state_.setZero();

        // kappa_ = params_.decimation_factor * params_.Ts;
        kappa_ = params_.Ts;
        kappa_sq_ = kappa_ * kappa_;

        P_.resize(9, 9);
        P_.setZero();
        P_.diagonal() << Vector3f::Constant(1e-5), Vector3f::Constant(0), Vector3f::Constant(0);

        Q_.resize(9, 9);
        Q_.setZero();
        predict_error_convariance(); // init P_

        H_.resize(3, 9);
        H_.setZero();

        R_.resize(3, 3);
        R_ = (params_.lambda + params_.xi + kappa_sq_ * (params_.beta + params_.eta)) * Eigen::Matrix3f::Identity(3, 3);

        q_ = init_q;
        filtered_acc_ = Eigen::Vector3f::Zero();
        filtered_gyro_ = Eigen::Vector3f::Zero();
    }

    // センサーの更新
    // gyro: ジャイロセンサーの値 [rad/s]
    // accel: 加速度センサーの値 [m/s^2]
    void update(Eigen::Vector3f gyro, Eigen::Vector3f accel) {
        ////////// model prediction step //////////

        // ジャイロのバイアスの補正
        prev_filtered_gyro_ = filtered_gyro_;
        filtered_gyro_ = gyro - gyro_offset_;

        // 姿勢クォータニオンの予測
        q_ = q_ * rpy2q((0.5f * prev_filtered_gyro_ + 0.5f * filtered_gyro_) * params_.Ts);

        // 姿勢からの重力の予測
        Eigen::Vector3f g_vec(0, 0, GRAVITY);     // 地球の重力加速度 [m/s^2]
        Eigen::Vector3f g = q_.inverse() * g_vec; // 重力をセンサーフレームに変換

        // 加速度センサからの重力の予測
        // 観測した加速度(モーション成分+重力)から前回推定した加速度(モーション成分のみ)を引く
        Eigen::Vector3f g_accel = accel - filtered_acc_; // (センサーフレーム)

        // 重力の誤差予測
        Eigen::Vector3f z = g - g_accel;

        ////////// Kalman filter step //////////
        // gが推定値、zが観測値

        // 観測モデル
        // 姿勢誤差
        H_(0, 1) = g.z();
        H_(1, 0) = -g.z();
        H_(0, 2) = -g.y();
        H_(2, 0) = g.y();
        H_(1, 2) = g.x();
        H_(2, 1) = -g.x();

        // ジャイロバイアス
        H_(0, 4) = -kappa_ * g.z();
        H_(1, 3) = kappa_ * g.z();
        H_(0, 5) = kappa_ * g.y();
        H_(2, 3) = -kappa_ * g.y();
        H_(1, 5) = -kappa_ * g.x();
        H_(2, 4) = kappa_ * g.x();

        // 重力加速度誤差
        H_(0, 6) = 1;
        H_(1, 7) = 1;
        H_(2, 8) = 1;

        // Innovation covariance
        Eigen::Matrix3f S = R_ + H_ * P_ * H_.transpose();

        // Kalman gain
        Eigen::MatrixXf K = P_ * H_.transpose() * S.inverse();

        // Update error estimate covariance
        auto I = Eigen::MatrixXf::Identity(9, 9);
        P_ = (I - K * H_) * P_;

        // Update a posteriori error(x+)
        state_ = K * z;

        ////////// correct step //////////
        // 姿勢の補正
        q_ = q_ * rpy2q(state_.segment<3>(0));
        q_.normalize();

        // 加速度の補正
        filtered_acc_ = params_.nu * filtered_acc_ - state_.segment<3>(6);

        // ジャイロオフセットの更新
        gyro_offset_ -= state_.segment<3>(3);

        test_g1_ = g;
        test_g2_ = g_accel;
        test_goffset_ = gyro_offset_;
    }

    Eigen::Quaternionf get_quaternion() const { return q_; }
    Eigen::Vector3f get_filtered_acc() const { return filtered_acc_; }
    Eigen::Vector3f get_filtered_gyro() const { return filtered_gyro_; }

    // debug
    Eigen::Vector3f test_g1_, test_g2_;
    Eigen::Vector3f test_goffset_;

    void debug_eigen_mat(Eigen::MatrixXf mat) {
        for (int x = 0; x < mat.rows(); x++) {
            for (int y = 0; y < mat.cols(); y++) {
                printf("%f ", mat(x, y));
            }
            printf("\n");
        }
    }

private:
    Eigen::Quaternionf rpy2q(Eigen::Vector3f v) {
        using namespace Eigen;
        Quaternionf q = AngleAxisf(v[0], Vector3f::UnitX()) * AngleAxisf(v[1], Vector3f::UnitY()) *
                        AngleAxisf(v[2], Vector3f::UnitZ());
        return q;
    }

    void predict_error_convariance() {
        const float nu_sq = params_.nu * params_.nu;
        for (int i = 0; i < 3; i++) {
            // 左上3x3
            Q_(i, i) = P_(i, i) + kappa_sq_ * (P_(3 + i, 3 + i) + params_.beta + params_.eta);

            // 上3x3
            Q_(3 + i, i) = -kappa_ * (P_(3 + i, 3 + i) + params_.beta);

            // 左3x3
            Q_(i, 3 + i) = -kappa_ * (P_(3 + i, 3 + i) + params_.beta);

            // 中央3x3
            Q_(3 + i, 3 + i) = P_(3 + i, 3 + i) + params_.beta;

            // 右下3x3
            Q_(6 + i, 6 + i) = nu_sq * P_(6 + i, 6 + i) + params_.xi;
        }
        P_ = Q_;
    }

private:
    const Parameter params_;
    float kappa_;
    float kappa_sq_;

    // 状態ベクトル：[\theta_x, \theta_y, \theta_z, bx, by, bz, ax, ay, az]^T
    // \theta_x, \theta_y, \theta_z: 方向誤差
    // bx, by, bz: ジャイロのバイアス(センサーフレーム)
    // ax, ay, az: 加速度誤差(???フレーム)
    Eigen::VectorXf state_;

    Eigen::MatrixXf H_; // 観測モデル
    Eigen::MatrixXf P_; // 誤差共分散行列
    Eigen::MatrixXf R_; // 観測ノイズの共分散行列
    Eigen::MatrixXf Q_; // プロセスノイズの共分散行列

    Eigen::Quaternionf q_;          // 姿勢
    Eigen::Vector3f filtered_acc_;  // 加速度(センサーフレーム)
    Eigen::Vector3f gyro_offset_;   // ジャイロ(センサーフレーム)
    Eigen::Vector3f filtered_gyro_; // ジャイロ(センサーフレーム)
    Eigen::Vector3f prev_filtered_gyro_;
};
