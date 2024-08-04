#pragma once

#include "calibration_data.hpp"
#include "lsm6dsr_array_32.hpp"
#include <Eigen/Dense>

class IMUFusion {
public:
    using Vector3f = Eigen::Vector3f;
    static constexpr size_t NUM_SENSOR = LSM6DSRArray32::NUM_SENSOR;

    IMUFusion(LSM6DSRArray32 &imu_array, float fusion_gyro_delta, float fusion_gyro_beta, float fusion_acc_delta, float fusion_acc_beta) : 
        imu_array_(imu_array), fusion_gyro_delta_(fusion_gyro_delta), fusion_gyro_beta_(fusion_gyro_beta), fusion_acc_delta_(fusion_acc_delta), fusion_acc_beta_(fusion_acc_beta)
    {
        // initialize fusion weights
        for (size_t i = 0; i < NUM_SENSOR; i++) {
            fusion_gyro_weights_[i] = {1, 1, 1};
            fusion_acc_weights_[i] = {1, 1, 1};
        }
    }

    void begin() {
        imu_array_.begin();
        imu_array_.enable_acc();
        imu_array_.enable_gyro();
    }

    stmbed::DigitalOut debug_pa8{stmbed::PA8};

    bool update() {
        // 174us
        if (imu_array_.update_acc_axes() != LSM6DSR_OK) {
            return false;
        }

        if (imu_array_.update_gyro_axes() != LSM6DSR_OK) {
            return false;
        }

        // 71us 〜 85usぐらい(powの分岐で変わる)
        // 座標系を合わせた値を取得
        for (size_t i = 0; i < NUM_SENSOR; i++) {
            // 31us
            // tmp_acc_[i] = rotate_vec(imu_pose_rot[i], imu_array_.get_acc_axes(i));
            // tmp_gyro_[i] = rotate_vec(imu_pose_rot[i], imu_array_.get_gyro_axes(i));

            // 29us
            tmp_acc_[i] = rotate_vec_i(i, imu_array_.get_acc_axes(i) - gyro_biases[i]);
            tmp_gyro_[i] = rotate_vec_i(i, imu_array_.get_gyro_axes(i) - gyro_biases[i]);
        }

        // 温度補正
        // Todo

        // 干渉抑制
        // Todo

        // 外れ値補正による重み付き平均 21us
        acc_.setZero();
        gyro_.setZero();
        acc_weight_sum_.setZero();
        for (auto &w : fusion_acc_weights_) {
            acc_weight_sum_ += w;
        }
        for (size_t i = 0; i < 3; i++) {
            acc_weight_sum_inv_[i] = 1 / acc_weight_sum_[i];
        }
        for (size_t i = 0; i < NUM_SENSOR; i++) {
            for (size_t ax = 0; ax < 3; ax++) {
                acc_[ax] += tmp_acc_[i][ax] * fusion_acc_weights_[i][ax] * acc_weight_sum_inv_[ax];
            }
        }
        gyro_weight_sum_.setZero();
        for (auto &w : fusion_gyro_weights_) {
            gyro_weight_sum_ += w;
        }
        for (size_t i = 0; i < 3; i++) {
            gyro_weight_sum_inv_[i] = 1 / gyro_weight_sum_[i];
        }
        for (size_t i = 0; i < NUM_SENSOR; i++) {
            for (size_t ax = 0; ax < 3; ax++) {
                gyro_[ax] += tmp_gyro_[i][ax] * fusion_gyro_weights_[i][ax] * gyro_weight_sum_inv_[ax];
            }
        }

        // 19us
        // update weights
        Vector3f acc_diff, gyro_diff;
        for (size_t i = 0; i < NUM_SENSOR; i++) {
            // acc
            acc_diff = acc_ - tmp_acc_[i];
            for (size_t ax = 0; ax < 3; ax++) {
                acc_diff[ax] = abs(acc_diff[ax]);
                if (fusion_acc_delta_ < acc_diff[ax]) {
                    fusion_acc_weights_[i][ax] = std::pow(fusion_acc_delta_ / acc_diff[ax], fusion_acc_beta_);
                } else {
                    fusion_acc_weights_[i][ax] = 1;
                }
            }

            // gyro
            gyro_diff = gyro_ - tmp_gyro_[i];
            for (size_t ax = 0; ax < 3; ax++) {
                gyro_diff[ax] = abs(gyro_diff[ax]);
                if (fusion_gyro_delta_ < gyro_diff[ax]) {
                    fusion_gyro_weights_[i][ax] = std::pow(fusion_gyro_delta_ / gyro_diff[ax], fusion_gyro_beta_);
                } else {
                    fusion_gyro_weights_[i][ax] = 1;
                }
            }
        }

        return true;
    }

    Vector3f get_acc_fusion() const { return acc_; }
    Vector3f get_gyro_fusion() const { return gyro_; }

    Vector3f get_acc(size_t i) const { return tmp_acc_[i]; }
    Vector3f get_gyro(size_t i) const { return tmp_gyro_[i]; }

    std::array<Vector3f, NUM_SENSOR> &fusion_gyro_weights() { return fusion_gyro_weights_; }
    std::array<Vector3f, NUM_SENSOR> &fusion_acc_weights() { return fusion_acc_weights_; }

    Vector3f rotate_vec_i(size_t &i, const Vector3f &v) {
        if (i < 16) {
            switch (i % 4) {
            case 0:
                return Vector3f(v.x(), v.y(), v.z());
            case 1:
                return Vector3f(v.y(), -v.x(), v.z());
            case 2:
                return Vector3f(-v.x(), -v.y(), v.z());
            case 3:
                return Vector3f(-v.y(), v.x(), v.z());
            }
        } else {
            switch (i % 4) {
            case 0:
                return Vector3f(-v.x(), v.y(), -v.z());
            case 1:
                return Vector3f(v.y(), v.x(), -v.z());
            case 2:
                return Vector3f(v.x(), -v.y(), -v.z());
            case 3:
                return Vector3f(-v.y(), -v.x(), -v.z());
            }
        }

        return v;
    }

private:
    LSM6DSRArray32 &imu_array_;
    const float fusion_gyro_delta_; // 平均との差のしきい値
    const float fusion_gyro_beta_ ; // 重みの減衰率(beta >= 1)
    const float fusion_acc_delta_;  // 平均との差のしきい値
    const float fusion_acc_beta_;   // 重みの減衰率(beta >= 1)
    std::array<Vector3f, NUM_SENSOR> tmp_acc_;
    std::array<Vector3f, NUM_SENSOR> tmp_gyro_;
    std::array<Vector3f, NUM_SENSOR> fusion_gyro_weights_, fusion_acc_weights_;
    Vector3f acc_weight_sum_, gyro_weight_sum_;
    Vector3f acc_weight_sum_inv_, gyro_weight_sum_inv_;


    Vector3f acc_;
    Vector3f gyro_;
};
