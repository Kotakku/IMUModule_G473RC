#pragma once

#include "calibration_data.hpp"
#include "lsm6dsr_array_32.hpp"
#include <Eigen/Dense>

#include "cpp_robotics/vector/vector3.hpp"

class IMUFusion {
public:
    using Vector3f = cpp_robotics::Vector3f;
    static constexpr size_t NUM_SENSOR = LSM6DSRArray32::NUM_SENSOR;

    IMUFusion(LSM6DSRArray32 &imu_array, double Ts) : imu_array_(imu_array), Ts_(Ts) {

        for (size_t i = 0; i < NUM_SENSOR; i++) {
            // bool is_exclude = false;

            // for (size_t ax = 0; ax < 3; ax++) {
            //     if (std::find(gyro_exlude_val_index.begin(), gyro_exlude_val_index.end(), 3 * i + ax) !=
            //     gyro_exlude_val_index.end()) {
            //         is_exclude = true;
            //         break;
            //     }
            // }

            // gyro_max_weight[i] = is_exclude ? 0 : 1;
            // gyro_max_weight[i] = 1;
        }

        for (size_t i = 0; i < NUM_SENSOR; i++) {
            // fusion_gyro_weights_[i] = gyro_max_weight[i];
            // fusion_acc_weights_[i] = gyro_max_weight[i];
            fusion_gyro_weights_[i] = 1.0f;
            fusion_acc_weights_[i] = 1.0f;
        }
    }

    void begin() {
        imu_array_.begin();
        imu_array_.enable_acc();
        imu_array_.enable_gyro();
    }

    bool update() {
        if (imu_array_.update_acc_axes() != LSM6DSR_OK) {
            return false;
        }

        if (imu_array_.update_gyro_axes() != LSM6DSR_OK) {
            return false;
        }

        // バイアス除去したセンサの値を取得
        for (size_t i = 0; i < NUM_SENSOR; i++) {
            tmp_acc_[i] = imu_array_.get_acc_axes(i);
            tmp_gyro_[i] = imu_array_.get_gyro_axes(i);
        }

        // 座標系を合わせる
        for (size_t i = 0; i < NUM_SENSOR; i++) {
            tmp_acc_[i] = rotate_vec(imu_pose_rot[i], tmp_acc_[i]);
            tmp_gyro_[i] = rotate_vec(imu_pose_rot[i], tmp_gyro_[i]);
        }

        // 温度補正
        // Todo

        // 干渉抑制
        // for (size_t i = 0; i < 16; i++) {
        //     auto &filter_list = ni_filters[i];
        //     for (auto &filter_pair : filter_list) {
        //         auto &nf = filter_pair.first;
        //         auto &isf = filter_pair.second;

        //         for (size_t j = 0; j < NUM_SENSOR; j++) {
        //             if (i == j) {
        //                 for (size_t ax = 0; ax < 3; ax++) {
        //                     tmp_gyro_[j][ax] = nf[ax].responce(tmp_gyro_[j][ax]);
        //                 }
        //             } else {
        //                 for (size_t ax = 0; ax < 3; ax++) {
        //                     tmp_gyro_[j][ax] = isf[ax].responce(tmp_gyro_[j][ax]);
        //                 }
        //             }
        //         }
        //     }
        // }

        // 外れ値補正による重み付き平均
        acc_.set(0, 0, 0);
        gyro_.set(0, 0, 0);
        float acc_weight_sum = 0;
        for (auto &w : fusion_acc_weights_) {
            acc_weight_sum += w;
        }
        for (size_t i = 0; i < NUM_SENSOR; i++) {
            float weight = fusion_acc_weights_[i] / acc_weight_sum;
            acc_ += tmp_acc_[i] * weight;
        }
        float gyro_weight_sum = 0;
        for (auto &w : fusion_gyro_weights_) {
            gyro_weight_sum += w;
        }
        for (size_t i = 0; i < NUM_SENSOR; i++) {
            float weight = fusion_gyro_weights_[i] / gyro_weight_sum;
            gyro_ += tmp_gyro_[i] * weight;
        }

        // 外れ値補正の重み更新
        for (size_t i = 0; i < NUM_SENSOR; i++) {
            // acc
            // float acc_diff = abs((acc_ - tmp_acc_[i]).norm());
            // if (fusion_acc_delta_ < acc_diff) {
            //     fusion_acc_weights_[i] = std::pow(fusion_acc_delta_ / acc_diff, fusion_acc_beta_);
            // } else {
            //     fusion_acc_weights_[i] = 1;
            // }

            // gyro
            // float gyro_diff = abs((gyro_ - tmp_gyro_[i]).norm());
            float gyro_diff = abs(gyro_[2] - tmp_gyro_[i][2]); // Todo
            if (fusion_gyro_delta_ < gyro_diff) {
                fusion_gyro_weights_[i] = std::pow(fusion_gyro_delta_ / gyro_diff, fusion_gyro_beta_);
            } else {
                fusion_gyro_weights_[i] = 1;
            }
        }

        return true;
    }

    Vector3f get_acc_fusion() const { return acc_; }
    Vector3f get_gyro_fusion() const { return gyro_; }

    Vector3f get_acc(size_t i) const { return tmp_acc_[i]; }
    Vector3f get_gyro(size_t i) const { return tmp_gyro_[i]; }

    std::array<float, NUM_SENSOR> &fusion_gyro_weights() { return fusion_gyro_weights_; }
    std::array<float, NUM_SENSOR> &fusion_acc_weights() { return fusion_acc_weights_; }

private:
    LSM6DSRArray32 &imu_array_;
    const double Ts_;
    std::array<Vector3f, NUM_SENSOR> tmp_acc_;
    std::array<Vector3f, NUM_SENSOR> tmp_gyro_;
    std::array<float, NUM_SENSOR> fusion_gyro_weights_, fusion_acc_weights_;
    static constexpr float gyro_var = 0.0025;
    static constexpr float fusion_gyro_delta_ = 2 * gyro_var; // 平均との差のしきい値
    static constexpr float fusion_gyro_beta_ = 4;             // 重みの減衰率(beta >= 1)
    static constexpr float acc_var = 0.0032;
    static constexpr float fusion_acc_delta_ = 2 * acc_var; // 平均との差のしきい値
    static constexpr float fusion_acc_beta_ = 4;            // 重みの減衰率(beta >= 1)

    Vector3f acc_;
    Vector3f gyro_;
};
