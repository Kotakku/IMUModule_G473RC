#pragma once

#include "calibration_data.hpp"
#include "lsm6dsr_array_32.hpp"
#include <Eigen/Dense>

class IMUFusion {
public:
    using Vector3f = Eigen::Vector3f;
    static constexpr size_t NUM_SENSOR = LSM6DSRArray32::NUM_SENSOR;

    IMUFusion(LSM6DSRArray32 &imu_array, double Ts) : imu_array_(imu_array), Ts_(Ts) {

        // for (size_t i = 0; i < NUM_SENSOR; i++) {
        //     // bool is_exclude = false;
        //     // for (size_t ax = 0; ax < 3; ax++) {
        //     //     if (std::find(gyro_exlude_val_index.begin(), gyro_exlude_val_index.end(), 3 * i + ax) !=
        //     //     gyro_exlude_val_index.end()) {
        //     //         is_exclude = true;
        //     //         break;
        //     //     }
        //     // }
        //     // gyro_max_weight[i] = is_exclude ? 0 : 1;
        //     // gyro_max_weight[i] = 1;
        // }

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

        // imu_array_.set_debug_pa4(1); // 174us
        if (imu_array_.update_acc_axes() != LSM6DSR_OK) {
            return false;
        }

        if (imu_array_.update_gyro_axes() != LSM6DSR_OK) {
            return false;
        }

        // imu_array_.set_debug_pa5(1); // 71us 〜 85usぐらい(powの分岐で変わる)

        // 座標系を合わせた値を取得
        // debug_pa8 = 1;
        for (size_t i = 0; i < NUM_SENSOR; i++) {
            // 31us
            // tmp_acc_[i] = rotate_vec(imu_pose_rot[i], imu_array_.get_acc_axes(i));
            // tmp_gyro_[i] = rotate_vec(imu_pose_rot[i], imu_array_.get_gyro_axes(i));

            // 29us
            tmp_acc_[i] = rotate_vec_i(i, imu_array_.get_acc_axes(i));
            tmp_gyro_[i] = rotate_vec_i(i, imu_array_.get_gyro_axes(i));
        }

        // debug_pa8 = 0;

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
                // 除算の回数を減らす、0.5〜1us高速化
                acc_[ax] += tmp_acc_[i][ax] * fusion_acc_weights_[i][ax] * acc_weight_sum_inv_[ax];
                // acc_[ax] += tmp_acc_[i][ax] * fusion_acc_weights_[i][ax] / acc_weight_sum_[ax];
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
                // gyro_[ax] += tmp_gyro_[i][ax] * fusion_gyro_weights_[i][ax] / gyro_weight_sum_[ax];
            }
        }

        // 19us
        Vector3f acc_diff, gyro_diff;
        // 外れ値補正の重み更新
        for (size_t i = 0; i < NUM_SENSOR; i++) {
            // acc
            acc_diff = acc_ - tmp_acc_[i];
            for (size_t ax = 0; ax < 3; ax++) {
                acc_diff[ax] = abs(acc_diff[ax]);
                if (fusion_acc_delta_ < acc_diff[ax]) {
                    fusion_acc_weights_[i][ax] = my_pow(fusion_acc_delta_ / acc_diff[ax], fusion_acc_beta_);
                } else {
                    fusion_acc_weights_[i][ax] = 1;
                }
            }

            // gyro
            gyro_diff = gyro_ - tmp_gyro_[i];
            for (size_t ax = 0; ax < 3; ax++) {
                gyro_diff[ax] = abs(gyro_diff[ax]);
                if (fusion_gyro_delta_ < gyro_diff[ax]) {
                    fusion_gyro_weights_[i][ax] = my_pow(fusion_gyro_delta_ / gyro_diff[ax], fusion_gyro_beta_);
                } else {
                    fusion_gyro_weights_[i][ax] = 1;
                }
            }
        }

        // imu_array_.set_debug_pa4(0);
        // imu_array_.set_debug_pa5(0);

        return true;
    }

    inline float my_pow(float x, int y) const {
        // float result = 1.0;
        // for (int i = 0; i < y; i++) {
        //     result *= x;
        // }
        // return result;
        // return pow(x, y);
        return x * x;
    }

    Vector3f get_acc_fusion() const { return acc_; }
    Vector3f get_gyro_fusion() const { return gyro_; }

    Vector3f get_acc(size_t i) const { return tmp_acc_[i]; }
    Vector3f get_gyro(size_t i) const { return tmp_gyro_[i]; }

    std::array<Vector3f, NUM_SENSOR> &fusion_gyro_weights() { return fusion_gyro_weights_; }
    std::array<Vector3f, NUM_SENSOR> &fusion_acc_weights() { return fusion_acc_weights_; }

private:
    LSM6DSRArray32 &imu_array_;
    const double Ts_;
    std::array<Vector3f, NUM_SENSOR> tmp_acc_;
    std::array<Vector3f, NUM_SENSOR> tmp_gyro_;
    std::array<Vector3f, NUM_SENSOR> fusion_gyro_weights_, fusion_acc_weights_;
    Vector3f acc_weight_sum_, gyro_weight_sum_;
    Vector3f acc_weight_sum_inv_, gyro_weight_sum_inv_;
    static constexpr float gyro_var = 0.0025;
    static constexpr float fusion_gyro_delta_ = 2 * gyro_var; // 平均との差のしきい値
    static constexpr float fusion_gyro_beta_ = 2;             // 重みの減衰率(beta >= 1)
    static constexpr float acc_var = 0.009;
    static constexpr float fusion_acc_delta_ = 2 * acc_var; // 平均との差のしきい値
    static constexpr float fusion_acc_beta_ = 2;            // 重みの減衰率(beta >= 1)

    Vector3f acc_;
    Vector3f gyro_;
};
