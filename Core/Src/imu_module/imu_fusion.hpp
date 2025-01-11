#pragma once

#include "arm_math.h"
#include "calibration_data.hpp"
#include "lsm6dsr_array.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace imu_module {

class IMUFusion {
public:
    using QScalar = q15_t;
    static constexpr QScalar QMAX = Q15_ABSMAX;
    using Vector3q = Eigen::Matrix<QScalar, 3, 1>;
    using Vector3f = Eigen::Vector3f;
    static constexpr size_t NUM_SENSOR = LSM6DSRArray::NUM_SENSOR;

    struct ArrayedVector {
        std::array<QScalar, NUM_SENSOR> x, y, z;
    };

    IMUFusion(LSM6DSRArray &imu_array, float fusion_gyro_delta, float fusion_acc_delta, const float Ts)
        : imu_array_(imu_array), fusion_gyro_delta_(fusion_gyro_delta), fusion_acc_delta_(fusion_acc_delta), Ts_(Ts) {}

    // QScalar to_q_scalar(float val) { return static_cast<QScalar>(val * (QMAX + 1)); }

    void begin() {
        imu_array_.begin();
        imu_array_.enable_acc();
        imu_array_.enable_gyro();

        // initialize fusion weights
        const QScalar init_weight = QMAX;

        const float acc_sense = imu_array_.get_acc_sensitivity();
        const float gyro_sense = imu_array_.get_gyro_sensitivity();

        using namespace calibration_data;
        calcu_imu_bias_from_3pose_data(gyro_biases_, acc_biases_);

        printf("bias\n");
        for (size_t i = 0; i < NUM_SENSOR; i++) {
            fusion_gyro_weights_.x[i] = init_weight;
            fusion_gyro_weights_.y[i] = init_weight;
            fusion_gyro_weights_.z[i] = init_weight;
            fusion_acc_weights_.x[i] = init_weight;
            fusion_acc_weights_.y[i] = init_weight;
            fusion_acc_weights_.z[i] = init_weight;
            acc_bias_array_.x[i] = static_cast<q15_t>(acc_biases_[i].x() / acc_sense);
            acc_bias_array_.y[i] = static_cast<q15_t>(acc_biases_[i].y() / acc_sense);
            acc_bias_array_.z[i] = static_cast<q15_t>(acc_biases_[i].z() / acc_sense);
            gyro_bias_array_.x[i] = static_cast<q15_t>(gyro_biases_[i].x() / gyro_sense);
            gyro_bias_array_.y[i] = static_cast<q15_t>(gyro_biases_[i].y() / gyro_sense);
            gyro_bias_array_.z[i] = static_cast<q15_t>(gyro_biases_[i].z() / gyro_sense);

            // printf("%.3f,%.3f,%.3f,%d\n", gyro_biases_[i].x(), gyro_sense, gyro_biases_[i].x() / gyro_sense,
            //        gyro_bias_array_.x[i]);

            // printf("%d,%d,%d,%d,%d,%d\n", acc_bias_array_.x[i], acc_bias_array_.y[i], acc_bias_array_.z[i],
            //        gyro_bias_array_.x[i], gyro_bias_array_.y[i], gyro_bias_array_.z[i]);
        }

        // while (1)
        //     ;

        fusion_gyro_delta_q_ = static_cast<q15_t>(fusion_gyro_delta_ / gyro_sense);
        fusion_acc_delta_q_ = static_cast<q15_t>(fusion_acc_delta_ / acc_sense);
    }

    stmbed::DigitalOut debug_pa8{stmbed::PA8};

    // 146us
    bool update() {
        debug_pa8 = 1;

        // データ読み出し 174->117us
        if (imu_array_.update_acc_axes() != LSM6DSR_OK) {
            return false;
        }

        if (imu_array_.update_gyro_axes() != LSM6DSR_OK) {
            return false;
        }

        // バイアス除去 6.7us
        for (size_t i = 0; i < NUM_SENSOR; i++) {
            raw_acc_array_.x[i] = imu_array_.sensor_data()[i].acc.raw_value[0] - acc_bias_array_.x[i];
            raw_acc_array_.y[i] = imu_array_.sensor_data()[i].acc.raw_value[1] - acc_bias_array_.y[i];
            raw_acc_array_.z[i] = imu_array_.sensor_data()[i].acc.raw_value[2] - acc_bias_array_.z[i];
            raw_gyro_array_.x[i] = imu_array_.sensor_data()[i].gyro.raw_value[0] - gyro_bias_array_.x[i];
            raw_gyro_array_.y[i] = imu_array_.sensor_data()[i].gyro.raw_value[1] - gyro_bias_array_.y[i];
            raw_gyro_array_.z[i] = imu_array_.sensor_data()[i].gyro.raw_value[2] - gyro_bias_array_.z[i];
        }

        // 座標系を合わせる 29us-> 11us
        rotate_pop(raw_acc_array_, rot_acc_array_);
        rotate_pop(raw_gyro_array_, rot_gyro_array_);

        // 温度補正
        // Todo

        // 干渉抑制
        // Todo

        // 外れ値補正による重み付き平均 18us
        calcu_weighted_avg(rot_acc_array_, fusion_acc_weights_, raw_acc_);
        calcu_weighted_avg(rot_gyro_array_, fusion_gyro_weights_, raw_gyro_);

        // 重みの更新 10us
        update_weights(raw_acc_, rot_acc_array_, fusion_acc_weights_, fusion_acc_delta_q_);
        update_weights(raw_gyro_, rot_gyro_array_, fusion_gyro_weights_, fusion_gyro_delta_q_);

        // 単位変換
        // 0.76us
        acc_ = raw_acc_.cast<float>() * imu_array_.get_acc_sensitivity();
        gyro_ = raw_gyro_.cast<float>() * imu_array_.get_gyro_sensitivity();

        debug_pa8 = 0;

        return true;
    }

    Vector3f get_acc_fusion() const { return acc_; }
    Vector3f get_gyro_fusion() const { return gyro_; }

    // Vector3f get_acc(size_t i) const { return tmp_acc_[i]; }
    Vector3f get_gyro(size_t i) const {
        return {rot_gyro_array_.x[i] * imu_array_.get_gyro_sensitivity(),
                rot_gyro_array_.y[i] * imu_array_.get_gyro_sensitivity(),
                rot_gyro_array_.z[i] * imu_array_.get_gyro_sensitivity()};
    }

    //    std::array<Vector3f, NUM_SENSOR> &fusion_gyro_weights() { return fusion_gyro_weights_; }
    //    std::array<Vector3f, NUM_SENSOR> &fusion_acc_weights() { return fusion_acc_weights_; }

    static Vector3f rotate_vec_i(size_t &i, const Vector3f &v) {
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
                return Vector3f(-v.y(), -v.x(), -v.z());
            case 2:
                return Vector3f(v.x(), -v.y(), -v.z());
            case 3:
                return Vector3f(v.y(), v.x(), -v.z());
            }
        }

        return v;
    }

    static Vector3f rotate_vec_inv_i(size_t &i, const Vector3f &v) {
        if (i < 16) {
            switch (i % 4) {
            case 0:
                return Vector3f(v.x(), v.y(), v.z());
            case 1:
                return Vector3f(-v.y(), v.x(), v.z());
            case 2:
                return Vector3f(-v.x(), -v.y(), v.z());
            case 3:
                return Vector3f(v.y(), -v.x(), v.z());
            }
        } else {
            switch (i % 4) {
            case 0:
                return Vector3f(-v.x(), v.y(), -v.z());
            case 1:
                return Vector3f(-v.y(), -v.x(), -v.z());
            case 2:
                return Vector3f(v.x(), -v.y(), -v.z());
            case 3:
                return Vector3f(v.y(), v.x(), -v.z());
            }
        }
    }

    void rotate_pop(const ArrayedVector &raw_data, ArrayedVector &output) {
        for (size_t i = 0; i < NUM_SENSOR; i++) {
            if (i < 16) {
                switch (i % 4) {
                case 0:
                    // return Vector3f(v.x(), v.y(), v.z());
                    output.x[i] = raw_data.x[i];
                    output.y[i] = raw_data.y[i];
                    output.z[i] = raw_data.z[i];
                    break;
                case 1:
                    // return Vector3f(v.y(), -v.x(), v.z());
                    output.x[i] = raw_data.y[i];
                    output.y[i] = -raw_data.x[i];
                    output.z[i] = raw_data.z[i];
                    break;

                case 2:
                    // return Vector3f(-v.x(), -v.y(), v.z());
                    output.x[i] = -raw_data.x[i];
                    output.y[i] = -raw_data.y[i];
                    output.z[i] = raw_data.z[i];
                    break;

                case 3:
                    // return Vector3f(-v.y(), v.x(), v.z());
                    output.x[i] = -raw_data.y[i];
                    output.y[i] = raw_data.x[i];
                    output.z[i] = raw_data.z[i];
                    break;
                }
            } else {
                switch (i % 4) {
                case 0:
                    // return Vector3f(-v.x(), v.y(), -v.z());
                    output.x[i] = -raw_data.x[i];
                    output.y[i] = raw_data.y[i];
                    output.z[i] = -raw_data.z[i];
                    break;

                case 1:
                    // return Vector3f(-v.y(), -v.x(), -v.z());
                    output.x[i] = -raw_data.y[i];
                    output.y[i] = -raw_data.x[i];
                    output.z[i] = -raw_data.z[i];
                    break;

                case 2:
                    // return Vector3f(v.x(), -v.y(), -v.z());
                    output.x[i] = raw_data.x[i];
                    output.y[i] = -raw_data.y[i];
                    output.z[i] = -raw_data.z[i];
                    break;

                case 3:
                    // return Vector3f(v.y(), v.x(), -v.z());
                    output.x[i] = raw_data.y[i];
                    output.y[i] = raw_data.x[i];
                    output.z[i] = -raw_data.z[i];
                    break;
                }
            }
        }
    }

    void calcu_weighted_avg(ArrayedVector &input, ArrayedVector &weights, Vector3q &output) {
        // x
        q63_t sum = 0;
        q63_t weight_sum = 0;
        for (size_t i = 0; i < IMUFusion::NUM_SENSOR; i++) {
            sum += input.x[i] * weights.x[i];
            weight_sum += weights.x[i];
        }
        output.x() = (QScalar)(sum / weight_sum);

        // y
        sum = 0;
        weight_sum = 0;
        for (size_t i = 0; i < IMUFusion::NUM_SENSOR; i++) {
            sum += input.y[i] * weights.y[i];
            weight_sum += weights.y[i];
        }
        output.y() = (QScalar)(sum / weight_sum);

        // z
        sum = 0;
        weight_sum = 0;
        for (size_t i = 0; i < IMUFusion::NUM_SENSOR; i++) {
            sum += input.z[i] * weights.z[i];
            weight_sum += weights.z[i];
        }
        output.z() = (QScalar)(sum / weight_sum);
    }

    q15_t calculate_q15_square(q15_t a, q15_t b) {
        if (b == 0) {
            // 分母が0の場合は適切に処理する（ここでは0を返す）
            return 0;
        }

        // スケール補正のために 32bit に拡張
        int32_t temp = ((int32_t)a << 15) / b; // a / b を計算し、スケールを維持
        int32_t result = (temp) >> 15;         // (a / b)^2 を計算し、スケール調整

        // 範囲を q15_t にクリップ
        if (result > 32767)
            result = 32767;
        if (result < -32768)
            result = -32768;

        return (q15_t)result;
    }

    void update_weights(const Vector3q &avg, ArrayedVector &tmp, ArrayedVector &weights, QScalar delta) {
        Vector3q diff;
        for (size_t i = 0; i < NUM_SENSOR; i++) {
            diff = avg - Vector3q(tmp.x[i], tmp.y[i], tmp.z[i]);
            for (size_t ax = 0; ax < 3; ax++) {
                diff[ax] = std::abs(diff[ax]);
                if (delta < diff[ax]) {
                    // optimize beta=2
                    // weights.x[i] = sq(delta / diff[ax]);
                    weights.x[i] = calculate_q15_square(delta, diff[ax]);
                } else {
                    weights.x[i] = QMAX;
                }
            }
        }
    }

private:
    LSM6DSRArray &imu_array_;
    const float fusion_gyro_delta_; // 平均との差のしきい値
    // const float fusion_gyro_beta_;  // 重みの減衰率(beta >= 1)
    const float fusion_acc_delta_; // 平均との差のしきい値
    // const float fusion_acc_beta_;   // 重みの減衰率(beta >= 1)
    const float Ts_;

    std::array<Vector3f, NUM_SENSOR> gyro_biases_;
    std::array<Vector3f, NUM_SENSOR> acc_biases_;

    q15_t fusion_gyro_delta_q_;
    q15_t fusion_acc_delta_q_;

    ArrayedVector raw_acc_array_;
    ArrayedVector raw_gyro_array_;
    ArrayedVector rot_acc_array_;
    ArrayedVector rot_gyro_array_;
    ArrayedVector acc_bias_array_;
    ArrayedVector gyro_bias_array_;
    ArrayedVector fusion_gyro_weights_;
    ArrayedVector fusion_acc_weights_;
    Vector3q raw_acc_;
    Vector3q raw_gyro_;

    Vector3f acc_;
    Vector3f gyro_;
};

} // namespace imu_module
