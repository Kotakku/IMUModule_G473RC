#pragma once

#include "board.hpp"

namespace imu_module::calibration_func {

#define printf_calibration_func()                                                                                      \
    { printf("calibration: %s\n", __func__); }

void imu_bias_calibration() {
    Peripherals::enable_std_printf();
    printf_calibration_func();

    auto &imu = Peripherals::get_instance().imu;
    auto &main_ticker = Peripherals::get_instance().main_ticker;
    auto &led1 = Peripherals::get_instance().led1;
    auto &led2 = Peripherals::get_instance().led2;

    // 連続でサンプリングするバッチ数の累乗(2^NUM_SAMPLING_BATCH_SHIFT)、1バッチごとに平均を計算する
    constexpr size_t NUM_SAMPLING_BATCH_SHIFT = 10; // 2^10 = 1024 samples 1ms周期で大体1秒

    // 複数バッチでの平均を取る回数(2^NUM_AVG_BATCH_SHIFT)
    constexpr size_t NUM_AVG_BATCH_SHIFT = 8; // 2^8 = 256 samples 1秒周期で5分弱
    size_t sampling_cnt = 0;
    size_t avg_cnt = 0;
    bool sampling_finished = false;

    using Vector3q15 = Eigen::Matrix<q15_t, 3, 1>;
    using Vector3q31 = Eigen::Matrix<q31_t, 3, 1>;
    std::array<Vector3q31, NUM_SENSOR> gyro_sampling_sum_data;
    std::array<Vector3q31, NUM_SENSOR> acc_sampling_sum_data;
    std::array<Vector3q31, NUM_SENSOR> gyro_avg_sum_data;
    std::array<Vector3q31, NUM_SENSOR> acc_avg_sum_data;
    std::array<Vector3q15, NUM_SENSOR> gyro_bias;
    std::array<Vector3q15, NUM_SENSOR> acc_bias;

    // 0で初期化
    for (size_t i = 0; i < NUM_SENSOR; i++) {
        for (size_t j = 0; j < 3; j++) {
            gyro_sampling_sum_data[i][j] = 0;
            acc_sampling_sum_data[i][j] = 0;
            gyro_avg_sum_data[i][j] = 0;
            acc_avg_sum_data[i][j] = 0;
        }
    }

    imu.begin();
    imu.enable_acc();
    imu.enable_gyro();

    HAL_Delay(200);

    // 起動直後のデータは捨てる
    const size_t INIT_LOOP_NUM = 1000;
    for (size_t j = 0; j < INIT_LOOP_NUM; j++) {
        imu.update_acc_axes();
        imu.update_gyro_axes();
        HAL_Delay(1);
    }

    printf("%d/%d\n", 0, (1UL << NUM_AVG_BATCH_SHIFT));

    main_ticker.attach(
        [&]() {
            imu.update_acc_axes();
            imu.update_gyro_axes();

            if (sampling_finished) {
                return;
            }

            if (avg_cnt == (1UL << NUM_AVG_BATCH_SHIFT)) {
                sampling_finished = true;

                // 平均を計算
                for (size_t i = 0; i < NUM_SENSOR; i++) {
                    gyro_bias[i].x() = static_cast<q15_t>(gyro_avg_sum_data[i].x() >> NUM_AVG_BATCH_SHIFT);
                    gyro_bias[i].y() = static_cast<q15_t>(gyro_avg_sum_data[i].y() >> NUM_AVG_BATCH_SHIFT);
                    gyro_bias[i].z() = static_cast<q15_t>(gyro_avg_sum_data[i].z() >> NUM_AVG_BATCH_SHIFT);
                    acc_bias[i].x() = static_cast<q15_t>(acc_avg_sum_data[i].x() >> NUM_AVG_BATCH_SHIFT);
                    acc_bias[i].y() = static_cast<q15_t>(acc_avg_sum_data[i].y() >> NUM_AVG_BATCH_SHIFT);
                    acc_bias[i].z() = static_cast<q15_t>(acc_avg_sum_data[i].z() >> NUM_AVG_BATCH_SHIFT);
                }

                return;
            }

            if (sampling_cnt < (1UL << NUM_SAMPLING_BATCH_SHIFT)) {
                // サンプリング
                for (size_t i = 0; i < NUM_SENSOR; i++) {
                    gyro_sampling_sum_data[i].x() += imu.get_gyro_axes_raw(i)[0];
                    gyro_sampling_sum_data[i].y() += imu.get_gyro_axes_raw(i)[1];
                    gyro_sampling_sum_data[i].z() += imu.get_gyro_axes_raw(i)[2];

                    acc_sampling_sum_data[i].x() += imu.get_acc_axes_raw(i)[0];
                    acc_sampling_sum_data[i].y() += imu.get_acc_axes_raw(i)[1];
                    acc_sampling_sum_data[i].z() += imu.get_acc_axes_raw(i)[2];
                }
                // printf("%d\n", imu.get_acc_axes_raw(0)[2]);

                sampling_cnt++;
            } else {
                // 1バッチ分の平均を計算
                for (size_t i = 0; i < NUM_SENSOR; i++) {
                    gyro_avg_sum_data[i].x() += gyro_sampling_sum_data[i].x() >> NUM_SAMPLING_BATCH_SHIFT;
                    gyro_avg_sum_data[i].y() += gyro_sampling_sum_data[i].y() >> NUM_SAMPLING_BATCH_SHIFT;
                    gyro_avg_sum_data[i].z() += gyro_sampling_sum_data[i].z() >> NUM_SAMPLING_BATCH_SHIFT;
                    acc_avg_sum_data[i].x() += acc_sampling_sum_data[i].x() >> NUM_SAMPLING_BATCH_SHIFT;
                    acc_avg_sum_data[i].y() += acc_sampling_sum_data[i].y() >> NUM_SAMPLING_BATCH_SHIFT;
                    acc_avg_sum_data[i].z() += acc_sampling_sum_data[i].z() >> NUM_SAMPLING_BATCH_SHIFT;

                    gyro_sampling_sum_data[i].setZero();
                    acc_sampling_sum_data[i].setZero();
                }

                sampling_cnt = 0;
                avg_cnt++;

                printf("%d/%d\n", avg_cnt, (1UL << NUM_AVG_BATCH_SHIFT));
            }
        },
        2); // 1kHzで呼び出す

    while (1) {
        led1 = 1;
        led2 = 0;
        HAL_Delay(200);
        led1 = 0;
        led2 = 1;
        HAL_Delay(200);

        if (sampling_finished)
            break;
    }

    // 結果の表示
    printf("---------- copy below to calibration_data.hpp ----------\n");
    printf("const static std::array<Vector3f, NUM_SENSOR> gyro_average = {\n");
    for (size_t i = 0; i < NUM_SENSOR; i++) {
        const float gyro_sense = imu.get_gyro_sensitivity();
        printf("\tVector3f{%e, %e, %e},  \n", gyro_bias[i].x() * gyro_sense, gyro_bias[i].y() * gyro_sense,
               gyro_bias[i].z() * gyro_sense);
    }
    printf("};\n\n");

    printf("const static std::array<Vector3f, NUM_SENSOR> acc_average = {\n");
    for (size_t i = 0; i < NUM_SENSOR; i++) {
        const float acc_sense = imu.get_acc_sensitivity();
        printf("\tVector3f{%e, %e, %e},  \n", acc_bias[i].x() * acc_sense, acc_bias[i].y() * acc_sense,
               acc_bias[i].z() * acc_sense);
    }
    printf("};\n\n");

    while (1) {
        led1 = 1;
        HAL_Delay(500);
    }
}

void check_imu_bias() {
    Peripherals::enable_std_printf();
    printf_calibration_func();
    auto &led1 = Peripherals::get_instance().led1;
    auto &led2 = Peripherals::get_instance().led2;

    std::array<Vector3f, NUM_SENSOR> gyro_biases;
    std::array<Vector3f, NUM_SENSOR> acc_biases;

    using namespace calibration_data;
    calcu_imu_bias_from_3pose_data(gyro_biases, acc_biases);

    printf("gyro_biases = {\n");
    for (size_t i = 0; i < NUM_SENSOR; i++) {
        printf("\t{%e, %e, %e},  \n", gyro_biases[i].x(), gyro_biases[i].y(), gyro_biases[i].z());
    }
    printf("};\n\n");

    printf("acc_biases = {\n");
    for (size_t i = 0; i < NUM_SENSOR; i++) {
        printf("\t{%e, %e, %e},  \n", acc_biases[i].x(), acc_biases[i].y(), acc_biases[i].z());
    }
    printf("};\n\n");

    while (1) {
        led1 = 1;
        led2 = 0;
        HAL_Delay(500);
        led1 = 0;
        led2 = 1;
        HAL_Delay(500);
    }
}

} // namespace imu_module::calibration_func
