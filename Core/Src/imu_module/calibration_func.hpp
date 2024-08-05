#pragma once

#include "board.hpp"

namespace imu_module::calibration_func {

#define printf_calibration_func()                                                                                             \
    { printf("calibration: %s\n", __func__); }

// ジャイロのバイアスを計測する関数
// 基板を水平に置いて、静止した状態で計測する
void imu_get_gyro_bias() {
    Peripherals::enable_std_printf();
    printf_calibration_func();

    auto &imu = Peripherals::get_instance().imu;

    std::array<Vector3f, 32> biases;
    float gain = 0.1;
    const size_t change_gain_interval = 100;
    const size_t change_gain_num = 20;
    const float gain_change_rate = 0.5;

    size_t cnt = 0;
    size_t change_gain_cnt = 0;

    imu.begin();
    imu.enable_acc();
    imu.enable_gyro();

    HAL_Delay(200);

    for (size_t i = 0; i < 32; i++) {
        biases[i] = {0, 0, 0};
    }

    const size_t init_loop_num = 1000;
    for (size_t j = 0; j < init_loop_num; j++) {
        imu.update_acc_axes();
        imu.update_gyro_axes();
        HAL_Delay(1);
    }

    while (1) {
        imu.update_gyro_axes();

        for (size_t i = 0; i < 32; i++) {
            biases[i] += (imu.get_gyro_axes(i) - biases[i]) * gain;
            if (i != 31) {
                printf("%.8f,%.8f,%.8f,", biases[i].x(), biases[i].y(), biases[i].z());
            } else {
                printf("%.8f,%.8f,%.8f\n", biases[i].x(), biases[i].y(), biases[i].z());
            }
        }

        if (change_gain_cnt < change_gain_num) {
            cnt++;
            if (cnt % change_gain_interval == 0) {
                gain *= gain_change_rate;
                cnt = 0;
                change_gain_cnt++;
            }
        }
        else {
            break;
        }
    }

    // finish calibration data output
    printf("=== copy below data to imu_module::calibration_data::gyro_biases ===\n");
    for (size_t i = 0; i < IMUFusion::NUM_SENSOR; i++) {
        printf("\tVector3f{%e, %e, %e},\n", i, biases[i].x(), biases[i].y(), biases[i].z());
    }


    while(1) {
        
    }
}

// 加速度センサのバイアスを計測する関数
// 基板を水平に置いて、静止した状態で計測する
void imu_get_acc_bias() {
    Peripherals::enable_std_printf();
    printf_calibration_func();

    auto &imu = Peripherals::get_instance().imu;

    std::array<Vector3f, 32> biases;
    float gain = 0.1;
    const size_t change_gain_interval = 100;
    const size_t change_gain_num = 20;
    const float gain_change_rate = 0.5;

    size_t cnt = 0;
    size_t change_gain_cnt = 0;

    imu.begin();
    imu.enable_acc();
    imu.enable_gyro();

    for (size_t i = 0; i < 32; i++) {
        biases[i] = {0, 0, 0};
    }

    for (size_t i = 0; i < 100; i++) {
        imu.update_acc_axes();
        imu.update_gyro_axes();
        HAL_Delay(1);
    }

    while (1) {
        imu.update_acc_axes();

        for (size_t i = 0; i < 32; i++) {

            Vector3f acc = imu.get_acc_axes(i);
            if (i < 16) {
                acc.z() -= GRAVITY;
            } else {
                acc.z() += GRAVITY;
            }

            biases[i] += (acc - biases[i]) * gain;

            if (i != 31) {
                printf("%.8f,%.8f,%.8f,", biases[i].x(), biases[i].y(), biases[i].z());
            } else {
                printf("%.8f,%.8f,%.8f\n", biases[i].x(), biases[i].y(), biases[i].z());
            }
        }

        if (change_gain_cnt < change_gain_num) {
            cnt++;
            if (cnt % change_gain_interval == 0) {
                gain *= gain_change_rate;
                cnt = 0;
                change_gain_cnt++;
            }
        }
        else {
            break;
        }
    }

    // finish calibration data output
    printf("=== copy below data to imu_module::calibration_data::acc_biases ===\n");
    for (size_t i = 0; i < IMUFusion::NUM_SENSOR; i++) {
        printf("\tVector3f{%e, %e, %e},\n", i, biases[i].x(), biases[i].y(), biases[i].z());
    }

    while(1) {
        
    }
}
    
}