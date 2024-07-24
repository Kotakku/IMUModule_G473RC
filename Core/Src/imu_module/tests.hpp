#pragma once

#include "board.hpp"

namespace imu_module {

#define printf_test_func()                                                                                             \
    { printf("test: %s\n", __func__); }

namespace unit_test {

void led_test() {
    auto &led1 = Peripherals::get_instance().led1;
    auto &led2 = Peripherals::get_instance().led2;

    while (1) {
        led1 = 1;
        led2 = 0;
        HAL_Delay(300);
        led1 = 0;
        led2 = 1;
        HAL_Delay(300);
    }
}

void vcp_test() {
    Peripherals::enable_std_printf();
    printf("Hello, world!\n");
    uint8_t cnt = 0;
    while (1) {
        printf("Hello, world! %d\n", cnt++);
        HAL_Delay(1000);
    }
}

void imu_whoami_test() {
    Peripherals::enable_std_printf();
    printf_test_func();

    auto &imu = Peripherals::get_instance().imu;

    imu.begin();
    imu.enable_acc();
    imu.enable_gyro();

    std::array<uint8_t, 32> result;
    if (imu.who_am_i(result) == LSM6DSR_OK) {
        printf("who am i is OK.\n");
    } else {
        printf("who am i is not OK!!!\n");
    }

    printf("read data\n");
    for (auto &v : result) {
        printf("%02X\n", v);
    }

    printf("test finished\n");

    while (1) {
        HAL_Delay(100);
    }
}

void imu_read_raw_test() {
    Peripherals::enable_std_printf();
    printf_test_func();

    auto &imu = Peripherals::get_instance().imu;

    imu.begin();
    imu.enable_acc();
    imu.enable_gyro();

    while (1) {
        imu.update_acc_axes();
        imu.update_gyro_axes();
        auto acc = imu.get_acc_axes_raw(0);
        auto gyro = imu.get_gyro_axes_raw(0);

        printf("acc: %.3f %.3f %.3f\n", acc.x, acc.y, acc.z);
        printf("gyro: %.3f %.3f %.3f\n", gyro.x, gyro.y, gyro.z);

        HAL_Delay(1000);
    }
}

void imu_axes_check_test() {
    Peripherals::enable_std_printf();
    printf_test_func();

    auto &imu_fusion = Peripherals::get_instance().imu_fusion;
    auto &tp = Peripherals::get_instance().tp;

    imu_fusion.begin();

    while (1) {
        imu_fusion.update();

        tp = 1;
        for (size_t i = 0; i < IMUFusion::NUM_SENSOR; i++) {
            printf("%.3f", imu_fusion.get_gyro(i).z);
            if (i == IMUFusion::NUM_SENSOR - 1)
                printf("\n");
            else
                printf(",");
        }
        tp = 0;

        // HAL_Delay(1);
    }
}

void ticker_test() {
    Peripherals::enable_std_printf();
    printf_test_func();

    auto &ticker_1khz = Peripherals::get_instance().ticker_1khz;
    auto &tp = Peripherals::get_instance().tp;

    ticker_1khz.attach([&]() { tp = !tp; });

    while (1)
        ;
}

void imu_fusion_test() {
    Peripherals::enable_std_printf();
    printf_test_func();

    auto &imu_fusion = Peripherals::get_instance().imu_fusion;
    auto &ticker_1khz = Peripherals::get_instance().ticker_1khz;
    auto &led1 = Peripherals::get_instance().led1;
    auto &led2 = Peripherals::get_instance().led2;
    auto &tp = Peripherals::get_instance().tp;

    imu_fusion.begin();

    ticker_1khz.attach([&]() {
        // tp = 1;
        imu_fusion.update();
        Vector3f gyro = imu_fusion.get_gyro_fusion();
        // Vector3f gyro = imu_fusion.get_gyro(0);
        // tp = 0;
        // printf("%.5f,%.5f,%.5f\n", gyro.x, gyro.y, gyro.z);
        printf("%.5f\n", gyro.z);
    });

    while (1) {
        //        led1 = 1;
        //        led2 = 0;
        //        HAL_Delay(500);
        //        led1 = 0;
        //        led2 = 1;
        //        HAL_Delay(500);
    }
}

void imu_fusion_angle_check_test() {
    Peripherals::enable_std_printf();
    printf_test_func();

    auto &imu_fusion = Peripherals::get_instance().imu_fusion;
    auto &ticker_1khz = Peripherals::get_instance().ticker_1khz;
    auto &led1 = Peripherals::get_instance().led1;
    auto &led2 = Peripherals::get_instance().led2;
    auto &tp = Peripherals::get_instance().tp;

    imu_fusion.begin();

    float angle = 0;
    float prev_gyro = 0;

    for (size_t i = 0; i < 100; i++) {
        imu_fusion.update();
        HAL_Delay(1);
    }

    const float gyro_z_bias = 0.000575;

    ticker_1khz.attach([&]() {
        // tp = 1;
        imu_fusion.update();
        Vector3f gyro = imu_fusion.get_gyro_fusion();

        float bgyro = gyro.z - gyro_z_bias;
        angle += (prev_gyro + bgyro) * 0.5f * 0.001f;
        prev_gyro = bgyro;

        printf("%.5f\n", angle);
    });

    while (1) {
        //        led1 = 1;
        //        led2 = 0;
        //        HAL_Delay(500);
        //        led1 = 0;
        //        led2 = 1;
        //        HAL_Delay(500);
    }
}

void imu_gyro_bias_check() {
    Peripherals::enable_std_printf();
    printf_test_func();

    auto &imu = Peripherals::get_instance().imu;
    auto &ticker_1khz = Peripherals::get_instance().ticker_1khz;

    std::array<Vector3f, 32> biases;
    const float gain = 0.001;

    imu.begin();
    imu.enable_acc();
    imu.enable_gyro();

    for (size_t i = 0; i < 100; i++) {
        imu.update_acc_axes();
        imu.update_gyro_axes();
        HAL_Delay(1);
    }

    while (1) {
        imu.update_acc_axes();
        imu.update_gyro_axes();

        for (size_t i = 0; i < 32; i++) {
            biases[i] += (imu.get_gyro_axes_raw(i) - biases[i]) * gain;
            if (i != 31) {
                printf("%.8f,%.8f,%.8f,", biases[i].x, biases[i].y, biases[i].z);
            } else {
                printf("%.8f,%.8f,%.8f\n", biases[i].x, biases[i].y, biases[i].z);
            }
        }
    }
}

} // namespace unit_test
} // namespace imu_module
