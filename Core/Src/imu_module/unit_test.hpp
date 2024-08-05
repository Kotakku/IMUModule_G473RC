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

void ticker_test() {
    Peripherals::enable_std_printf();
    printf_test_func();

    auto &main_ticker = Peripherals::get_instance().main_ticker;
    auto &led1 = Peripherals::get_instance().led1;

    main_ticker.attach([&]() { led1 = !led1; });

    while (1)
        ;
}

void spi_slave_dma_test() {
    Peripherals::enable_std_printf();
    printf_test_func();
    printf("Hello, world!\n");

    auto &led1 = Peripherals::get_instance().led1;
    auto &spi_slave = Peripherals::get_instance().spi_slave;

    uint8_t tx_buf[6] = {0, 33, 44, 55, 66, 77};
    uint8_t rx_buf[6];

    spi_slave.on_dma_complete([&]() {
        spi_slave.write_read(tx_buf, rx_buf, 6, 10);
        // printf("nss\n");
    });
    spi_slave.write_read(tx_buf, rx_buf, 6, 10);

    printf("is_dma_enabled: %d\n", spi_slave.is_dma_enabled());

    while (1) {
    	led1 = !led1;
        tx_buf[0]++;

        HAL_Delay(1);
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
        auto acc = imu.get_acc_axes(0);
        auto gyro = imu.get_gyro_axes(0);

        printf("acc: %.3f %.3f %.3f\n", acc.x(), acc.y(), acc.z());
        printf("gyro: %.3f %.3f %.3f\n", gyro.x(), gyro.y(), gyro.z());

        HAL_Delay(1000);
    }
}

void imu_axes_check_test() {
    Peripherals::enable_std_printf();
    printf_test_func();

    auto &imu_fusion = Peripherals::get_instance().imu_fusion;

    imu_fusion.begin();

    while (1) {
        imu_fusion.update();

        for (size_t i = 0; i < IMUFusion::NUM_SENSOR; i++) {
            printf("%.3f", imu_fusion.get_gyro(i).x());
            if (i == IMUFusion::NUM_SENSOR - 1)
                printf("\n");
            else
                printf(",");
        }

        // HAL_Delay(1);
    }
}

void imu_gyro_fusion_test() {
    Peripherals::enable_std_printf();
    printf_test_func();

    auto &imu_fusion = Peripherals::get_instance().imu_fusion;
    auto &main_ticker = Peripherals::get_instance().main_ticker;
    auto &led1 = Peripherals::get_instance().led1;
    auto &led2 = Peripherals::get_instance().led2;

    imu_fusion.begin();

    main_ticker.attach([&]() {
        imu_fusion.update();
        Vector3f gyro = imu_fusion.get_gyro_fusion();
        printf("%.3f,%.3f,%.3f\n", gyro.x(), gyro.y(), gyro.z());
    });

    while (1) {
        led1 = 1;
        led2 = 0;
        HAL_Delay(500);
        led1 = 0;
        led2 = 1;
        HAL_Delay(500);
    }
}

void imu_acc_fusion_test() {
    Peripherals::enable_std_printf();
    printf_test_func();

    auto &imu_fusion = Peripherals::get_instance().imu_fusion;
    auto &main_ticker = Peripherals::get_instance().main_ticker;
    auto &led1 = Peripherals::get_instance().led1;
    auto &led2 = Peripherals::get_instance().led2;

    imu_fusion.begin();

    main_ticker.attach([&]() {
        imu_fusion.update();
        Vector3f acc = imu_fusion.get_acc_fusion();
        // Vector3f gyro = imu_fusion.get_gyro(0);
        // printf("%.5f,%.5f,%.5f\n", gyro.x, gyro.y, gyro.z);
        printf("%.5f\n", acc.x());
    });

    while (1) {
        led1 = 1;
        led2 = 0;
        HAL_Delay(500);
        led1 = 0;
        led2 = 1;
        HAL_Delay(500);
    }
}

void imu_fusion_gyro_integrate_check_test() {
    Peripherals::enable_std_printf();
    printf_test_func();

    auto &imu_fusion = Peripherals::get_instance().imu_fusion;
    auto &main_ticker = Peripherals::get_instance().main_ticker;
    const float Ts = Peripherals::get_instance().ticker_period;
    auto &led1 = Peripherals::get_instance().led1;
    auto &led2 = Peripherals::get_instance().led2;

    imu_fusion.begin();

    float angle = 0;
    float prev_gyro = 0;

    for (size_t i = 0; i < 100; i++) {
        imu_fusion.update();
        HAL_Delay(1);
    }

    main_ticker.attach([&]() {
        imu_fusion.update();
        Vector3f gyro = imu_fusion.get_gyro_fusion();

        angle += (prev_gyro + gyro.z()) * 0.5f * Ts;
        prev_gyro = gyro.z();

        printf("%.5f\n", angle);
    });

    while (1) {
        led1 = 1;
        led2 = 0;
        HAL_Delay(500);
        led1 = 0;
        led2 = 1;
        HAL_Delay(500);
    }
}

void imu_fusion_acc_integrate_check_test() {
    Peripherals::enable_std_printf();
    printf_test_func();

    auto &imu_fusion = Peripherals::get_instance().imu_fusion;
    auto &main_ticker = Peripherals::get_instance().main_ticker;
    const float Ts = Peripherals::get_instance().ticker_period;
    auto &led1 = Peripherals::get_instance().led1;
    auto &led2 = Peripherals::get_instance().led2;

    imu_fusion.begin();

    float vel = 0;
    float prev_acc = 0;

    for (size_t i = 0; i < 100; i++) {
        imu_fusion.update();
        HAL_Delay(1);
    }

    main_ticker.attach([&]() {
        imu_fusion.update();
        Vector3f acc = imu_fusion.get_acc_fusion();

        float bacc = acc.x() + 0.0f;
        vel += (prev_acc + bacc) * 0.5f * Ts;
        prev_acc = bacc;

        printf("%.5f\n", vel);
    });

    while (1) {
        led1 = 1;
        led2 = 0;
        HAL_Delay(500);
        led1 = 0;
        led2 = 1;
        HAL_Delay(500);
    }
}

void imu_calibrated_gyro_bias_check() {
    Peripherals::enable_std_printf();
    printf_test_func();

    auto &imu = Peripherals::get_instance().imu_fusion;
    imu.begin();

    HAL_Delay(200);

    while (1) {
        imu.update();

        for (size_t i = 0; i < 32; i++) {
            Vector3f gyro = imu.get_gyro(i);
            if (i != 31) {
                printf("%.8f,%.8f,%.8f,", gyro.x(), gyro.y(), gyro.z());
            } else {
                printf("%.8f,%.8f,%.8f\n", gyro.x(), gyro.y(), gyro.z());
            }
        }
    }
}

void imu_calibrated_acc_bias_check() {
    Peripherals::enable_std_printf();
    printf_test_func();

    auto &imu = Peripherals::get_instance().imu_fusion;
    imu.begin();

    HAL_Delay(200);

    while (1) {
        imu.update();

        for (size_t i = 0; i < 32; i++) {
            Vector3f gyro = imu.get_acc(i);
            if (i != 31) {
                printf("%.8f,%.8f,%.8f,", gyro.x(), gyro.y(), gyro.z());
            } else {
                printf("%.8f,%.8f,%.8f\n", gyro.x(), gyro.y(), gyro.z());
            }
        }
    }
}

// void imu_acc_raw_variance_check() {
//     Peripherals::enable_std_printf();
//     printf_test_func();

//     auto &imu = Peripherals::get_instance().imu;
//     // auto &main_ticker = Peripherals::get_instance().main_ticker;

//     imu.begin();
//     imu.enable_acc();
//     imu.enable_gyro();

//     for (size_t i = 0; i < 100; i++) {
//         imu.update_acc_axes();
//         imu.update_gyro_axes();
//         HAL_Delay(1);
//     }

//     while (1) {
//         imu.update_acc_axes();
//         // imu.update_gyro_axes();

//         Vector3f acc = imu.get_acc_axes(0);
//         printf("%.8f,%.8f,%.8f\n", acc.x(), acc.y(), acc.z());
//     }
// }

#include <Eigen/Geometry>
Eigen::Quaternionf integrateAngularVelocity(const Eigen::Quaternionf &q, const Eigen::Vector3f &omega, float dt) {
    Eigen::Quaternionf dq;
    Eigen::Vector3f half_omega_dt = 0.5f * omega * dt;
    dq.w() = 1.0f;
    dq.vec() = half_omega_dt;

    Eigen::Quaternionf q_new = q * dq;
    q_new.normalize(); // クォータニオンの正規化

    return q_new;
}

void imu_angle_fusion_check() {
    Peripherals::enable_std_printf();
    printf_test_func();

    auto &imu_fusion = Peripherals::get_instance().imu_fusion;
    auto &main_ticker = Peripherals::get_instance().main_ticker;
    const float Ts = Peripherals::get_instance().ticker_period;

    imu_fusion.begin();

    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    Eigen::Vector3f omega, prev_omega, acc_rot;
    omega.setZero();

    HAL_Delay(500);

    // 静止状態で姿勢推定
    for (size_t i = 0; i < 500; i++) {
        imu_fusion.update();
        HAL_Delay(1);
    }

    main_ticker.attach([&]() {
        imu_fusion.update();
        prev_omega = omega;
        omega = imu_fusion.get_gyro_fusion();
        q = integrateAngularVelocity(q, (prev_omega + omega) * 0.5f, Ts);

        // Eigen::Vector3f rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
    });

    while (1) {
        printf("%.5f,%.5f,%.5f,%.5f\r\n", q.w(), q.x(), q.y(), q.z());
        HAL_Delay(30);
    }
}

void imu_vel_fusion_check() {
    Peripherals::enable_std_printf();
    printf_test_func();

    auto &imu_fusion = Peripherals::get_instance().imu_fusion;
    auto &main_ticker = Peripherals::get_instance().main_ticker;

    imu_fusion.begin();

    float vel = 0;
    float prev_acc = 0;

    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    Eigen::Vector3f omega, acc_rot;

    HAL_Delay(500);

    // 静止状態で姿勢推定
    for (size_t i = 0; i < 500; i++) {
        imu_fusion.update();
        HAL_Delay(1);
    }

    main_ticker.attach([&]() {
        imu_fusion.update();
        omega = imu_fusion.get_gyro_fusion();
        q = integrateAngularVelocity(q, omega, 0.001f);

        acc_rot = imu_fusion.get_acc_fusion();
        acc_rot = q * acc_rot;

        float bacc = acc_rot.y();
        vel += (prev_acc + bacc) * 0.5f * 0.001f;
        prev_acc = bacc;

        if (abs(acc_rot.y()) < 0.05f && abs(vel) < 0.05f) {
            vel = 0;
        }
        // pos += vel * 0.001f;

        printf("%.5f\n", vel);
    });

    while (1) {
    }
}


} // namespace unit_test
} // namespace imu_module
