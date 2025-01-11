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

    std::array<uint8_t, LSM6DSRArray::NUM_SENSOR> result;
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

// void imu_read_raw_test() {
//     Peripherals::enable_std_printf();
//     printf_test_func();
//
//     auto &imu = Peripherals::get_instance().imu;
//
//     imu.begin();
//     imu.enable_acc();
//     imu.enable_gyro();
//
//     while (1) {
//         imu.update_acc_axes();
//         imu.update_gyro_axes();
//         auto acc = imu.get_acc_axes(0);
//         auto gyro = imu.get_gyro_axes(0);
//
//         // printf("acc: %.3f %.3f %.3f\n", acc.x(), acc.y(), acc.z());
//         // printf("gyro: %.3f %.3f %.3f\n", gyro.x(), gyro.y(), gyro.z());
//
//         printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", acc.x(), acc.y(), acc.z(), gyro.x(), gyro.y(), gyro.z());
//
//         HAL_Delay(10);
//     }
// }

void imu_read_raw_gyro_all_test() {
    Peripherals::enable_std_printf();
    printf_test_func();

    auto &imu_fusion = Peripherals::get_instance().imu_fusion;

    imu_fusion.begin();

    while (1) {
        imu_fusion.update();

        for (size_t i = 0; i < 32; i++) {
            auto gyro = imu_fusion.get_gyro(i);

            char r = (i == 31) ? '\n' : ',';
            printf("%.3f%c", gyro.z(), r);
        }

        HAL_Delay(10);
    }
}

// void imu_read_raw_acc_all_test() {
//     Peripherals::enable_std_printf();
//     printf_test_func();
//
//     auto &imu_fusion = Peripherals::get_instance().imu_fusion;
//
//     imu_fusion.begin();
//
//     while (1) {
//         imu_fusion.update();
//
//         for (size_t i = 0; i < 32; i++) {
//             auto gyro = imu_fusion.get_acc(i);
//
//             char r = (i == 31) ? '\n' : ',';
//             printf("%.3f%c", gyro.z(), r);
//         }
//
//         HAL_Delay(10);
//     }
// }
//
// void imu_read_temperature_raw_test() {
//     Peripherals::enable_std_printf();
//     printf_test_func();
//
//     auto &imu = Peripherals::get_instance().imu;
//
//     imu.begin();
//     imu.enable_acc();
//     imu.enable_gyro();
//
//     while (1) {
//         // printf("start update\n");
//         imu.update_temperature();
//
//         // printf("start printf\n");
//         for (size_t i = 0; i < 32; i++) {
//             if (i == 31) {
//                 printf("%4.1f\n", imu.get_temperature(i));
//             } else {
//                 printf("%4.1f,", imu.get_temperature(i));
//             }
//         }
//
//         HAL_Delay(100);
//     }
// }
//
// void imu_axes_check_test() {
//     Peripherals::enable_std_printf();
//     printf_test_func();
//
//     auto &imu_fusion = Peripherals::get_instance().imu_fusion;
//
//     imu_fusion.begin();
//
//     while (1) {
//         imu_fusion.update();
//
//         for (size_t i = 0; i < IMUFusion::NUM_SENSOR; i++) {
//             printf("%.3f", imu_fusion.get_gyro(i).x());
//             if (i == IMUFusion::NUM_SENSOR - 1)
//                 printf("\n");
//             else
//                 printf(",");
//         }
//
//         // HAL_Delay(1);
//     }
// }
//
void imu_gyro_fusion_test() {
    Peripherals::enable_std_printf();
    printf_test_func();

    auto &imu_fusion = Peripherals::get_instance().imu_fusion;
    auto &main_ticker = Peripherals::get_instance().main_ticker;
    auto &led1 = Peripherals::get_instance().led1;
    auto &led2 = Peripherals::get_instance().led2;

    imu_fusion.begin();

    constexpr float earth_rotation_rate_rad_per_sec = 360.0f / 86400.0f * M_PI / 180.0f;

    main_ticker.attach([&]() {
        imu_fusion.update();
        Vector3f gyro = imu_fusion.get_gyro_fusion();
        printf("%.3f,%.3f,%.3f\r\n", gyro.x() * 100, gyro.y() * 100, gyro.z() * 100);
        Vector3f gyro_raw = imu_fusion.get_gyro(0);

        // printf("%.3f,%.3f\n", gyro_raw.x() * 100, gyro.x() * 100);
        // printf("%.8f\n", gyro.x());
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
//
// void imu_acc_fusion_test() {
//    Peripherals::enable_std_printf();
//    printf_test_func();
//
//    auto &imu_fusion = Peripherals::get_instance().imu_fusion;
//    auto &main_ticker = Peripherals::get_instance().main_ticker;
//    auto &led1 = Peripherals::get_instance().led1;
//    auto &led2 = Peripherals::get_instance().led2;
//
//    imu_fusion.begin();
//
//    main_ticker.attach([&]() {
//        imu_fusion.update();
//        Vector3f acc = imu_fusion.get_acc_fusion();
//        // Vector3f gyro = imu_fusion.get_gyro(0);
//        printf("%.5f,%.5f,%.5f\n", acc.x(), acc.y(), acc.z());
//        // printf("%.5f\n", acc.x());
//    });
//
//    while (1) {
//        led1 = 1;
//        led2 = 0;
//        HAL_Delay(500);
//        led1 = 0;
//        led2 = 1;
//        HAL_Delay(500);
//    }
//}

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

// void imu_fusion_acc_integrate_check_test() {
//     Peripherals::enable_std_printf();
//     printf_test_func();

//     auto &imu_fusion = Peripherals::get_instance().imu_fusion;
//     auto &main_ticker = Peripherals::get_instance().main_ticker;
//     const float Ts = Peripherals::get_instance().ticker_period;
//     auto &led1 = Peripherals::get_instance().led1;
//     auto &led2 = Peripherals::get_instance().led2;

//     imu_fusion.begin();

//     float vel = 0;
//     float prev_acc = 0;

//     for (size_t i = 0; i < 100; i++) {
//         imu_fusion.update();
//         HAL_Delay(1);
//     }

//     main_ticker.attach([&]() {
//         imu_fusion.update();
//         Vector3f acc = imu_fusion.get_acc_fusion();

//         float bacc = acc.x() + 0.0f;
//         vel += (prev_acc + bacc) * 0.5f * Ts;
//         prev_acc = bacc;

//         printf("%.5f\n", vel);
//     });

//     while (1) {
//         led1 = 1;
//         led2 = 0;
//         HAL_Delay(500);
//         led1 = 0;
//         led2 = 1;
//         HAL_Delay(500);
//     }
// }

// void imu_calibrated_gyro_bias_check() {
//     Peripherals::enable_std_printf();
//     printf_test_func();
//
//     auto &imu = Peripherals::get_instance().imu_fusion;
//     imu.begin();
//
//     HAL_Delay(200);
//
//     while (1) {
//         imu.update();
//
//         for (size_t i = 0; i < 32; i++) {
//             Vector3f gyro = imu.get_gyro(i);
//             if (i != 31) {
//                 printf("%.8f,%.8f,%.8f,", gyro.x(), gyro.y(), gyro.z());
//             } else {
//                 printf("%.8f,%.8f,%.8f\n", gyro.x(), gyro.y(), gyro.z());
//             }
//         }
//     }
// }

// void imu_calibrated_acc_bias_check() {
//     Peripherals::enable_std_printf();
//     printf_test_func();
//
//     auto &imu = Peripherals::get_instance().imu_fusion;
//     imu.begin();
//
//     HAL_Delay(200);
//
//     while (1) {
//         imu.update();
//
//         for (size_t i = 0; i < 32; i++) {
//             Vector3f gyro = imu.get_acc(i);
//             if (i != 31) {
//                 printf("%.8f,%.8f,%.8f,", gyro.x(), gyro.y(), gyro.z());
//             } else {
//                 printf("%.8f,%.8f,%.8f\n", gyro.x(), gyro.y(), gyro.z());
//             }
//         }
//     }
// }

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

// #include <Eigen/Geometry>
// Eigen::Quaternionf integrateAngularVelocity(const Eigen::Quaternionf &q, const Eigen::Vector3f &omega, float dt) {
//     Eigen::Quaternionf dq;
//     Eigen::Vector3f half_omega_dt = 0.5f * omega * dt;
//     dq.w() = 1.0f;
//     dq.vec() = half_omega_dt;

//     Eigen::Quaternionf q_new = q * dq;
//     q_new.normalize(); // クォータニオンの正規化

//     return q_new;
// }

// void imu_angle_fusion_check() {
//     Peripherals::enable_std_printf();
//     printf_test_func();

//     auto &imu_fusion = Peripherals::get_instance().imu_fusion;
//     auto &main_ticker = Peripherals::get_instance().main_ticker;
//     const float Ts = Peripherals::get_instance().ticker_period;

//     imu_fusion.begin();

//     Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
//     Eigen::Vector3f omega, prev_omega, acc_rot;
//     omega.setZero();

//     HAL_Delay(500);

//     // 静止状態で姿勢推定
//     for (size_t i = 0; i < 500; i++) {
//         imu_fusion.update();
//         HAL_Delay(1);
//     }

//     main_ticker.attach([&]() {
//         imu_fusion.update();
//         prev_omega = omega;
//         omega = imu_fusion.get_gyro_fusion();
//         q = integrateAngularVelocity(q, (prev_omega + omega) * 0.5f, Ts);

//         // Eigen::Vector3f rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
//     });

//     while (1) {
//         printf("%.5f,%.5f,%.5f,%.5f\r\n", q.w(), q.x(), q.y(), q.z());
//         HAL_Delay(30);
//     }
// }

// void imu_filtered_acc_check() {
//     Peripherals::enable_std_printf();
//     printf_test_func();
//
//     auto &imu_fusion = Peripherals::get_instance().imu_fusion;
//     auto &main_ticker = Peripherals::get_instance().main_ticker;
//     const float Ts = Peripherals::get_instance().ticker_period;
//
//     imu_fusion.begin();
//
//     HAL_Delay(500);
//
//     // 静止状態で姿勢推定
//     for (size_t i = 0; i < 500; i++) {
//         imu_fusion.update();
//         HAL_Delay(5);
//     }
//     imu_fusion.reset_filter();
//
//     main_ticker.attach([&]() {
//         imu_fusion.update();
//         Eigen::Vector3f acc = imu_fusion.get_acc_fusion();
//         Eigen::Vector3f facc = imu_fusion.get_filtered_acc();
//         printf("%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n", acc.x(), acc.y(), acc.z(), facc.x(), facc.y(), facc.z());
//     });
//
//     while (1) {
//     }
// }

// void imu_fusion_weights_check() {
//     Peripherals::enable_std_printf();
//     printf_test_func();
//
//     auto &imu_fusion = Peripherals::get_instance().imu_fusion;
//
//     imu_fusion.begin();
//
//     HAL_Delay(500);
//
//     while (1) {
//         imu_fusion.update();
//         auto weights = imu_fusion.fusion_acc_weights();
//
//         for (size_t i = 0; i < weights.size(); i++) {
//             char r = (i == weights.size() - 1) ? '\n' : ',';
//             printf("%.5f%c", weights[i].z(), r);
//         }
//     }
// }

// void imu_vel_fusion_check() {
//     Peripherals::enable_std_printf();
//     printf_test_func();
//
//     auto &imu_fusion = Peripherals::get_instance().imu_fusion;
//     auto &main_ticker = Peripherals::get_instance().main_ticker;
//     const float Ts = Peripherals::get_instance().ticker_period;
//
//     imu_fusion.begin();
//
//     float vel = 0;
//     float prev_acc = 0;
//
//     Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
//     Eigen::Vector3f vel_vec = Eigen::Vector3f::Zero();
//     Eigen::Vector3f prev_world_acc_vec = Eigen::Vector3f::Zero();
//
//     HAL_Delay(500);
//
//     // 静止状態で姿勢推定
//     for (size_t i = 0; i < 500; i++) {
//         imu_fusion.update();
//         HAL_Delay(5);
//     }
//     imu_fusion.reset_filter();
//     HAL_Delay(5);
//
//     Eigen::Vector3f gra(0, 0, 0);
//     for (size_t i = 0; i < 100; i++) {
//         imu_fusion.update();
//         gra += imu_fusion.get_acc_fusion() / 100.0f;
//         HAL_Delay(1);
//     }
//
//     printf("attach2\n");
//
//     main_ticker.attach([&]() {
//         imu_fusion.update();
//
//         Eigen::Quaternionf q = imu_fusion.get_quaternion();
//         Eigen::Vector3f acc = imu_fusion.get_acc_fusion();
//
//         Eigen::Vector3f world_acc = q * acc;
//         world_acc -= gra;
//         vel_vec += (prev_world_acc_vec + world_acc) * 0.5f * Ts;
//         prev_world_acc_vec = world_acc;
//
//         printf("%.5f,%.5f,%.5f\n", vel_vec.x(), vel_vec.y(), vel_vec.z());
//     });
//
//     auto &led1 = Peripherals::get_instance().led1;
//     auto &led2 = Peripherals::get_instance().led2;
//
//     while (1) {
//         led1 = 1;
//         led2 = 0;
//         HAL_Delay(500);
//         led1 = 0;
//         led2 = 1;
//         HAL_Delay(500);
//     }
// }
//
// void imu_filter_fusion_check() {
//     Peripherals::enable_std_printf();
//     printf_test_func();
//
//     auto &imu_fusion = Peripherals::get_instance().imu_fusion;
//     auto &main_ticker = Peripherals::get_instance().main_ticker;
//     auto &led1 = Peripherals::get_instance().led1;
//     auto &led2 = Peripherals::get_instance().led2;
//
//     imu_fusion.begin();
//
//     for (size_t i = 0; i < 500; i++) {
//         imu_fusion.update();
//         HAL_Delay(10);
//     }
//
//     imu_fusion.reset_filter();
//
//     int cnt = 0;
//     Eigen::Vector3f world_vel = Eigen::Vector3f::Zero();
//
//     bool is_error = false;
//     main_ticker.attach([&]() {
//         if (is_error)
//             return;
//
//         imu_fusion.update();
//
//         // Eigen::Vector3f acc = imu_fusion.get_filtered_acc();
//         // printf("%.5f,%.5f,%.5f\n", acc.x(), acc.y(), acc.z());
//
//         // world_vel += world_acc * 0.001f;
//         // printf("%.5f,%.5f,%.5f\n", world_vel.x(), world_vel.y(), world_vel.z());
//
//         // Eigen::Quaternionf q = imu_fusion.get_quaternion();
//         // printf("%.5f,%.5f,%.5f,%.5f\n", q.w(), q.x(), q.y(), q.z());
//
//         // if (std::isnan(q.w()) || std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z())) {
//         //     is_error = true;
//         //     printf("Error\n");
//         // }
//
//         if (cnt == 50) {
//             cnt = 0;
//
//             // printf("0\n");
//             Eigen::Quaternionf q = imu_fusion.get_quaternion();
//             printf("%.6f,%.6f,%.6f,%.6f\n", q.w(), q.x(), q.y(), q.z());
//
//             // Eigen::Vector3f rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
//             // printf("%.6f,%.6f,%.6f\n", rpy.x(), rpy.y(), rpy.z());
//
//             // Eigen::Vector3f linacc = imu_fusion.get_filtered_acc();
//             // printf("%.6f,%.6f,%.6f\n", linacc.x(), linacc.y(), linacc.z());
//
//             // Eigen::Vector3f fgyro = imu_fusion.get_filtered_gyro();
//             // printf("%.6f,%.6f,%.6f\n", fgyro.x(), fgyro.y(), fgyro.z());
//
//             // Eigen::Vector3f goffset = imu_fusion.filter().test_goffset_;
//             // printf("%.6f,%.6f,%.6f\n", goffset.x(), goffset.y(), goffset.z());
//
//             // Eigen::Vector3f acc = imu_fusion.get_acc_fusion();
//             // Eigen::Vector3f g1 = imu_fusion.filter().test_g1_;
//             // Eigen::Vector3f g2 = imu_fusion.filter().test_g2_;
//
//             // printf("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", acc.x(), acc.y(), acc.z(), g1.x(), g1.y(),
//             // g1.z(),
//             //        g2.x(), g2.y(), g2.z());
//         }
//         cnt++;
//     });
//
//     while (1) {
//         led1 = 1;
//         led2 = 0;
//         HAL_Delay(500);
//         led1 = 0;
//         led2 = 1;
//         HAL_Delay(500);
//     }
// }

// Eigen::Matrix3f computeRotationMatrix(const Eigen::Vector3f &a, const Eigen::Vector3f &b) {
//     // ベクトルを正規化
//     Eigen::Vector3f a_normalized = a.normalized();
//     Eigen::Vector3f b_normalized = b.normalized();
//
//     // 回転軸 (外積)
//     Eigen::Vector3f axis = a_normalized.cross(b_normalized);
//
//     // 回転角 (内積)
//     float cosTheta = a_normalized.dot(b_normalized); // cos(θ)
//     float sinTheta = axis.norm();                    // sin(θ)
//
//     // 回転軸を正規化
//     if (sinTheta > 1e-6) { // 0除算を避けるためにチェック
//         axis.normalize();
//     } else {
//         // a と b が平行の場合、回転なしまたは180度回転
//         if (cosTheta > 0) {
//             return Eigen::Matrix3f::Identity(); // 回転なし
//         } else {
//             // 180度回転（任意の直交ベクトルを回転軸とする）
//             Eigen::Vector3f ortho = (a_normalized.unitOrthogonal());
//             Eigen::Matrix3f R = Eigen::AngleAxisf(M_PI, ortho).toRotationMatrix();
//             return R;
//         }
//     }
//
//     // 回転行列の構築 (ロドリゲスの回転公式)
//     Eigen::Matrix3f K;
//     K << 0, -axis.z(), axis.y(), axis.z(), 0, -axis.x(), -axis.y(), axis.x(), 0;
//
//     printf("sinTheta: %.6f\n", sinTheta);
//     printf("cosTheta: %.6f\n", cosTheta);
//
//     Eigen::Matrix3f rotationMatrix = Eigen::Matrix3f::Identity() + sinTheta * K + (1 - cosTheta) * K * K;
//
//     return rotationMatrix;
// }
//
// void imu_filter_fusion_check2() {
//     Peripherals::enable_std_printf();
//     printf_test_func();
//
//     auto &imu_fusion = Peripherals::get_instance().imu_fusion;
//     auto &main_ticker = Peripherals::get_instance().main_ticker;
//     auto &led1 = Peripherals::get_instance().led1;
//     auto &led2 = Peripherals::get_instance().led2;
//
//     iekf::InvariantExtendedKalmanFilter ekf(Peripherals::get_instance().ticker_period);
//
//     imu_fusion.begin();
//
//     for (size_t i = 0; i < 100; i++) {
//         imu_fusion.update();
//         HAL_Delay(10);
//     }
//
//     Eigen::Vector3f avg_acc = Eigen::Vector3f::Zero();
//     for (size_t i = 0; i < 500; i++) {
//         imu_fusion.update();
//         HAL_Delay(10);
//
//         Eigen::Vector3f acc = imu_fusion.get_acc_fusion();
//         printf("%.6f,%.6f,%.6f\n", acc.x(), acc.y(), acc.z());
//
//         // if (acc.squaredNorm() < 1.1f) {
//         avg_acc += acc / 500.0f;
//         // }
//     }
//
//     avg_acc.normalize();
//
//     printf("avg_acc\n");
//     printf("%.6f,%.6f,%.6f\n", avg_acc.x(), avg_acc.y(), avg_acc.z());
//
//     Eigen::Matrix3f R_init = computeRotationMatrix(avg_acc, Eigen::Vector3f(0, 0, 1));
//
//     printf("R_init\n");
//     printf("%.6f,%.6f,%.6f\n", R_init(0, 0), R_init(0, 1), R_init(0, 2));
//     printf("%.6f,%.6f,%.6f\n", R_init(1, 0), R_init(1, 1), R_init(1, 2));
//     printf("%.6f,%.6f,%.6f\n", R_init(2, 0), R_init(2, 1), R_init(2, 2));
//
//     // while (1);
//
//     ekf.initialize(R_init, Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(),
//                    Eigen::Vector3f::Zero());
//
//     imu_fusion.reset_filter();
//
//     int cnt = 0;
//     Eigen::Vector3f world_vel = Eigen::Vector3f::Zero();
//
//     bool is_error = false;
//     main_ticker.attach([&]() {
//         if (is_error)
//             return;
//
//         imu_fusion.update();
//         ekf.propagate(imu_fusion.get_gyro_fusion(), imu_fusion.get_acc_fusion());
//
//         if (cnt == 50) {
//             cnt = 0;
//
//             // printf("0\n");
//             // Eigen::Quaternionf q(ekf.getRotation());
//             // printf("%.6f,%.6f,%.6f,%.6f\n", q.w(), q.x(), q.y(), q.z());
//
//             // Eigen::Vector3f vec = ekf.getVelocity();
//             Eigen::Vector3f vec = ekf.getPosition();
//             // Eigen::Vector3f vec = imu_fusion.get_acc_fusion() + ekf.getGravity();
//             printf("%.6f,%.6f,%.6f\n", vec.x(), vec.y(), vec.z());
//         }
//         cnt++;
//     });
//
//     while (1) {
//         led1 = 1;
//         led2 = 0;
//         HAL_Delay(500);
//         led1 = 0;
//         led2 = 1;
//         HAL_Delay(500);
//     }
// }

} // namespace unit_test
} // namespace imu_module
