#include "imu_module/board.hpp"
#include "imu_module/fpu_dsp_test.hpp"
#include "imu_module/unit_test.hpp"
#include "main.h"

void all_tests() {
    // fpu_dsp_test::float_mul_add_time_test();
    // fpu_dsp_test::float_div_time_test();
    // fpu_dsp_test::float_sqrtf_time_test();
    // fpu_dsp_test::float_sin_add_time_test();
    // fpu_dsp_test::float_sinf_add_time_test();
    // fpu_dsp_test::float_arm_math_sinf_add_time_test();

    // fpu_dsp_test::vector_eigen_add_time_test();
    // fpu_dsp_test::vector_cmsis_dsp_add_time_test();
    // fpu_dsp_test::vector_raw_add_time_test();

    // fpu_dsp_test::matrix_eigen_mul_time_test();
    // fpu_dsp_test::matrix_cmsis_dsp_mul_time_test();

    // unit_test::led_test();
    // unit_test::vcp_test();
    // unit_test::ticker_test();
    // unit_test::imu_whoami_test();
    // unit_test::imu_read_raw_test();
    // unit_test::imu_axes_check_test();
    // unit_test::imu_rot_check();
    // unit_test::imu_gyro_fusion_test();
    // unit_test::imu_acc_fusion_test();

    // unit_test::imu_fusion_gyro_integrate_check_test();
    // unit_test::imu_fusion_acc_integrate_check_test();

    // unit_test::imu_gyro_bias_check();
    // unit_test::imu_acc_bias_check();
    // unit_test::imu_acc_raw_variance_check();

    // unit_test::imu_angle_fusion_check();
    // unit_test::imu_vel_fusion_check();

    // unit_test::spi_nss_it_test();
    // unit_test::spi_slave_dma_test();
}

extern "C" void cppmain() {
    using namespace imu_module;

    // all_tests();

    auto &led1 = Peripherals::get_instance().led1;
    auto &led2 = Peripherals::get_instance().led2;
    auto &spi_slave = Peripherals::get_instance().spi_slave;
    auto &nss = Peripherals::get_instance().spi_cs;

    auto &imu_fusion = Peripherals::get_instance().imu_fusion;
    auto &ticker_1khz = Peripherals::get_instance().ticker_1khz;

    imu_fusion.begin();

    uint8_t tx_buf[24] = {0};
    uint8_t rx_buf[24] = {0};

    Vector3f latest_gyro_data, latest_acc_data;

    // IMU data update
    ticker_1khz.attach([&]() {
        imu_fusion.update();
        __disable_irq();
        latest_gyro_data = imu_fusion.get_gyro_fusion();
        latest_acc_data = imu_fusion.get_acc_fusion();
        __enable_irq();
    });

    // SPI data update on NSS rising edge
    nss.attach([&]() {
        std::memcpy(tx_buf, latest_gyro_data.data(), sizeof(float) * 3);
        std::memcpy(tx_buf + sizeof(float) * 3, latest_acc_data.data(), sizeof(float) * 3);
        spi_slave.write_read(tx_buf, rx_buf, 12, 10);
    });

    while (1) {
        led1 = !led1;
        HAL_Delay(500);
    }
}
