#include "imu_module/board.hpp"
#include "imu_module/fpu_dsp_test.hpp"
#include "imu_module/unit_test.hpp"
#include "main.h"

void all_tests() {
	using namespace imu_module;

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
    // Peripherals::enable_std_printf();

    auto &led1 = Peripherals::get_instance().led1;
    // auto &led2 = Peripherals::get_instance().led2;
    auto &spi_slave = Peripherals::get_instance().spi_slave;

    auto &imu_fusion = Peripherals::get_instance().imu_fusion;
    auto &main_ticker = Peripherals::get_instance().main_ticker;

    imu_fusion.begin();

    uint8_t tx_buf[12] = {0};
    uint8_t rx_buf[12] = {0};

    Vector3f latest_gyro_data, latest_acc_data;

    // IMU data update
    main_ticker.attach([&]() {
        imu_fusion.update();
        latest_gyro_data = imu_fusion.get_gyro_fusion();
        latest_acc_data = imu_fusion.get_acc_fusion();
    });

    // SPI buffer update & start next transfer on DMA complete
    spi_slave.on_dma_complete([&](){
        for(size_t i = 0; i < 3; i++) {
            float val = latest_gyro_data[i] / LSM6DSR_GYRO_SENSITIVITY_FS_1000DPS;
            int16_t val_int = static_cast<int16_t>(std::roundf(val));
            tx_buf[i * 2] = val_int & 0xFF;
            tx_buf[i * 2 + 1] = (val_int >> 8) & 0xFF;
        }

        for(size_t i = 0; i < 3; i++) {
            float val = latest_acc_data[i] / LSM6DSR_ACC_SENSITIVITY_FS_2G;
            int16_t val_int = static_cast<int16_t>(std::roundf(val));
            tx_buf[6 + i * 2] = val_int & 0xFF;
            tx_buf[6 + i * 2 + 1] = (val_int >> 8) & 0xFF;
        }

        spi_slave.write_read(tx_buf, rx_buf, 12, 10);
        });

    // Start first transfer
    spi_slave.write_read(tx_buf, rx_buf, 12, 10);

    while (1) {
    	// heartbeat
        led1 = 1;
        HAL_Delay(500);
        led1 = 0;
        HAL_Delay(500);
    }
}
