#include "imu_module/board.hpp"
#include "imu_module/tests.hpp"
#include "main.h"

extern "C" void cppmain() {
    using namespace imu_module;

    // unit_test::led_test();
    // unit_test::vcp_test();
    // unit_test::ticker_test();
    // unit_test::imu_whoami_test();
    // unit_test::imu_read_raw_test();
    // unit_test::imu_axes_check_test();
    unit_test::imu_fusion_test();

    // unit_test::imu_fusion_angle_check_test();

    // unit_test::imu_gyro_bias_check();
}
