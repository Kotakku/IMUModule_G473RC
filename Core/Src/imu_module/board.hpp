#pragma once

#include "imu_fusion.hpp"
#include "lsm6dsr_array_32.hpp"
#include "stmbed/digital_in.hpp"
#include "stmbed/digital_out.hpp"
#include "stmbed/stdio_support.hpp"
#include "stmbed/ticker.hpp"
#include "stmbed/uart.hpp"

#include "spi_slave.hpp"

extern UART_HandleTypeDef huart2; // VCP
extern TIM_HandleTypeDef htim16;  // 2khz

extern SPI_HandleTypeDef hspi1;

namespace imu_module {

using namespace stmbed;

struct Peripherals {
private:
    Peripherals() {}
    Peripherals(Peripherals const &) = delete;
    void operator=(Peripherals const &) = delete;

public:
    static Peripherals &get_instance() {
        static Peripherals instance;
        return instance;
    }

    static void enable_std_printf() { stmbed::enable_std_printf(&huart2); }

    DigitalOut led1{PA0};
    DigitalOut led2{PA1};

    std::array<DigitalIn, 32> imu_misos = {
        // 1..16
        PB12,
        PC4,
        PC5,
        PB11,
        PB10,
        PB0,
        PB1,
        PB2,
        PB5,
        PC9,
        PA9,
        PB4,
        PB7,
        PC7,
        PC8,
        PB6,

        // 17..32
        PC0,
        PC6,
        PB15,
        PC1,
        PC2,
        PB14,
        PB13,
        PC3,
        PC12,
        PC13,
        PB9,
        PB3,
        PC10,
        PC15,
        PC14,
        PC11,
    };
    DigitalOut imu_cs{PA10};
    DigitalOut imu_clk{PA11};
    DigitalOut imu_mosi{PA12};

    Ticker main_ticker{&htim16};

    LSM6DSRArray32 imu{imu_mosi, imu_misos, imu_clk, imu_cs};
    IMUFusion imu_fusion{imu, 2*raw_gyro_var, 2, 2*raw_acc_var, 2};

    SPISlave spi_slave{&hspi1};
};

} // namespace imu_module
