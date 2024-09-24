#pragma once

#include "stmbed/digital_in.hpp"
#include "stmbed/digital_out.hpp"
#include <Eigen/Dense>
#include <array>
#include <cstring>

#include "lsm6dsr_reg.hpp"

/* Defines -------------------------------------------------------------------*/
#define GRAVITY 9.81
#define LSM6DSR_ACC_SENSITIVITY_FS_2G (0.061e-3 * GRAVITY)
#define LSM6DSR_ACC_SENSITIVITY_FS_4G (0.122e-3 * GRAVITY)
#define LSM6DSR_ACC_SENSITIVITY_FS_8G (0.244e-3 * GRAVITY)
#define LSM6DSR_ACC_SENSITIVITY_FS_16G (0.488e-3 * GRAVITY)

#define LSM6DSR_GYRO_SENSITIVITY_FS_125DPS (4.370e-3 * M_PI / 180.0)
#define LSM6DSR_GYRO_SENSITIVITY_FS_250DPS (8.750e-3 * M_PI / 180.0)
#define LSM6DSR_GYRO_SENSITIVITY_FS_500DPS (17.500e-3 * M_PI / 180.0)
#define LSM6DSR_GYRO_SENSITIVITY_FS_1000DPS (35.000e-3 * M_PI / 180.0)
#define LSM6DSR_GYRO_SENSITIVITY_FS_2000DPS (70.000e-3 * M_PI / 180.0)
#define LSM6DSR_GYRO_SENSITIVITY_FS_4000DPS (140.000e-3 * M_PI / 180.0)

/* Typedefs ------------------------------------------------------------------*/

typedef enum { LSM6DSR_OK = 0, LSM6DSR_ERROR = -1 } LSM6DSRStatusTypeDef;

typedef enum { LSM6DSR_ACC_HIGH_PERFORMANCE_MODE, LSM6DSR_ACC_LOW_POWER_NORMAL_MODE } LSM6DSR_ACC_Operating_Mode_t;

typedef enum { LSM6DSR_GYRO_HIGH_PERFORMANCE_MODE, LSM6DSR_GYRO_LOW_POWER_NORMAL_MODE } LSM6DSR_GYRO_Operating_Mode_t;

/* Class Declaration ---------------------------------------------------------*/

#define BIT_READ_SHIFT(miso_pin_configs, i)                                                                            \
    ((miso_pin_configs.data_ptr[i] & miso_pin_configs.pin_mask) << miso_pin_configs.left_shift[i])

class LSM6DSRArray32 {
public:
    using Scaler = float;
    using Vector3f = Eigen::Vector3f;
    constexpr static size_t NUM_SENSOR = 32;

    struct lsm6dst_data_t {
        // Accelerometer
        struct {
            std::array<int16_t, 3> raw_value;
            Vector3f value;
        } acc;

        // Gyroscope
        struct {
            std::array<int16_t, 3> raw_value;
            Vector3f value;
        } gyro;

        struct {
            int16_t raw_value;
            float value;
        } temperature;
    };

    LSM6DSRArray32(stmbed::DigitalOut &mosi_pin, std::array<stmbed::DigitalIn, 32> &miso_pins,
                   stmbed::DigitalOut &clk_pin, stmbed::DigitalOut &cs_pin)
        : mosi_pin_(mosi_pin), miso_pins_(miso_pins), clk_pin_(clk_pin), cs_pin_(cs_pin) {
        set_cs(1);

        for (size_t i = 0; i < NUM_SENSOR; i++) {
            auto port = miso_pins_[i].get_port();

            uint32_t *data_ptr = nullptr;
            if (port == GPIOA) {
                data_ptr = porta_reads_;
            } else if (port == GPIOB) {
                data_ptr = portb_reads_;
            } else if (port == GPIOC) {
                data_ptr = portc_reads_;
            }

            uint32_t pin_num = 0;
            for (size_t j = 0; j < 16; j++) {
                if (miso_pins_[i].get_pin() & (1 << j)) {
                    pin_num = j;
                    break;
                }
            }

            std::array<uint32_t, 8> left_shift;
            for (uint32_t j = 0; j < 8; j++) {
                left_shift[j] = (16 + 7 - j - pin_num);
            }

            miso_pin_configs_[i] = {data_ptr, miso_pins_[i].get_pin(), left_shift};
        }
    }

    LSM6DSRStatusTypeDef begin();
    LSM6DSRStatusTypeDef who_am_i(std::array<uint8_t, NUM_SENSOR> &result);

    LSM6DSRStatusTypeDef i3c_disable_set(uint8_t val);
    LSM6DSRStatusTypeDef auto_increment_set(uint8_t val);
    LSM6DSRStatusTypeDef block_data_update_set(uint8_t val);
    LSM6DSRStatusTypeDef fifo_mode_set(lsm6dsr_fifo_mode_t val);
    LSM6DSRStatusTypeDef xl_data_rate_set(lsm6dsr_odr_xl_t val);
    LSM6DSRStatusTypeDef xl_full_scale_set(lsm6dsr_fs_xl_t val);
    LSM6DSRStatusTypeDef gy_data_rate_set(lsm6dsr_odr_g_t val);
    LSM6DSRStatusTypeDef gy_full_scale_set(lsm6dsr_fs_g_t val);

    LSM6DSRStatusTypeDef mem_bank_set(lsm6dsr_reg_access_t val);
    LSM6DSRStatusTypeDef fsm_enable_get(lsm6dsr_emb_fsm_enable_t *val);
    LSM6DSRStatusTypeDef fsm_data_rate_get(lsm6dsr_fsm_odr_t *val);

    LSM6DSRStatusTypeDef xl_filter_lp2_set(uint8_t val);
    LSM6DSRStatusTypeDef xl_hp_path_on_out_set(lsm6dsr_hp_slope_xl_en_t val);

    LSM6DSRStatusTypeDef gy_filter_lp1_set(uint8_t val);
    LSM6DSRStatusTypeDef gy_lp1_bandwidth_set(lsm6dsr_ftype_t val);

    // Accelerometer
    LSM6DSRStatusTypeDef enable_acc();
    LSM6DSRStatusTypeDef update_acc_axes();
    // Vector3f get_acc_axes(size_t index);
    Vector3f get_acc_axes(size_t index);
    std::array<int16_t, 3> get_acc_axes_raw(size_t index);

    // Gyroscope
    LSM6DSRStatusTypeDef enable_gyro();
    LSM6DSRStatusTypeDef update_gyro_axes();
    // Vector3f get_gyro_axes(size_t index);
    Vector3f get_gyro_axes(size_t index);
    std::array<int16_t, 3> get_gyro_axes_raw(size_t index);

    // Temprature
    LSM6DSRStatusTypeDef update_temperature();
    float get_temperature(size_t index);
    int16_t get_temperature_raw(size_t index);

    // debug
    std::array<lsm6dst_data_t, NUM_SENSOR> sensor_data() { return sensor_data_; }

private:
    inline void set_cs(uint8_t value) { cs_pin_ = value; }

    inline void software_spi_clk_wait(const int num) {
        for (int i = 0; i < num; i++) {
            asm("nop");
        }
    }
    inline void write_byte(uint8_t &src) {
        for (int i = 0; i < 8; i++) {
            mosi_pin_ = (src >> (7 - i)) & 1;
            clk_pin_ = 1;
            software_spi_clk_wait(1);
            clk_pin_ = 0;
            software_spi_clk_wait(1);
        }
    }

    inline uint8_t write_reg(uint8_t addr, uint8_t *data, uint16_t len) {
        set_cs(0);
        write_byte(addr);
        for (int i = 0; i < len; i++) {
            write_byte(data[i]);
        }

        set_cs(1);
        return 0;
    }

    inline uint8_t read_reg(uint8_t addr, const std::array<uint8_t *, NUM_SENSOR> &data, uint16_t len) {
        set_cs(0);
        addr |= 0x80;
        write_byte(addr);

        // read bytes
        for (int b = 0; b < len; b++) {
            ////////////////////////////// 15us / per byte //////////////////////////////
            for (int i = 0; i < 8; i++) {
                clk_pin_ = 1;
                software_spi_clk_wait(1);

                porta_reads_[i] = GPIOA->IDR;
                portb_reads_[i] = GPIOB->IDR;
                portc_reads_[i] = GPIOC->IDR;

                clk_pin_ = 0;
                software_spi_clk_wait(1);
            }

            // 12us
            for (size_t ch = 0; ch < NUM_SENSOR; ch++) {
                uint32_t high_byte = 0;
                high_byte |= BIT_READ_SHIFT(miso_pin_configs_[ch], 0);
                high_byte |= BIT_READ_SHIFT(miso_pin_configs_[ch], 1);
                high_byte |= BIT_READ_SHIFT(miso_pin_configs_[ch], 2);
                high_byte |= BIT_READ_SHIFT(miso_pin_configs_[ch], 3);
                high_byte |= BIT_READ_SHIFT(miso_pin_configs_[ch], 4);
                high_byte |= BIT_READ_SHIFT(miso_pin_configs_[ch], 5);
                high_byte |= BIT_READ_SHIFT(miso_pin_configs_[ch], 6);
                high_byte |= BIT_READ_SHIFT(miso_pin_configs_[ch], 7);
                *(data[ch] + b) = static_cast<uint8_t>(high_byte >> 16);
            }
        }

        set_cs(1);
        return 0;
    }

    std::array<lsm6dst_data_t, NUM_SENSOR> sensor_data_;

    struct MemoryWorkspace {
        static constexpr size_t mem_size_per_ch = 16;
        std::array<std::array<uint8_t, mem_size_per_ch>, NUM_SENSOR> data;
        std::array<uint8_t *, NUM_SENSOR> ptr() {
            std::array<uint8_t *, NUM_SENSOR> ptr;
            for (size_t i = 0; i < NUM_SENSOR; i++) {
                ptr[i] = data[i].data();
            }
            return ptr;
        }
    } mem_ws_;

    stmbed::DigitalOut &mosi_pin_;
    std::array<stmbed::DigitalIn, NUM_SENSOR> &miso_pins_;
    stmbed::DigitalOut &clk_pin_;
    stmbed::DigitalOut &cs_pin_;

    uint32_t porta_reads_[8];
    uint32_t portb_reads_[8];
    uint32_t portc_reads_[8];
    struct MisoPinConfig {
        uint32_t *data_ptr;
        uint32_t pin_mask; // GPIO_PIN_0 ~ GPIO_PIN_15
        std::array<uint32_t, 8> left_shift;
    };
    std::array<MisoPinConfig, NUM_SENSOR> miso_pin_configs_;

    Scaler acc_sensitivity_;
    Scaler gyro_sensitivity_;
    lsm6dsr_odr_xl_t acc_odr;
    lsm6dsr_odr_g_t gyro_odr;
    bool is_acc_enabled_ = false;
    bool is_gyro_enabled_ = false;
};
