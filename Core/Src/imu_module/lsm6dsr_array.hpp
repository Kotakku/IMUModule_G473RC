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

class LSM6DSRArray {
public:
    using Scaler = float;
    using Vector3f = Eigen::Vector3f;
    constexpr static size_t NUM_SENSOR = 32; // 32;

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

    LSM6DSRArray(stmbed::DigitalOut &mosi_pin, std::array<stmbed::DigitalIn, 32> &miso_pins,
                 stmbed::DigitalOut &clk_pin, stmbed::DigitalOut &cs_pin)
        : mosi_pin_(mosi_pin), miso_pins_(miso_pins), clk_pin_(clk_pin), cs_pin_(cs_pin) {
        set_cs(1);

        // for (size_t i = 0; i < NUM_SENSOR; i++) {
        //     auto port = miso_pins_[i].get_port();

        //     uint32_t *data_ptr = nullptr;
        //     if (port == GPIOA) {
        //         data_ptr = porta_reads_;
        //     } else if (port == GPIOB) {
        //         data_ptr = portb_reads_;
        //     } else if (port == GPIOC) {
        //         data_ptr = portc_reads_;
        //     }

        //     uint32_t pin_num = 0;
        //     for (size_t j = 0; j < 16; j++) {
        //         if (miso_pins_[i].get_pin() & (1 << j)) {
        //             pin_num = j;
        //             break;
        //         }
        //     }

        //     std::array<uint32_t, 8> left_shift;
        //     for (uint32_t j = 0; j < 8; j++) {
        //         left_shift[j] = (16 + 7 - j - pin_num);
        //     }

        //     miso_pin_configs_[i] = {data_ptr, miso_pins_[i].get_pin(), left_shift};
        // }
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
    std::array<int16_t, 3> get_acc_axes_raw(size_t index);
    //    Vector3f calcu_scaled_acc_axes(size_t index);
    float get_acc_sensitivity() { return acc_sensitivity_; }

    // Gyroscope
    LSM6DSRStatusTypeDef enable_gyro();
    LSM6DSRStatusTypeDef update_gyro_axes();
    std::array<int16_t, 3> get_gyro_axes_raw(size_t index);
    //    Vector3f calcu_scaled_gyro_axes(size_t index);
    float get_gyro_sensitivity() { return gyro_sensitivity_; }

    // Temprature
    LSM6DSRStatusTypeDef update_temperature();
    float get_temperature(size_t index);
    int16_t get_temperature_raw(size_t index);

    std::array<lsm6dst_data_t, NUM_SENSOR> &sensor_data() { return sensor_data_; }

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
            ////////////////////////////// 8.5us / per byte //////////////////////////////
            // 2.5us
            for (int i = 0; i < 8; i++) {
                clk_pin_ = 1;
                software_spi_clk_wait(1);

                // C  B (A9) B
                port_reads_[i] = (GPIOC->IDR << 16) | (GPIOB->IDR & 0xFEFF) | ((GPIOA->IDR & 0x200) >> 1);

                clk_pin_ = 0;
                software_spi_clk_wait(1);
            }

            // 12us
            // for (size_t ch = 0; ch < NUM_SENSOR; ch++) {
            //     uint32_t high_byte = 0;
            //     high_byte |= BIT_READ_SHIFT(miso_pin_configs_[ch], 0);
            //     high_byte |= BIT_READ_SHIFT(miso_pin_configs_[ch], 1);
            //     high_byte |= BIT_READ_SHIFT(miso_pin_configs_[ch], 2);
            //     high_byte |= BIT_READ_SHIFT(miso_pin_configs_[ch], 3);
            //     high_byte |= BIT_READ_SHIFT(miso_pin_configs_[ch], 4);
            //     high_byte |= BIT_READ_SHIFT(miso_pin_configs_[ch], 5);
            //     high_byte |= BIT_READ_SHIFT(miso_pin_configs_[ch], 6);
            //     high_byte |= BIT_READ_SHIFT(miso_pin_configs_[ch], 7);
            //     *(data[ch] + b) = static_cast<uint8_t>(high_byte >> 16);
            // }

            // 8us
            // ループ展開前提
            // uint32_t in0 = port_reads_[0];
            // uint32_t in1 = port_reads_[1];
            // uint32_t in2 = port_reads_[2];
            // uint32_t in3 = port_reads_[3];
            // uint32_t in4 = port_reads_[4];
            // uint32_t in5 = port_reads_[5];
            // uint32_t in6 = port_reads_[6];
            // uint32_t in7 = port_reads_[7];

            // for (int ch = 0; ch < NUM_SENSOR; ch++) {
            //     // 第 i ビットをそれぞれ取り出し、まとめる
            //     uint8_t bit_val = ((in0 >> ch) & 1U) << 7 | ((in1 >> ch) & 1U) << 6 | ((in2 >> ch) & 1U) << 5 |
            //                       ((in3 >> ch) & 1U) << 4 | ((in4 >> ch) & 1U) << 3 | ((in5 >> ch) & 1U) << 2 |
            //                       ((in6 >> ch) & 1U) << 1 | ((in7 >> ch) & 1U) << 0;

            //     *(data[pin_bit_num_[ch]] + b) = bit_val;
            // }

            // 7us/6us (1回目だけ7us)
            uint8_t output[32];
            bit_transpose_8x32_32bit(port_reads_, output);
            for (int ch = 0; ch < NUM_SENSOR; ch++) {
                *(data[pin_bit_num_[ch]] + b) = output[ch];
            }
        }

        set_cs(1);
        return 0;
    }

    inline uint32_t mul_upper(uint32_t a, uint32_t b) {
        // 64ビットの積をとって上位32ビットを返す
        return static_cast<uint32_t>((static_cast<uint64_t>(a) * static_cast<uint64_t>(b)) >> 32);
    }

    union {
        uint32_t x[2];
        uint8_t b[8];
    } m4x8d;

    void bit_transpose_8x32_32bit(const uint32_t port_reads_[8], uint8_t data[32]) {
        // 4回(=32bit÷8bit) ループして、各8ビットブロックを 8×8転置
        for (int block = 0; block < 4; block++) {
            for (int i = 0; i < 8; i++) {
                m4x8d.b[7 - i] = (uint8_t)((port_reads_[i] >> (block * 8)) & 0xFF);
            }

            uint8_t *out = data + block * 8;
            for (int i = 0; i < 7; i++) {
                out[7 - i] = (mul_upper(m4x8d.x[1] & (0x80808080 >> i), (0x02040810 << i)) & 0x0f) << 4;
                out[7 - i] |= (mul_upper(m4x8d.x[0] & (0x80808080 >> i), (0x02040810 << i)) & 0x0f);
            }

            out[0] = (mul_upper((m4x8d.x[1] << 7) & (0x80808080 >> 0), (0x02040810 << 0)) & 0x0f) << 4;
            out[0] |= (mul_upper((m4x8d.x[0] << 7) & (0x80808080 >> 0), (0x02040810 << 0)) & 0x0f);
        }
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
    std::array<stmbed::DigitalIn, 32> &miso_pins_;
    stmbed::DigitalOut &clk_pin_;
    stmbed::DigitalOut &cs_pin_;

    stmbed::DigitalOut debug_pa8{stmbed::PA8};

    uint32_t porta_reads_[8];
    uint32_t portb_reads_[8];
    uint32_t portc_reads_[8];
    uint32_t port_reads_[8];
    struct MisoPinConfig {
        uint32_t *data_ptr;
        uint32_t pin_mask; // GPIO_PIN_0 ~ GPIO_PIN_15
        std::array<uint32_t, 8> left_shift;
    };
    std::array<MisoPinConfig, NUM_SENSOR> miso_pin_configs_;

    std::array<uint32_t, 32> pin_bit_num_ = {
        // port B
        5, 6, 7, 27, 11, 8, 15, 12,
        // port A
        10,
        // port B
        26, 4, 3, 0, 22, 21, 18,
        // port C
        16, 19, 20, 23, 1, 2, 17, 13, 14, 9, 28, 31, 24, 25, 30, 29};

    Scaler acc_sensitivity_;
    Scaler gyro_sensitivity_;
    lsm6dsr_odr_xl_t acc_odr;
    lsm6dsr_odr_g_t gyro_odr;
    bool is_acc_enabled_ = false;
    bool is_gyro_enabled_ = false;
};
