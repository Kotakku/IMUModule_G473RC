#pragma once

#include "arm_math.h"
#include "board.hpp"

namespace imu_module {

namespace fpu_dsp_test {

#define printf_fpu_dsp_test_func()                                                                                     \
    { printf("test: %s\n", __func__); }

void float_mul_add_time_test() {
    Peripherals::enable_std_printf();
    printf_fpu_dsp_test_func();

    while (1) {
        float a = 1.1f;
        float b = 1.2f;
        float c = 0.0f;
        for (int i = 0; i < 10000; i++) {
            b += a;
            c += a * b;
        }
        printf("%f\n", c);

        HAL_Delay(10);
    }
}

void float_div_time_test() {
    Peripherals::enable_std_printf();
    printf_fpu_dsp_test_func();

    while (1) {
        float a = 1e8f;
        float b = 1.1f;
        for (int i = 0; i < 10000; i++) {
            a /= b;
        }
        printf("%f\n", a);

        HAL_Delay(10);
    }
}

void float_sqrtf_time_test() {
    Peripherals::enable_std_printf();
    printf_fpu_dsp_test_func();

    while (1) {
        float a = 0.1f;
        float b = 0.0f;
        for (int i = 0; i < 10000; i++) {
            a += 0.01f;
            b += sqrtf(a);
        }
        printf("%f\n", b);

        HAL_Delay(10);
    }
}

void float_sin_add_time_test() {
    Peripherals::enable_std_printf();
    printf_fpu_dsp_test_func();

    while (1) {
        float a = 1.1f;
        float b = 0.0f;
        for (int i = 0; i < 10000; i++) {
            a += 0.1f;
            b += sin(a);
        }
        printf("%f\n", b);

        HAL_Delay(10);
    }
}

void float_sinf_add_time_test() {
    Peripherals::enable_std_printf();
    printf_fpu_dsp_test_func();

    while (1) {
        float a = 1.1f;
        float b = 0.0f;
        for (int i = 0; i < 10000; i++) {
            a += 0.1f;
            b += sinf(a);
        }
        printf("%f\n", b);

        HAL_Delay(10);
    }
}

void float_arm_math_sinf_add_time_test() {
    Peripherals::enable_std_printf();
    printf_fpu_dsp_test_func();

    while (1) {
        float a = 1.1f;
        float b = 0.0f;
        for (int i = 0; i < 10000; i++) {
            a += 0.1f;
            b += arm_sin_f32(a);
        }
        printf("%f\n", b);

        HAL_Delay(10);
    }
}

void vector_eigen_add_time_test() {
    Peripherals::enable_std_printf();
    printf_fpu_dsp_test_func();

    while (1) {

        Eigen::Vector3f a(1.1f, 2.2f, 3.3f);
        Eigen::Vector3f b(4.4f, 5.5f, 6.6f);
        Eigen::Vector3f c(0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 10000; i++) {
            c += a;
        }
        printf("%f\n", c[0]);

        HAL_Delay(10);
    }
}

void vector_cmsis_dsp_add_time_test() {
    Peripherals::enable_std_printf();
    printf_fpu_dsp_test_func();

    while (1) {
        float32_t a_data[3] = {1.1f, 2.2f, 3.3f};
        float32_t b_data[3] = {4.4f, 5.5f, 6.6f};
        float32_t c_data[3] = {0.0f, 0.0f, 0.0f};

        arm_matrix_instance_f32 a;
        arm_matrix_instance_f32 b;
        arm_matrix_instance_f32 c;

        arm_mat_init_f32(&a, 3, 1, a_data);
        arm_mat_init_f32(&b, 3, 1, b_data);
        arm_mat_init_f32(&c, 3, 1, c_data);

        for (int i = 0; i < 10000; i++) {
            arm_mat_add_f32(&a, &c, &c);
        }
        printf("%f\n", c_data[0]);

        HAL_Delay(10);
    }
}

void vector_raw_add_time_test() {
    Peripherals::enable_std_printf();
    printf_fpu_dsp_test_func();

    while (1) {
        float32_t a_data[3] = {1.1f, 2.2f, 3.3f};
        float32_t c_data[3] = {0.0f, 0.0f, 0.0f};

        for (int i = 0; i < 10000; i++) {
            for (int j = 0; j < 3; j++) {
                c_data[j] += a_data[j];
            }
        }
        printf("%f\n", c_data[0]);

        HAL_Delay(10);
    }
}

void matrix_eigen_mul_time_test() {
    Peripherals::enable_std_printf();
    printf_fpu_dsp_test_func();

    while (1) {
        Eigen::Matrix3f a;
        a << 1.1f, 2.2f, 3.3f, 4.4f, 5.5f, 6.6f, 7.7f, 8.8f, 9.9f;
        Eigen::Matrix3f b;
        b << 1.1f, 2.2f, 3.3f, 4.4f, 5.5f, 6.6f, 7.7f, 8.8f, 9.9f;
        Eigen::Matrix3f c;
        for (int i = 0; i < 10000; i++) {
            c = a * b;
        }
        printf("%f\n", c(0, 0));

        HAL_Delay(10);
    }
}

void matrix_cmsis_dsp_mul_time_test() {
    Peripherals::enable_std_printf();
    printf_fpu_dsp_test_func();

    while (1) {
        float32_t a_data[9] = {1.1f, 2.2f, 3.3f, 4.4f, 5.5f, 6.6f, 7.7f, 8.8f, 9.9f};
        float32_t b_data[9] = {1.1f, 2.2f, 3.3f, 4.4f, 5.5f, 6.6f, 7.7f, 8.8f, 9.9f};
        float32_t c_data[9];

        arm_matrix_instance_f32 a;
        arm_matrix_instance_f32 b;
        arm_matrix_instance_f32 c;

        arm_mat_init_f32(&a, 3, 3, a_data);
        arm_mat_init_f32(&b, 3, 3, b_data);
        arm_mat_init_f32(&c, 3, 3, c_data);

        for (int i = 0; i < 10000; i++) {
            arm_mat_mult_f32(&a, &b, &c);
        }
        printf("%f\n", c_data[0]);
    }
}

} // namespace fpu_dsp_test

} // namespace imu_module
