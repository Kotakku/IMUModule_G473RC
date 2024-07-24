#pragma once

#include "Eigen/Dense"
#include "cpp_robotics/vector/vector3.hpp"
#include <array>
#include <cmath>
#include <tuple>
using Vector3f = cpp_robotics::Vector3f;

constexpr size_t NUM_SENSOR = 32;

// 生値のジャイロのバイアス[rad/s]
constexpr static std::array<Vector3f, NUM_SENSOR> gyro_bias = {
    Vector3f{1.470230e-03, -5.445150e-03, -2.594200e-04},  //
    Vector3f{4.230970e-03, -4.702240e-03, 4.822290e-03},   //
    Vector3f{5.942560e-03, -4.953960e-03, -1.341330e-03},  //
    Vector3f{1.042582e-02, -8.862000e-04, -2.114060e-03},  //
    Vector3f{9.463280e-03, -4.336130e-03, -3.007700e-03},  //
    Vector3f{1.928700e-03, -8.362590e-03, -6.584200e-04},  //
    Vector3f{-6.019100e-04, -2.299350e-03, -6.631680e-03}, //
    Vector3f{3.601480e-03, -3.351030e-03, 2.390280e-03},   //
    Vector3f{4.841470e-03, -7.975290e-03, -1.207990e-03},  //
    Vector3f{9.148310e-03, -6.790400e-03, -2.519800e-03},  //
    Vector3f{-1.546070e-03, -1.807101e-02, 1.712000e-05},  //
    Vector3f{2.689070e-03, -7.002380e-03, 1.483040e-03},   //
    Vector3f{-1.160360e-03, -1.450262e-02, 1.683840e-03},  //
    Vector3f{4.387080e-03, -6.770360e-03, -4.756530e-03},  //
    Vector3f{1.941880e-03, -7.979660e-03, -2.545700e-03},  //
    Vector3f{1.463710e-03, -1.251826e-02, 3.632430e-03},   //
    Vector3f{1.279190e-03, -8.273720e-03, 6.090820e-03},   //
    Vector3f{1.525460e-03, -1.078639e-02, -3.661590e-03},  //
    Vector3f{7.246900e-04, -8.199960e-03, 3.628910e-03},   //
    Vector3f{3.000430e-03, -1.544247e-02, -4.359140e-03},  //
    Vector3f{2.092300e-04, -6.835060e-03, 4.744000e-04},   //
    Vector3f{2.196900e-04, -7.196050e-03, -4.504640e-03},  //
    Vector3f{-1.016800e-03, -1.124365e-02, -3.740540e-03}, //
    Vector3f{4.577620e-03, -7.272460e-03, 2.157150e-03},   //
    Vector3f{2.183610e-03, -8.722730e-03, -2.925180e-03},  //
    Vector3f{-6.390300e-04, -7.587670e-03, -5.302400e-03}, //
    Vector3f{7.873710e-03, -1.009365e-02, -2.942090e-03},  //
    Vector3f{9.480100e-04, -9.251850e-03, -2.037380e-03},  //
    Vector3f{2.600000e-03, -6.519720e-03, 6.414530e-03},   //
    Vector3f{8.865540e-03, -7.998480e-03, -8.243870e-03},  //
    Vector3f{7.453180e-03, -6.396410e-03, -5.236410e-03},  //
    Vector3f{7.877460e-03, -7.071060e-03, -1.742550e-03},  //
};

// 生値の加速度センサのバイアス[g]
constexpr static std::array<Vector3f, NUM_SENSOR> acc_bias = {
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
    Vector3f{0, 0, 0}, //
};

struct EulerAngleXYZ {
    double alpha;
    double beta;
    double gamma;
};

using Matrix3f = Eigen::Matrix3f;

// IMUの取り付け姿勢
constexpr static std::array<EulerAngleXYZ, NUM_SENSOR> imu_pose_init = {
    // Tops
    EulerAngleXYZ{0.0, 0.0, -0 * M_PI / 2}, //
    EulerAngleXYZ{0.0, 0.0, -1 * M_PI / 2}, //
    EulerAngleXYZ{0.0, 0.0, -2 * M_PI / 2}, //
    EulerAngleXYZ{0.0, 0.0, -3 * M_PI / 2}, //

    EulerAngleXYZ{0.0, 0.0, -0 * M_PI / 2}, //
    EulerAngleXYZ{0.0, 0.0, -1 * M_PI / 2}, //
    EulerAngleXYZ{0.0, 0.0, -2 * M_PI / 2}, //
    EulerAngleXYZ{0.0, 0.0, -3 * M_PI / 2}, //

    EulerAngleXYZ{0.0, 0.0, -0 * M_PI / 2}, //
    EulerAngleXYZ{0.0, 0.0, -1 * M_PI / 2}, //
    EulerAngleXYZ{0.0, 0.0, -2 * M_PI / 2}, //
    EulerAngleXYZ{0.0, 0.0, -3 * M_PI / 2}, //

    EulerAngleXYZ{0.0, 0.0, -0 * M_PI / 2}, //
    EulerAngleXYZ{0.0, 0.0, -1 * M_PI / 2}, //
    EulerAngleXYZ{0.0, 0.0, -2 * M_PI / 2}, //
    EulerAngleXYZ{0.0, 0.0, -3 * M_PI / 2}, //

    // Bottoms
    EulerAngleXYZ{0.0, -M_PI, -0 * M_PI / 2}, //
    EulerAngleXYZ{0.0, -M_PI, -1 * M_PI / 2}, //
    EulerAngleXYZ{0.0, -M_PI, -2 * M_PI / 2}, //
    EulerAngleXYZ{0.0, -M_PI, -3 * M_PI / 2}, //

    EulerAngleXYZ{0.0, -M_PI, -0 * M_PI / 2}, //
    EulerAngleXYZ{0.0, -M_PI, -1 * M_PI / 2}, //
    EulerAngleXYZ{0.0, -M_PI, -2 * M_PI / 2}, //
    EulerAngleXYZ{0.0, -M_PI, -3 * M_PI / 2}, //

    EulerAngleXYZ{0.0, -M_PI, -0 * M_PI / 2}, //
    EulerAngleXYZ{0.0, -M_PI, -1 * M_PI / 2}, //
    EulerAngleXYZ{0.0, -M_PI, -2 * M_PI / 2}, //
    EulerAngleXYZ{0.0, -M_PI, -3 * M_PI / 2}, //

    EulerAngleXYZ{0.0, -M_PI, -0 * M_PI / 2}, //
    EulerAngleXYZ{0.0, -M_PI, -1 * M_PI / 2}, //
    EulerAngleXYZ{0.0, -M_PI, -2 * M_PI / 2}, //
    EulerAngleXYZ{0.0, -M_PI, -3 * M_PI / 2}, //
};

static Matrix3f gen_rotate_matrix(const EulerAngleXYZ &angle) {
    Matrix3f rot;

    float cos_alpha = std::cos(angle.alpha);
    float sin_alpha = std::sin(angle.alpha);
    float cos_beta = std::cos(angle.beta);
    float sin_beta = std::sin(angle.beta);
    float cos_gamma = std::cos(angle.gamma);
    float sin_gamma = std::sin(angle.gamma);

    rot(0, 0) = cos_gamma * cos_beta;
    rot(0, 1) = cos_gamma * sin_beta * sin_alpha - sin_gamma * cos_alpha;
    rot(0, 2) = cos_gamma * sin_beta * cos_alpha + sin_gamma * sin_alpha;
    rot(1, 0) = sin_gamma * cos_beta;
    rot(1, 1) = sin_gamma * sin_beta * sin_alpha + cos_gamma * cos_alpha;
    rot(1, 2) = sin_gamma * sin_beta * cos_alpha - cos_gamma * sin_alpha;
    rot(2, 0) = -sin_beta;
    rot(2, 1) = cos_beta * sin_alpha;
    rot(2, 2) = cos_beta * cos_alpha;

    return rot;
}

static const std::array<Matrix3f, NUM_SENSOR> imu_pose_rot = {
    gen_rotate_matrix(imu_pose_init[0]),  //
    gen_rotate_matrix(imu_pose_init[1]),  //
    gen_rotate_matrix(imu_pose_init[2]),  //
    gen_rotate_matrix(imu_pose_init[3]),  //
    gen_rotate_matrix(imu_pose_init[4]),  //
    gen_rotate_matrix(imu_pose_init[5]),  //
    gen_rotate_matrix(imu_pose_init[6]),  //
    gen_rotate_matrix(imu_pose_init[7]),  //
    gen_rotate_matrix(imu_pose_init[8]),  //
    gen_rotate_matrix(imu_pose_init[9]),  //
    gen_rotate_matrix(imu_pose_init[10]), //
    gen_rotate_matrix(imu_pose_init[11]), //
    gen_rotate_matrix(imu_pose_init[12]), //
    gen_rotate_matrix(imu_pose_init[13]), //
    gen_rotate_matrix(imu_pose_init[14]), //
    gen_rotate_matrix(imu_pose_init[15]), //

    gen_rotate_matrix(imu_pose_init[16]), //
    gen_rotate_matrix(imu_pose_init[17]), //
    gen_rotate_matrix(imu_pose_init[18]), //
    gen_rotate_matrix(imu_pose_init[19]), //
    gen_rotate_matrix(imu_pose_init[20]), //
    gen_rotate_matrix(imu_pose_init[21]), //
    gen_rotate_matrix(imu_pose_init[22]), //
    gen_rotate_matrix(imu_pose_init[23]), //
    gen_rotate_matrix(imu_pose_init[24]), //
    gen_rotate_matrix(imu_pose_init[25]), //
    gen_rotate_matrix(imu_pose_init[26]), //
    gen_rotate_matrix(imu_pose_init[27]), //
    gen_rotate_matrix(imu_pose_init[28]), //
    gen_rotate_matrix(imu_pose_init[29]), //
    gen_rotate_matrix(imu_pose_init[30]), //
    gen_rotate_matrix(imu_pose_init[31]), //
};

static Vector3f rotate_vec(const Matrix3f &R, const Vector3f &v) {
    Vector3f ret;

    ret[0] = R(0, 0) * v[0] + R(0, 1) * v[1] + R(0, 2) * v[2];
    ret[1] = R(1, 0) * v[0] + R(1, 1) * v[1] + R(1, 2) * v[2];
    ret[2] = R(2, 0) * v[0] + R(2, 1) * v[1] + R(2, 2) * v[2];

    return ret;
}

// using TF = cpp_robotics::TransferFunction<float>;
// using VectorTF3 = std::array<TF, 3>;
// using FilterPair = std::pair<VectorTF3, VectorTF3>;

// constexpr float notch_zeta = 0.05;
// constexpr float notch_d = 0.0;
// static std::vector<FilterPair> generate_filter(const std::vector<float> &notch_hz_freqs, float Ts = 1.0f / 200.0f,
//                                                size_t sensor_num = NUM_SENSOR) {
//     using namespace cpp_robotics;
//     std::vector<FilterPair> filters;
//     for (const auto &freq : notch_hz_freqs) {
//         // printf("tp1: %f\n", freq);
//         TF notch_filter = NotchFilter<float>(freq * 2 * M_PI, notch_zeta, notch_d, Ts);
//         notch_filter = notch_filter;

//         // auto debug_vec = [&](const auto &vec) {
//         //     printf("vec(%ld) = ", vec.size());
//         //     for (auto &v : vec) {
//         //         printf("%f, ", v);
//         //     }
//         //     printf("\n");
//         // };
//         auto notch_num = notch_filter.descrete_num_array();
//         auto notch_den = notch_filter.descrete_den_array();
//         // debug_vec(notch_num);
//         // debug_vec(notch_den);

//         TF::tf_t notch_tf_info = {
//             .num = notch_num,
//             .den = notch_den,
//             .Ts = Ts,
//         };

//         TF::tf_t comp_tf_info =
//             (static_cast<float>(sensor_num - 1) - notch_tf_info) / static_cast<float>(sensor_num - 1);
//         TF comp_filter;
//         comp_filter.set_discrite(comp_tf_info.num, comp_tf_info.den, comp_tf_info.Ts);

//         // debug_vec(comp_tf_info.num);
//         // debug_vec(comp_tf_info.den);

//         // printf("tp4\n");

//         filters.push_back(FilterPair{VectorTF3{notch_filter, notch_filter, notch_filter},
//                                      VectorTF3{comp_filter, comp_filter, comp_filter}});
//     }
//     // printf("tp4\n");
//     return filters;
// }

// // Notch and Interference suppression filter
// static std::array<std::vector<FilterPair>, NUM_SENSOR> ni_filters = {
//     generate_filter(std::vector<float>{92.606722f}), //
//     generate_filter(std::vector<float>{}),           //
//     generate_filter(std::vector<float>{}),           //
//     generate_filter(std::vector<float>{}),           //
//     generate_filter(std::vector<float>{}),           //
//     generate_filter(std::vector<float>{}),           //
//     generate_filter(std::vector<float>{}),           //
//     generate_filter(std::vector<float>{}),           //
//     generate_filter(std::vector<float>{}),           //
//     generate_filter(std::vector<float>{}),           //
//     generate_filter(std::vector<float>{}),           //
//     generate_filter(std::vector<float>{}),           //
//     generate_filter(std::vector<float>{}),           //
//     generate_filter(std::vector<float>{}),           //
//     generate_filter(std::vector<float>{}),           //
//     generate_filter(std::vector<float>{}),           //

//     // generate_filter(std::vector<float>{92.606722f}),                                    //
//     // generate_filter(std::vector<float>{}),                                              //
//     // generate_filter(std::vector<float>{}),                                              //
//     // generate_filter(std::vector<float>{}),                                              //
//     // generate_filter(std::vector<float>{10.289636f, 28.568423f}),                        //
//     // generate_filter(std::vector<float>{3.776705f}),                                     //
//     // generate_filter(std::vector<float>{}),                                              //
//     // generate_filter(std::vector<float>{}),                                              //
//     // generate_filter(std::vector<float>{12.351539f, 16.022208f}),                        //
//     // generate_filter(std::vector<float>{18.277130f}),                                    //
//     // generate_filter(std::vector<float>{20.006843f}),                                    //
//     // generate_filter(std::vector<float>{}),                                              //
//     // generate_filter(std::vector<float>{}),                                              //
//     // generate_filter(std::vector<float>{8.601344f, 12.389646f, 28.310788f, 74.949239f}), //
//     // generate_filter(std::vector<float>{8.601344f, 36.911303f}),                         //
//     // generate_filter(std::vector<float>{12.351539f, 28.310788f, 36.911303f}),            //
// };
