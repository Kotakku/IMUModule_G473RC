#pragma once

#include "Eigen/Dense"
#include <array>
#include <cmath>
#include <tuple>
using Vector3f = Eigen::Vector3f;

constexpr size_t NUM_SENSOR = 32;

// 生値のジャイロのバイアス[rad/s]
const static std::array<Vector3f, NUM_SENSOR> gyro_bias = {
    Vector3f{1.617710e-03, -5.860810e-03, -2.585900e-04},  //
    Vector3f{4.340720e-03, -5.259050e-03, 5.470480e-03},   //
    Vector3f{6.167680e-03, -5.321210e-03, -1.086770e-03},  //
    Vector3f{1.057086e-02, -1.174580e-03, -1.684230e-03},  //
    Vector3f{9.078660e-03, -4.450680e-03, -2.730570e-03},  //
    Vector3f{2.112580e-03, -8.834430e-03, -4.735100e-04},  //
    Vector3f{-5.644900e-04, -2.375890e-03, -6.423400e-03}, //
    Vector3f{3.626490e-03, -3.280220e-03, 2.273040e-03},   //
    Vector3f{4.896210e-03, -8.275980e-03, -7.985500e-04},  //
    Vector3f{9.483540e-03, -7.139860e-03, -2.136770e-03},  //
    Vector3f{-1.570800e-03, -1.869635e-02, 7.652400e-04},  //
    Vector3f{2.657320e-03, -7.256230e-03, 1.511930e-03},   //
    Vector3f{-1.304260e-03, -1.462047e-02, 2.154130e-03},  //
    Vector3f{4.667950e-03, -7.298910e-03, -4.566580e-03},  //
    Vector3f{2.010630e-03, -8.227390e-03, -2.223980e-03},  //
    Vector3f{1.329100e-03, -1.266887e-02, 3.831450e-03},   //
    Vector3f{1.511910e-03, -8.217540e-03, 6.013320e-03},   //
    Vector3f{1.555230e-03, -1.138830e-02, -3.999070e-03},  //
    Vector3f{9.126100e-04, -8.215930e-03, 3.979420e-03},   //
    Vector3f{2.401720e-03, -1.612159e-02, -3.944110e-03},  //
    Vector3f{1.093700e-04, -6.910860e-03, 2.428800e-04},   //
    Vector3f{7.301900e-04, -7.770340e-03, -4.336950e-03},  //
    Vector3f{-1.339790e-03, -1.124593e-02, -3.643740e-03}, //
    Vector3f{4.704120e-03, -7.450070e-03, 1.901270e-03},   //
    Vector3f{2.793190e-03, -8.948910e-03, -2.478310e-03},  //
    Vector3f{-4.764100e-04, -7.253600e-03, -5.668860e-03}, //
    Vector3f{7.910790e-03, -1.016978e-02, -2.680230e-03},  //
    Vector3f{1.344150e-03, -9.632710e-03, -2.332980e-03},  //
    Vector3f{2.749590e-03, -6.740860e-03, 6.981160e-03},   //
    Vector3f{8.221650e-03, -7.141640e-03, -8.451610e-03},  //
    Vector3f{7.233470e-03, -6.820900e-03, -4.838810e-03},  //
    Vector3f{7.842060e-03, -7.536230e-03, -1.753900e-03},  //
};

// 生値の加速度センサのバイアス[m/s^2]
const static std::array<Vector3f, NUM_SENSOR> acc_bias = {
    Vector3f{1.470424e-02, -2.647168e-01, 2.735083e-01},  //
    Vector3f{2.197779e-01, -8.625408e-02, 4.813346e-02},  //
    Vector3f{2.262461e-02, -7.117907e-02, 9.537342e-02},  //
    Vector3f{2.397146e-02, -1.537760e-01, 1.933496e-01},  //
    Vector3f{1.308711e-01, -2.136724e-01, 6.277633e-02},  //
    Vector3f{2.072626e-01, -1.217710e-01, 3.249734e-02},  //
    Vector3f{5.059871e-02, -2.987557e-02, 1.587879e-02},  //
    Vector3f{-1.972070e-03, -1.518415e-01, 4.699983e-02}, //
    Vector3f{5.046672e-02, -2.940633e-01, 2.007419e-01},  //
    Vector3f{1.517063e-01, -1.154677e-01, 1.175078e-01},  //
    Vector3f{1.277203e-02, -1.858936e-02, 1.032311e-01},  //
    Vector3f{-2.681960e-03, -1.379705e-01, 1.080198e-01}, //
    Vector3f{8.549488e-02, -2.128755e-01, -6.057690e-03}, //
    Vector3f{1.468002e-01, -1.420442e-01, -1.310832e-01}, //
    Vector3f{6.554469e-02, 2.206225e-02, 6.375820e-03},   //
    Vector3f{-8.508460e-02, -1.657459e-01, 3.259346e-01}, //
    Vector3f{9.497400e-03, -2.266312e-01, 2.365010e-01},  //
    Vector3f{3.434699e-02, -6.173163e-02, 2.981665e-02},  //
    Vector3f{3.285680e-03, -1.128154e-01, -2.728141e-02}, //
    Vector3f{-8.236120e-03, -1.063868e-01, 1.248611e-01}, //
    Vector3f{1.343440e-01, -1.744559e-01, 5.412075e-02},  //
    Vector3f{1.224228e-01, -1.454939e-01, 3.005319e-02},  //
    Vector3f{2.089365e-02, -2.613412e-02, 2.294575e-01},  //
    Vector3f{-1.032089e-01, -1.328413e-01, 2.390634e-01}, //
    Vector3f{-5.066295e-02, -2.954597e-01, 2.627111e-01}, //
    Vector3f{1.688090e-01, -1.549915e-01, 3.629144e-02},  //
    Vector3f{5.532591e-02, -8.390965e-02, 1.206166e-01},  //
    Vector3f{-3.745046e-02, -9.431680e-02, 1.645267e-01}, //
    Vector3f{-1.263705e-01, -2.286535e-01, 2.863024e-01}, //
    Vector3f{2.132987e-01, 1.603293e-01, 9.815665e-02},   //
    Vector3f{-1.386283e-02, -1.396196e-01, 1.433067e-01}, //
    Vector3f{3.564300e-04, -1.041491e-01, 1.099965e-01},  //
};

struct EulerAngleXYZ {
    float alpha;
    float beta;
    float gamma;
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
    ret = R * v;
    return ret;
}

static Vector3f rotate_vec_i(size_t &i, const Vector3f &v) {
    if (i < 16) {
        //  0: 1.00000,2.00000,3.00000
        //  1: 2.00000,-1.00000,3.00000
        //  2: -1.00000,-2.00000,3.00000
        //  3: -2.00000,1.00000,3.00000

        switch (i % 4) {
        case 0:
            return Vector3f(v.x(), v.y(), v.z());
        case 1:
            return Vector3f(v.y(), -v.x(), v.z());
        case 2:
            return Vector3f(-v.x(), -v.y(), v.z());
        case 3:
            return Vector3f(-v.y(), v.x(), v.z());
        }
    } else {
        // 16: -1.00000,2.00000,-3.00000
        // 17: 2.00000,1.00000,-3.00000
        // 18: 1.00000,-2.00000,-3.00000
        // 19: -2.00000,-1.00000,-3.00000

        switch (i % 4) {
        case 0:
            return Vector3f(-v.x(), v.y(), -v.z());
        case 1:
            return Vector3f(v.y(), v.x(), -v.z());
        case 2:
            return Vector3f(v.x(), -v.y(), -v.z());
        case 3:
            return Vector3f(-v.y(), -v.x(), -v.z());
        }
    }

    return v;
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
