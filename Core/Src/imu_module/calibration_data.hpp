#pragma once

#include "Eigen/Dense"
#include <array>
#include <cmath>
#include <tuple>
using Vector3f = Eigen::Vector3f;

constexpr size_t NUM_SENSOR = 32;

static constexpr float raw_gyro_var = 0.0023;
static constexpr float raw_acc_var = 0.009;

// 生値のジャイロのバイアス[rad/s]
const static std::array<Vector3f, NUM_SENSOR> gyro_biases = {
	Vector3f{1.160132e-03, -6.248791e-03, 5.877708e-04},
	Vector3f{4.733679e-03, -5.332512e-03, 5.546316e-03},
	Vector3f{6.447411e-03, -5.404891e-03, 9.769382e-04},
	Vector3f{1.056069e-02, -7.573662e-04, -1.444299e-03},
	Vector3f{9.254740e-03, -4.103965e-03, -1.617072e-03},
	Vector3f{2.389687e-03, -8.628457e-03, 2.207640e-04},
	Vector3f{-1.809263e-04, -2.285679e-03, -5.532247e-03},
	Vector3f{3.522483e-03, -3.274047e-03, 3.049550e-03},
	Vector3f{5.075059e-03, -8.000398e-03, -2.278171e-04},
	Vector3f{9.722997e-03, -6.848299e-03, -1.698835e-03},
	Vector3f{-1.626704e-03, -1.883859e-02, 1.470016e-03},
	Vector3f{2.774682e-03, -7.009162e-03, 1.902767e-03},
	Vector3f{-1.478366e-03, -1.447473e-02, 2.882664e-03},
	Vector3f{4.758902e-03, -7.522257e-03, -4.543768e-03},
	Vector3f{1.947796e-03, -8.152645e-03, -1.574517e-03},
	Vector3f{1.068660e-03, -1.268062e-02, 4.527587e-03},
	Vector3f{1.554610e-03, -8.490143e-03, 6.747087e-03},
	Vector3f{1.801230e-03, -1.132978e-02, -4.033995e-03},
	Vector3f{8.294070e-04, -8.495452e-03, 4.512665e-03},
	Vector3f{3.053457e-03, -1.629095e-02, -4.064872e-03},
	Vector3f{8.423823e-04, -6.681768e-03, 1.253766e-03},
	Vector3f{1.103265e-03, -7.802263e-03, -4.598820e-03},
	Vector3f{-9.134122e-04, -1.136559e-02, -3.235320e-03},
	Vector3f{5.140770e-03, -7.126653e-03, 2.251981e-03},
	Vector3f{2.450532e-03, -8.767827e-03, -1.931099e-03},
	Vector3f{-2.751326e-04, -7.194623e-03, -5.584416e-03},
	Vector3f{7.696238e-03, -9.730491e-03, -2.103199e-03},
	Vector3f{1.467297e-03, -9.501039e-03, -1.659069e-03},
	Vector3f{2.429795e-03, -6.649597e-03, 7.628217e-03},
	Vector3f{8.067138e-03, -6.969686e-03, -8.721627e-03},
	Vector3f{7.422742e-03, -6.774517e-03, -3.907290e-03},
	Vector3f{7.818590e-03, -7.070076e-03, -1.761190e-03},
};

// 生値の加速度センサのバイアス[m/s^2]
const static std::array<Vector3f, NUM_SENSOR> acc_biases = {
	Vector3f{-1.561795e-01, -1.541092e-01, 2.837969e-01},
	Vector3f{9.846993e-02, -2.446907e-01, 5.974369e-02},
	Vector3f{1.793425e-01, -1.891401e-01, 9.352856e-02},
	Vector3f{1.405509e-01, 1.358656e-03, 1.872576e-01},
	Vector3f{-1.657858e-02, -8.756004e-02, 5.684054e-02},
	Vector3f{9.228908e-02, -2.754936e-01, 3.158953e-02},
	Vector3f{1.959222e-01, -1.576722e-01, 3.000278e-02},
	Vector3f{1.124483e-01, -8.966994e-04, 4.645527e-02},
	Vector3f{-1.120570e-01, -1.835539e-01, 1.983187e-01},
	Vector3f{2.311628e-02, -2.775536e-01, 1.284639e-01},
	Vector3f{1.598996e-01, -1.449827e-01, 1.027460e-01},
	Vector3f{1.123341e-01, 8.934936e-03, 1.086147e-01},
	Vector3f{-9.070820e-02, -1.148890e-01, 1.399617e-02},
	Vector3f{2.645419e-02, -2.975042e-01, -1.258460e-01},
	Vector3f{2.198843e-01, -9.836705e-02, 1.063759e-02},
	Vector3f{1.046031e-02, -3.085845e-02, 3.509895e-01},
	Vector3f{1.612579e-01, -1.069693e-01, 2.345636e-01},
	Vector3f{-9.374162e-02, 9.167720e-02, 4.633002e-02},
	Vector3f{-1.559547e-01, -2.268606e-01, -3.901517e-02},
	Vector3f{1.052748e-01, -2.592626e-01, 1.287542e-01},
	Vector3f{2.823044e-01, -5.288136e-02, 6.395879e-02},
	Vector3f{-1.337192e-02, -7.154031e-03, 4.008176e-02},
	Vector3f{-1.391644e-01, -1.461138e-01, 2.398493e-01},
	Vector3f{-8.375369e-03, -3.064686e-01, 2.550964e-01},
	Vector3f{8.971516e-02, -1.868194e-01, 2.688620e-01},
	Vector3f{5.143248e-02, 6.075989e-03, 2.758195e-02},
	Vector3f{-1.061364e-01, -2.042931e-01, 1.225868e-01},
	Vector3f{7.917409e-02, -2.483778e-01, 1.625151e-01},
	Vector3f{2.015839e-02, -1.152792e-01, 2.923823e-01},
	Vector3f{9.480211e-02, 3.137753e-01, 1.095472e-01},
	Vector3f{-1.708353e-01, -2.590479e-01, 1.459011e-01},
	Vector3f{1.140375e-01, -2.592392e-01, 1.183406e-01},
};

// struct EulerAngleXYZ {
//     float alpha;
//     float beta;
//     float gamma;
// };

// using Matrix3f = Eigen::Matrix3f;

// // IMUの取り付け姿勢
// constexpr static std::array<EulerAngleXYZ, NUM_SENSOR> imu_pose_init = {
//     // Tops
//     EulerAngleXYZ{0.0, 0.0, -0 * M_PI / 2}, //
//     EulerAngleXYZ{0.0, 0.0, -1 * M_PI / 2}, //
//     EulerAngleXYZ{0.0, 0.0, -2 * M_PI / 2}, //
//     EulerAngleXYZ{0.0, 0.0, -3 * M_PI / 2}, //

//     EulerAngleXYZ{0.0, 0.0, -0 * M_PI / 2}, //
//     EulerAngleXYZ{0.0, 0.0, -1 * M_PI / 2}, //
//     EulerAngleXYZ{0.0, 0.0, -2 * M_PI / 2}, //
//     EulerAngleXYZ{0.0, 0.0, -3 * M_PI / 2}, //

//     EulerAngleXYZ{0.0, 0.0, -0 * M_PI / 2}, //
//     EulerAngleXYZ{0.0, 0.0, -1 * M_PI / 2}, //
//     EulerAngleXYZ{0.0, 0.0, -2 * M_PI / 2}, //
//     EulerAngleXYZ{0.0, 0.0, -3 * M_PI / 2}, //

//     EulerAngleXYZ{0.0, 0.0, -0 * M_PI / 2}, //
//     EulerAngleXYZ{0.0, 0.0, -1 * M_PI / 2}, //
//     EulerAngleXYZ{0.0, 0.0, -2 * M_PI / 2}, //
//     EulerAngleXYZ{0.0, 0.0, -3 * M_PI / 2}, //

//     // Bottoms
//     EulerAngleXYZ{-M_PI, 0.0, -0 * M_PI / 2}, //
//     EulerAngleXYZ{-M_PI, 0.0, -1 * M_PI / 2}, //
//     EulerAngleXYZ{-M_PI, 0.0, -2 * M_PI / 2}, //
//     EulerAngleXYZ{-M_PI, 0.0, -3 * M_PI / 2}, //

//     EulerAngleXYZ{-M_PI, 0.0, -0 * M_PI / 2}, //
//     EulerAngleXYZ{-M_PI, 0.0, -1 * M_PI / 2}, //
//     EulerAngleXYZ{-M_PI, 0.0, -2 * M_PI / 2}, //
//     EulerAngleXYZ{-M_PI, 0.0, -3 * M_PI / 2}, //

//     EulerAngleXYZ{-M_PI, 0.0, -0 * M_PI / 2}, //
//     EulerAngleXYZ{-M_PI, 0.0, -1 * M_PI / 2}, //
//     EulerAngleXYZ{-M_PI, 0.0, -2 * M_PI / 2}, //
//     EulerAngleXYZ{-M_PI, 0.0, -3 * M_PI / 2}, //

//     EulerAngleXYZ{-M_PI, 0.0, -0 * M_PI / 2}, //
//     EulerAngleXYZ{-M_PI, 0.0, -1 * M_PI / 2}, //
//     EulerAngleXYZ{-M_PI, 0.0, -2 * M_PI / 2}, //
//     EulerAngleXYZ{-M_PI, 0.0, -3 * M_PI / 2}, //
// };

// static Matrix3f gen_rotate_matrix(const EulerAngleXYZ &angle) {
//     Matrix3f rot;

//     float cos_alpha = std::cos(angle.alpha);
//     float sin_alpha = std::sin(angle.alpha);
//     float cos_beta = std::cos(angle.beta);
//     float sin_beta = std::sin(angle.beta);
//     float cos_gamma = std::cos(angle.gamma);
//     float sin_gamma = std::sin(angle.gamma);

//     rot(0, 0) = cos_gamma * cos_beta;
//     rot(0, 1) = cos_gamma * sin_beta * sin_alpha - sin_gamma * cos_alpha;
//     rot(0, 2) = cos_gamma * sin_beta * cos_alpha + sin_gamma * sin_alpha;
//     rot(1, 0) = sin_gamma * cos_beta;
//     rot(1, 1) = sin_gamma * sin_beta * sin_alpha + cos_gamma * cos_alpha;
//     rot(1, 2) = sin_gamma * sin_beta * cos_alpha - cos_gamma * sin_alpha;
//     rot(2, 0) = -sin_beta;
//     rot(2, 1) = cos_beta * sin_alpha;
//     rot(2, 2) = cos_beta * cos_alpha;

//     return rot;
// }

// static const std::array<Matrix3f, NUM_SENSOR> imu_pose_rot = {
//     gen_rotate_matrix(imu_pose_init[0]),  //
//     gen_rotate_matrix(imu_pose_init[1]),  //
//     gen_rotate_matrix(imu_pose_init[2]),  //
//     gen_rotate_matrix(imu_pose_init[3]),  //
//     gen_rotate_matrix(imu_pose_init[4]),  //
//     gen_rotate_matrix(imu_pose_init[5]),  //
//     gen_rotate_matrix(imu_pose_init[6]),  //
//     gen_rotate_matrix(imu_pose_init[7]),  //
//     gen_rotate_matrix(imu_pose_init[8]),  //
//     gen_rotate_matrix(imu_pose_init[9]),  //
//     gen_rotate_matrix(imu_pose_init[10]), //
//     gen_rotate_matrix(imu_pose_init[11]), //
//     gen_rotate_matrix(imu_pose_init[12]), //
//     gen_rotate_matrix(imu_pose_init[13]), //
//     gen_rotate_matrix(imu_pose_init[14]), //
//     gen_rotate_matrix(imu_pose_init[15]), //

//     gen_rotate_matrix(imu_pose_init[16]), //
//     gen_rotate_matrix(imu_pose_init[17]), //
//     gen_rotate_matrix(imu_pose_init[18]), //
//     gen_rotate_matrix(imu_pose_init[19]), //
//     gen_rotate_matrix(imu_pose_init[20]), //
//     gen_rotate_matrix(imu_pose_init[21]), //
//     gen_rotate_matrix(imu_pose_init[22]), //
//     gen_rotate_matrix(imu_pose_init[23]), //
//     gen_rotate_matrix(imu_pose_init[24]), //
//     gen_rotate_matrix(imu_pose_init[25]), //
//     gen_rotate_matrix(imu_pose_init[26]), //
//     gen_rotate_matrix(imu_pose_init[27]), //
//     gen_rotate_matrix(imu_pose_init[28]), //
//     gen_rotate_matrix(imu_pose_init[29]), //
//     gen_rotate_matrix(imu_pose_init[30]), //
//     gen_rotate_matrix(imu_pose_init[31]), //
// };
