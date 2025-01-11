# IMUModule_G473RC

Firmware of [IMUModule_STM32_PCB](https://github.com/Kotakku/IMUModule_STM32_PCB)

## Development environment
- STM32CubeIde 1.14.0

## Communication
- SPI (Clock polarity: Low, Clock phase: 1edge)

After reading 12 bytes, convert according to the following

| byte index | 0 | 1 | 2 | 3 | 4 | 5 |
| :-: | :-: | :-: | :-: | :-: | :-: | :-: |
| description | gyro.x (low) | gyro.x (high) | gyro.y (low) | gyro.y (high) | gyro.z (low) | gyro.z (high) |

| byte index | 6 | 7 | 8 | 9 | 10 | 11 |
| :-: | :-: | :-: | :-: | :-: | :-: | :-: |
| description | acc.x (low) | acc.x (high) | acc.y (low) | acc.y (high) | acc.z (low) | acc.z (high) |

Linear acceleration sensitivity: 0.061mg/LSB  
Angular rate sensitivity: 35mdps/LSB

## キャリブレーション方法

基板を固定した状態で、以下の3方向でimu_bias_calibration関数を実行する  
- 姿勢a: 基準となる任意の姿勢
- 姿勢b: 姿勢aからx軸とy軸が逆になる方向
- 姿勢c: 姿勢aからz軸が逆になる方向

imu_bias_calibration関数を実行するとUARTで結果が出力されるのでcalibration_data.hppにコピーする  
- 姿勢a -> imu_module::calibration_data::base_pose
- 姿勢b -> imu_module::calibration_data::xy_inverted_pose
- 姿勢c -> imu_module::calibration_data::z_inverted_pose
