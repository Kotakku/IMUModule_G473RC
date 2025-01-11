#include "lsm6dsr_array.hpp"
#include "calibration_data.hpp"
#include "lsm6dsr_reg.hpp"

LSM6DSRStatusTypeDef LSM6DSRArray::begin() {
    /* Disable I3C */
    if (i3c_disable_set(LSM6DSR_I3C_DISABLE) != LSM6DSR_OK) {
        return LSM6DSR_ERROR;
    }

    /* Enable register address automatically incremented during a multiple byte
    access with a serial interface. */
    if (auto_increment_set(PROPERTY_ENABLE) != LSM6DSR_OK) {
        return LSM6DSR_ERROR;
    }

    /* Enable BDU */
    if (block_data_update_set(PROPERTY_ENABLE) != LSM6DSR_OK) {
        return LSM6DSR_ERROR;
    }

    /* FIFO mode selection */
    if (fifo_mode_set(LSM6DSR_BYPASS_MODE) != LSM6DSR_OK) {
        return LSM6DSR_ERROR;
    }

    /* Select default output data rate. */
    acc_odr = LSM6DSR_XL_ODR_3333Hz;

    /* Output data rate selection - power down. */
    if (xl_data_rate_set(LSM6DSR_XL_ODR_OFF) != LSM6DSR_OK) {
        return LSM6DSR_ERROR;
    }

    /* Full scale selection. */
    if (xl_full_scale_set(LSM6DSR_2g) != LSM6DSR_OK) {
        return LSM6DSR_ERROR;
    }

    // enable acc LPF2
    if (xl_filter_lp2_set(PROPERTY_DISABLE) != LSM6DSR_OK) {
        return LSM6DSR_ERROR;
    }

    // if (xl_filter_lp2_set(PROPERTY_ENABLE) != LSM6DSR_OK) {
    //     return LSM6DSR_ERROR;
    // }

    // acc LPF2 bandwidth
    // if (xl_hp_path_on_out_set(LSM6DSR_LP_ODR_DIV_10) != LSM6DSR_OK) {
    //     return LSM6DSR_ERROR;
    // }

    /* Select default output data rate. */
    gyro_odr = LSM6DSR_GY_ODR_3333Hz;

    /* Output data rate selection - power down. */
    if (gy_data_rate_set(LSM6DSR_GY_ODR_OFF) != LSM6DSR_OK) {
        return LSM6DSR_ERROR;
    }

    /* Full scale selection. */
    if (gy_full_scale_set(LSM6DSR_2000dps) != LSM6DSR_OK) {
        return LSM6DSR_ERROR;
    }

    if (gy_filter_lp1_set(PROPERTY_DISABLE) != LSM6DSR_OK) {
        return LSM6DSR_ERROR;
    }

    // // enable gyro LPF1
    // if (gy_filter_lp1_set(PROPERTY_ENABLE) != LSM6DSR_OK) {
    //     return LSM6DSR_ERROR;
    // }

    // // gyro LPF1 bandwidth
    // if (gy_lp1_bandwidth_set(LSM6DSR_ULTRA_LIGHT) != LSM6DSR_OK) {
    //     return LSM6DSR_ERROR;
    // }

    is_acc_enabled_ = 0;
    is_gyro_enabled_ = 0;

    return LSM6DSR_OK;
}

LSM6DSRStatusTypeDef LSM6DSRArray::who_am_i(std::array<uint8_t, NUM_SENSOR> &result) {
    if (read_reg(LSM6DSR_WHO_AM_I, mem_ws_.ptr(), 1) != 0) {
        return LSM6DSR_ERROR;
    }

    bool is_all_ok = true;
    for (size_t i = 0; i < NUM_SENSOR; i++) {
        result[i] = mem_ws_.data[i][0];
        if (result[i] != LSM6DSR_ID)
            is_all_ok = false;
    }

    if (not is_all_ok)
        return LSM6DSR_ERROR;

    return LSM6DSR_OK;
}

#define structcpy(val_t) std::memcpy((uint8_t *)&val_t, &mem_ws_.data[0][0], sizeof(val_t))

LSM6DSRStatusTypeDef LSM6DSRArray::i3c_disable_set(uint8_t val) {
    lsm6dsr_ctrl9_xl_t ctrl9_xl;
    lsm6dsr_i3c_bus_avb_t i3c_bus_avb;
    int32_t ret;

    ret = read_reg(LSM6DSR_CTRL9_XL, mem_ws_.ptr() /*(uint8_t *)&ctrl9_xl*/, 1);
    structcpy(ctrl9_xl);

    if (ret == 0) {
        ctrl9_xl.i3c_disable = ((uint8_t)val & 0x80U) >> 7;
        ret = write_reg(LSM6DSR_CTRL9_XL, (uint8_t *)&ctrl9_xl, 1);
    }
    if (ret == 0) {
        ret = read_reg(LSM6DSR_I3C_BUS_AVB, mem_ws_.ptr() /*(uint8_t *)&i3c_bus_avb*/, 1);
        structcpy(i3c_bus_avb);
    }
    if (ret == 0) {
        i3c_bus_avb.i3c_bus_avb_sel = (uint8_t)val & 0x03U;
        ret = write_reg(LSM6DSR_I3C_BUS_AVB, (uint8_t *)&i3c_bus_avb, 1);
    }
    return (ret == 0) ? LSM6DSR_OK : LSM6DSR_ERROR;
}

LSM6DSRStatusTypeDef LSM6DSRArray::auto_increment_set(uint8_t val) {
    lsm6dsr_ctrl3_c_t ctrl3_c;
    int32_t ret;

    ret = read_reg(LSM6DSR_CTRL3_C, mem_ws_.ptr(), 1);
    structcpy(ctrl3_c);
    if (ret == 0) {
        ctrl3_c.if_inc = (uint8_t)val;
        ret = write_reg(LSM6DSR_CTRL3_C, (uint8_t *)&ctrl3_c, 1);
    }
    return (ret == 0) ? LSM6DSR_OK : LSM6DSR_ERROR;
}

LSM6DSRStatusTypeDef LSM6DSRArray::block_data_update_set(uint8_t val) {
    lsm6dsr_ctrl3_c_t ctrl3_c;
    int32_t ret;

    ret = read_reg(LSM6DSR_CTRL3_C, mem_ws_.ptr(), 1);
    structcpy(ctrl3_c);
    if (ret == 0) {
        ctrl3_c.bdu = (uint8_t)val;
        ret = write_reg(LSM6DSR_CTRL3_C, (uint8_t *)&ctrl3_c, 1);
    }
    return (ret == 0) ? LSM6DSR_OK : LSM6DSR_ERROR;
}

LSM6DSRStatusTypeDef LSM6DSRArray::fifo_mode_set(lsm6dsr_fifo_mode_t val) {
    lsm6dsr_fifo_ctrl4_t fifo_ctrl4;
    int32_t ret;

    ret = read_reg(LSM6DSR_FIFO_CTRL4, mem_ws_.ptr(), 1);
    structcpy(fifo_ctrl4);
    if (ret == 0) {
        fifo_ctrl4.fifo_mode = (uint8_t)val;
        ret = write_reg(LSM6DSR_FIFO_CTRL4, (uint8_t *)&fifo_ctrl4, 1);
    }
    return (ret == 0) ? LSM6DSR_OK : LSM6DSR_ERROR;
}

LSM6DSRStatusTypeDef LSM6DSRArray::xl_data_rate_set(lsm6dsr_odr_xl_t val) {
    lsm6dsr_odr_xl_t odr_xl = val;
    lsm6dsr_emb_fsm_enable_t fsm_enable;
    lsm6dsr_fsm_odr_t fsm_odr;
    lsm6dsr_ctrl1_xl_t ctrl1_xl;
    int32_t ret;

    // Check the Finite State Machine data rate constraints
    ret = fsm_enable_get(&fsm_enable);
    if (ret == 0) {
        if ((fsm_enable.fsm_enable_a.fsm1_en | fsm_enable.fsm_enable_a.fsm2_en | fsm_enable.fsm_enable_a.fsm3_en |
             fsm_enable.fsm_enable_a.fsm4_en | fsm_enable.fsm_enable_a.fsm5_en | fsm_enable.fsm_enable_a.fsm6_en |
             fsm_enable.fsm_enable_a.fsm7_en | fsm_enable.fsm_enable_a.fsm8_en | fsm_enable.fsm_enable_b.fsm9_en |
             fsm_enable.fsm_enable_b.fsm10_en | fsm_enable.fsm_enable_b.fsm11_en | fsm_enable.fsm_enable_b.fsm12_en |
             fsm_enable.fsm_enable_b.fsm13_en | fsm_enable.fsm_enable_b.fsm14_en | fsm_enable.fsm_enable_b.fsm15_en |
             fsm_enable.fsm_enable_b.fsm16_en) == PROPERTY_ENABLE) {

            ret = fsm_data_rate_get(&fsm_odr);
            if (ret == 0) {
                switch (fsm_odr) {
                case LSM6DSR_ODR_FSM_12Hz5:
                    if (val == LSM6DSR_XL_ODR_OFF) {
                        odr_xl = LSM6DSR_XL_ODR_12Hz5;
                    } else {
                        odr_xl = val;
                    }
                    break;
                case LSM6DSR_ODR_FSM_26Hz:
                    if (val == LSM6DSR_XL_ODR_OFF) {
                        odr_xl = LSM6DSR_XL_ODR_26Hz;
                    } else if (val == LSM6DSR_XL_ODR_12Hz5) {
                        odr_xl = LSM6DSR_XL_ODR_26Hz;
                    } else {
                        odr_xl = val;
                    }
                    break;
                case LSM6DSR_ODR_FSM_52Hz:
                    if (val == LSM6DSR_XL_ODR_OFF) {
                        odr_xl = LSM6DSR_XL_ODR_52Hz;
                    } else if (val == LSM6DSR_XL_ODR_12Hz5) {
                        odr_xl = LSM6DSR_XL_ODR_52Hz;
                    } else if (val == LSM6DSR_XL_ODR_26Hz) {
                        odr_xl = LSM6DSR_XL_ODR_52Hz;
                    } else {
                        odr_xl = val;
                    }
                    break;
                case LSM6DSR_ODR_FSM_104Hz:
                    if (val == LSM6DSR_XL_ODR_OFF) {
                        odr_xl = LSM6DSR_XL_ODR_104Hz;
                    } else if (val == LSM6DSR_XL_ODR_12Hz5) {
                        odr_xl = LSM6DSR_XL_ODR_104Hz;
                    } else if (val == LSM6DSR_XL_ODR_26Hz) {
                        odr_xl = LSM6DSR_XL_ODR_104Hz;
                    } else if (val == LSM6DSR_XL_ODR_52Hz) {
                        odr_xl = LSM6DSR_XL_ODR_104Hz;
                    } else {
                        odr_xl = val;
                    }
                    break;
                default:
                    odr_xl = val;
                    break;
                }
            }
        }
    }

    if (ret == 0) {
        ret = read_reg(LSM6DSR_CTRL1_XL, mem_ws_.ptr(), 1);
        structcpy(ctrl1_xl);
    }
    if (ret == 0) {
        ctrl1_xl.odr_xl = (uint8_t)odr_xl;
        ret = write_reg(LSM6DSR_CTRL1_XL, (uint8_t *)&ctrl1_xl, 1);
    }
    return (ret == 0) ? LSM6DSR_OK : LSM6DSR_ERROR;
}

LSM6DSRStatusTypeDef LSM6DSRArray::xl_full_scale_set(lsm6dsr_fs_xl_t val) {
    lsm6dsr_ctrl1_xl_t ctrl1_xl;
    int32_t ret;

    ret = read_reg(LSM6DSR_CTRL1_XL, mem_ws_.ptr(), 1);
    structcpy(ctrl1_xl);
    if (ret == 0) {
        ctrl1_xl.fs_xl = (uint8_t)val;
        ret = write_reg(LSM6DSR_CTRL1_XL, (uint8_t *)&ctrl1_xl, 1);
    }

    switch (val) {
    case LSM6DSR_2g:
        acc_sensitivity_ = LSM6DSR_ACC_SENSITIVITY_FS_2G;
        break;
    case LSM6DSR_4g:
        acc_sensitivity_ = LSM6DSR_ACC_SENSITIVITY_FS_4G;
        break;
    case LSM6DSR_8g:
        acc_sensitivity_ = LSM6DSR_ACC_SENSITIVITY_FS_8G;
        break;
    case LSM6DSR_16g:
        acc_sensitivity_ = LSM6DSR_ACC_SENSITIVITY_FS_16G;
        break;
    }

    return (ret == 0) ? LSM6DSR_OK : LSM6DSR_ERROR;
}

LSM6DSRStatusTypeDef LSM6DSRArray::xl_filter_lp2_set(uint8_t val) {
    lsm6dsr_ctrl1_xl_t ctrl1_xl;
    int32_t ret;

    ret = read_reg(LSM6DSR_CTRL1_XL, mem_ws_.ptr(), 1);
    structcpy(ctrl1_xl);
    if (ret == 0) {
        ctrl1_xl.lpf2_xl_en = (uint8_t)val;
        ret = write_reg(LSM6DSR_CTRL1_XL, (uint8_t *)&ctrl1_xl, 1);
    }
    return (ret == 0) ? LSM6DSR_OK : LSM6DSR_ERROR;
}

LSM6DSRStatusTypeDef LSM6DSRArray::xl_hp_path_on_out_set(lsm6dsr_hp_slope_xl_en_t val) {
    lsm6dsr_ctrl8_xl_t ctrl8_xl;
    int32_t ret;

    ret = read_reg(LSM6DSR_CTRL8_XL, mem_ws_.ptr(), 1);
    structcpy(ctrl8_xl);
    if (ret == 0) {
        ctrl8_xl.hp_slope_xl_en = (((uint8_t)val & 0x10U) >> 4);
        ctrl8_xl.hp_ref_mode_xl = (((uint8_t)val & 0x20U) >> 5);
        ctrl8_xl.hpcf_xl = (uint8_t)val & 0x07U;
        ret = write_reg(LSM6DSR_CTRL8_XL, (uint8_t *)&ctrl8_xl, 1);
    }
    return (ret == 0) ? LSM6DSR_OK : LSM6DSR_ERROR;
}

LSM6DSRStatusTypeDef LSM6DSRArray::gy_data_rate_set(lsm6dsr_odr_g_t val) {
    lsm6dsr_odr_g_t odr_gy = val;
    lsm6dsr_emb_fsm_enable_t fsm_enable;
    lsm6dsr_fsm_odr_t fsm_odr;
    lsm6dsr_ctrl2_g_t ctrl2_g;
    int32_t ret;

    // Check the Finite State Machine data rate constraints
    ret = fsm_enable_get(&fsm_enable);
    if (ret == 0) {
        if ((fsm_enable.fsm_enable_a.fsm1_en | fsm_enable.fsm_enable_a.fsm2_en | fsm_enable.fsm_enable_a.fsm3_en |
             fsm_enable.fsm_enable_a.fsm4_en | fsm_enable.fsm_enable_a.fsm5_en | fsm_enable.fsm_enable_a.fsm6_en |
             fsm_enable.fsm_enable_a.fsm7_en | fsm_enable.fsm_enable_a.fsm8_en | fsm_enable.fsm_enable_b.fsm9_en |
             fsm_enable.fsm_enable_b.fsm10_en | fsm_enable.fsm_enable_b.fsm11_en | fsm_enable.fsm_enable_b.fsm12_en |
             fsm_enable.fsm_enable_b.fsm13_en | fsm_enable.fsm_enable_b.fsm14_en | fsm_enable.fsm_enable_b.fsm15_en |
             fsm_enable.fsm_enable_b.fsm16_en) == PROPERTY_ENABLE) {

            ret = fsm_data_rate_get(&fsm_odr);
            if (ret == 0) {
                switch (fsm_odr) {
                case LSM6DSR_ODR_FSM_12Hz5:
                    if (val == LSM6DSR_GY_ODR_OFF) {
                        odr_gy = LSM6DSR_GY_ODR_12Hz5;
                    } else {
                        odr_gy = val;
                    }
                    break;
                case LSM6DSR_ODR_FSM_26Hz:
                    if (val == LSM6DSR_GY_ODR_OFF) {
                        odr_gy = LSM6DSR_GY_ODR_26Hz;
                    } else if (val == LSM6DSR_GY_ODR_12Hz5) {
                        odr_gy = LSM6DSR_GY_ODR_26Hz;
                    } else {
                        odr_gy = val;
                    }
                    break;
                case LSM6DSR_ODR_FSM_52Hz:
                    if (val == LSM6DSR_GY_ODR_OFF) {
                        odr_gy = LSM6DSR_GY_ODR_52Hz;
                    } else if (val == LSM6DSR_GY_ODR_12Hz5) {
                        odr_gy = LSM6DSR_GY_ODR_52Hz;
                    } else if (val == LSM6DSR_GY_ODR_26Hz) {
                        odr_gy = LSM6DSR_GY_ODR_52Hz;
                    } else {
                        odr_gy = val;
                    }
                    break;
                case LSM6DSR_ODR_FSM_104Hz:
                    if (val == LSM6DSR_GY_ODR_OFF) {
                        odr_gy = LSM6DSR_GY_ODR_104Hz;
                    } else if (val == LSM6DSR_GY_ODR_12Hz5) {
                        odr_gy = LSM6DSR_GY_ODR_104Hz;
                    } else if (val == LSM6DSR_GY_ODR_26Hz) {
                        odr_gy = LSM6DSR_GY_ODR_104Hz;
                    } else if (val == LSM6DSR_GY_ODR_52Hz) {
                        odr_gy = LSM6DSR_GY_ODR_104Hz;
                    } else {
                        odr_gy = val;
                    }
                    break;
                default:
                    odr_gy = val;
                    break;
                }
            }
        }
    }
    if (ret == 0) {
        ret = read_reg(LSM6DSR_CTRL2_G, mem_ws_.ptr(), 1);
        structcpy(ctrl2_g);
    }
    if (ret == 0) {
        ctrl2_g.odr_g = (uint8_t)odr_gy;
        ret = write_reg(LSM6DSR_CTRL2_G, (uint8_t *)&ctrl2_g, 1);
    }
    return (ret == 0) ? LSM6DSR_OK : LSM6DSR_ERROR;
}

LSM6DSRStatusTypeDef LSM6DSRArray::gy_full_scale_set(lsm6dsr_fs_g_t val) {
    lsm6dsr_ctrl2_g_t ctrl2_g;
    int32_t ret;

    ret = read_reg(LSM6DSR_CTRL2_G, mem_ws_.ptr(), 1);
    structcpy(ctrl2_g);
    if (ret == 0) {
        ctrl2_g.fs_g = (uint8_t)val;
        ret = write_reg(LSM6DSR_CTRL2_G, (uint8_t *)&ctrl2_g, 1);
    }

    switch (val) {
    case LSM6DSR_125dps:
        gyro_sensitivity_ = LSM6DSR_GYRO_SENSITIVITY_FS_125DPS;
        break;
    case LSM6DSR_250dps:
        gyro_sensitivity_ = LSM6DSR_GYRO_SENSITIVITY_FS_250DPS;
        break;
    case LSM6DSR_500dps:
        gyro_sensitivity_ = LSM6DSR_GYRO_SENSITIVITY_FS_500DPS;
        break;
    case LSM6DSR_1000dps:
        gyro_sensitivity_ = LSM6DSR_GYRO_SENSITIVITY_FS_1000DPS;
        break;
    case LSM6DSR_2000dps:
        gyro_sensitivity_ = LSM6DSR_GYRO_SENSITIVITY_FS_2000DPS;
        break;
    case LSM6DSR_4000dps:
        gyro_sensitivity_ = LSM6DSR_GYRO_SENSITIVITY_FS_4000DPS;
        break;
    }
    return (ret == 0) ? LSM6DSR_OK : LSM6DSR_ERROR;
}

LSM6DSRStatusTypeDef LSM6DSRArray::gy_filter_lp1_set(uint8_t val) {
    lsm6dsr_ctrl4_c_t ctrl4_c;
    int32_t ret;

    ret = read_reg(LSM6DSR_CTRL4_C, mem_ws_.ptr(), 1);
    structcpy(ctrl4_c);
    if (ret == 0) {
        ctrl4_c.lpf1_sel_g = (uint8_t)val;
        ret = write_reg(LSM6DSR_CTRL4_C, (uint8_t *)&ctrl4_c, 1);
    }
    return (ret == 0) ? LSM6DSR_OK : LSM6DSR_ERROR;
}

LSM6DSRStatusTypeDef LSM6DSRArray::gy_lp1_bandwidth_set(lsm6dsr_ftype_t val) {
    lsm6dsr_ctrl6_c_t ctrl6_c;
    int32_t ret;

    ret = read_reg(LSM6DSR_CTRL6_C, mem_ws_.ptr(), 1);
    structcpy(ctrl6_c);
    if (ret == 0) {
        ctrl6_c.ftype = (uint8_t)val;
        ret = write_reg(LSM6DSR_CTRL6_C, (uint8_t *)&ctrl6_c, 1);
    }
    return (ret == 0) ? LSM6DSR_OK : LSM6DSR_ERROR;
}

LSM6DSRStatusTypeDef LSM6DSRArray::fsm_enable_get(lsm6dsr_emb_fsm_enable_t *val) {
    int32_t ret;

    ret = mem_bank_set(LSM6DSR_EMBEDDED_FUNC_BANK);
    if (ret == 0) {
        ret = read_reg(LSM6DSR_FSM_ENABLE_A, mem_ws_.ptr(), 1);
        structcpy(val->fsm_enable_a);
    }
    if (ret == 0) {
        ret = read_reg(LSM6DSR_FSM_ENABLE_B, mem_ws_.ptr(), 1);
        structcpy(val->fsm_enable_b);
    }
    if (ret == 0) {
        ret = mem_bank_set(LSM6DSR_USER_BANK);
    }
    return (ret == 0) ? LSM6DSR_OK : LSM6DSR_ERROR;
}

LSM6DSRStatusTypeDef LSM6DSRArray::mem_bank_set(lsm6dsr_reg_access_t val) {
    lsm6dsr_func_cfg_access_t func_cfg_access;
    int32_t ret;

    ret = read_reg(LSM6DSR_FUNC_CFG_ACCESS, mem_ws_.ptr(), 1);
    structcpy(func_cfg_access);
    if (ret == 0) {
        func_cfg_access.reg_access = (uint8_t)val;
        ret = write_reg(LSM6DSR_FUNC_CFG_ACCESS, (uint8_t *)&func_cfg_access, 1);
    }
    return (ret == 0) ? LSM6DSR_OK : LSM6DSR_ERROR;
}

LSM6DSRStatusTypeDef LSM6DSRArray::fsm_data_rate_get(lsm6dsr_fsm_odr_t *val) {
    lsm6dsr_emb_func_odr_cfg_b_t emb_func_odr_cfg_b;
    int32_t ret;

    ret = mem_bank_set(LSM6DSR_EMBEDDED_FUNC_BANK);

    if (ret == 0) {
        ret = read_reg(LSM6DSR_EMB_FUNC_ODR_CFG_B, mem_ws_.ptr(), 1);
        structcpy(emb_func_odr_cfg_b);
    }
    if (ret == 0) {
        ret = mem_bank_set(LSM6DSR_USER_BANK);
    }
    switch (emb_func_odr_cfg_b.fsm_odr) {
    case LSM6DSR_ODR_FSM_12Hz5:
        *val = LSM6DSR_ODR_FSM_12Hz5;
        break;
    case LSM6DSR_ODR_FSM_26Hz:
        *val = LSM6DSR_ODR_FSM_26Hz;
        break;
    case LSM6DSR_ODR_FSM_52Hz:
        *val = LSM6DSR_ODR_FSM_52Hz;
        break;
    case LSM6DSR_ODR_FSM_104Hz:
        *val = LSM6DSR_ODR_FSM_104Hz;
        break;
    default:
        *val = LSM6DSR_ODR_FSM_12Hz5;
        break;
    }
    return (ret == 0) ? LSM6DSR_OK : LSM6DSR_ERROR;
}

LSM6DSRStatusTypeDef LSM6DSRArray::enable_acc() {
    /* Check if the component is already enabled */
    if (is_acc_enabled_ == true) {
        return LSM6DSR_OK;
    }

    /* Output data rate selection. */
    if (xl_data_rate_set(acc_odr) != LSM6DSR_OK) {
        return LSM6DSR_ERROR;
    }

    is_acc_enabled_ = 1;

    return LSM6DSR_OK;
}

LSM6DSRStatusTypeDef LSM6DSRArray::update_acc_axes() {
    if (read_reg(LSM6DSR_OUTX_L_A, mem_ws_.ptr(), 6) != 0) {
        return LSM6DSR_ERROR;
    }

    for (size_t i = 0; i < NUM_SENSOR; i++) {
        sensor_data_[i].acc.raw_value[0] = (int16_t)((mem_ws_.data[i][1] << 8) | mem_ws_.data[i][0]);
        sensor_data_[i].acc.raw_value[1] = (int16_t)((mem_ws_.data[i][3] << 8) | mem_ws_.data[i][2]);
        sensor_data_[i].acc.raw_value[2] = (int16_t)((mem_ws_.data[i][5] << 8) | mem_ws_.data[i][4]);

        // sensor_data_[i].acc.raw_value[0] = (int32_t)((mem_ws_.data[i][1] << 8) | mem_ws_.data[i][0]) < 16;
        // sensor_data_[i].acc.raw_value[1] = (int32_t)((mem_ws_.data[i][3] << 8) | mem_ws_.data[i][2]) < 16;
        // sensor_data_[i].acc.raw_value[2] = (int32_t)((mem_ws_.data[i][5] << 8) | mem_ws_.data[i][4]) < 16;

        // sensor_data_[i].acc.value[0] = sensor_data_[i].acc.raw_value[0] * acc_sensitivity_;
        // sensor_data_[i].acc.value[1] = sensor_data_[i].acc.raw_value[1] * acc_sensitivity_;
        // sensor_data_[i].acc.value[2] = sensor_data_[i].acc.raw_value[2] * acc_sensitivity_;
    }

    return LSM6DSR_OK;
}

std::array<int16_t, 3> LSM6DSRArray::get_acc_axes_raw(size_t index) { return sensor_data_[index].acc.raw_value; }

// LSM6DSRArray::Vector3f LSM6DSRArray::calcu_scaled_acc_axes(size_t index) {
//     return sensor_data_[index].acc.raw_value * acc_sensitivity_;
// }

LSM6DSRStatusTypeDef LSM6DSRArray::enable_gyro() {
    /* Check if the component is already enabled */
    if (is_gyro_enabled_ == 1U) {
        return LSM6DSR_OK;
    }

    /* Output data rate selection. */
    if (gy_data_rate_set(gyro_odr) != LSM6DSR_OK) {
        return LSM6DSR_ERROR;
    }

    is_gyro_enabled_ = 1;

    return LSM6DSR_OK;
}

LSM6DSRStatusTypeDef LSM6DSRArray::update_gyro_axes() {
    if (read_reg(LSM6DSR_OUTX_L_G, mem_ws_.ptr(), 6) != 0) {
        return LSM6DSR_ERROR;
    }

    for (size_t i = 0; i < NUM_SENSOR; i++) {
        sensor_data_[i].gyro.raw_value[0] = (int16_t)((mem_ws_.data[i][1] << 8) | mem_ws_.data[i][0]);
        sensor_data_[i].gyro.raw_value[1] = (int16_t)((mem_ws_.data[i][3] << 8) | mem_ws_.data[i][2]);
        sensor_data_[i].gyro.raw_value[2] = (int16_t)((mem_ws_.data[i][5] << 8) | mem_ws_.data[i][4]);

        // sensor_data_[i].gyro.raw_value[0] = (int32_t)((mem_ws_.data[i][1] << 8) | mem_ws_.data[i][0]) << 16;
        // sensor_data_[i].gyro.raw_value[1] = (int32_t)((mem_ws_.data[i][3] << 8) | mem_ws_.data[i][2]) << 16;
        // sensor_data_[i].gyro.raw_value[2] = (int32_t)((mem_ws_.data[i][5] << 8) | mem_ws_.data[i][4]) << 16;

        // sensor_data_[i].gyro.value[0] = sensor_data_[i].gyro.raw_value[0] * gyro_sensitivity_;
        // sensor_data_[i].gyro.value[1] = sensor_data_[i].gyro.raw_value[1] * gyro_sensitivity_;
        // sensor_data_[i].gyro.value[2] = sensor_data_[i].gyro.raw_value[2] * gyro_sensitivity_;
    }

    return LSM6DSR_OK;
}

std::array<int16_t, 3> LSM6DSRArray::get_gyro_axes_raw(size_t index) { return sensor_data_[index].gyro.raw_value; }

// LSM6DSRArray::Vector3f LSM6DSRArray::calcu_scaled_gyro_axes(size_t index) {
//     return sensor_data_[index].gyro.raw_value * gyro_sensitivity_;
// }

LSM6DSRStatusTypeDef LSM6DSRArray::update_temperature() {
    if (read_reg(LSM6DSR_OUT_TEMP_L, mem_ws_.ptr(), 2) != 0) {
        return LSM6DSR_ERROR;
    }

    for (size_t i = 0; i < NUM_SENSOR; i++) {
        sensor_data_[i].temperature.raw_value = (int16_t)((mem_ws_.data[i][1] << 8) | mem_ws_.data[i][0]);
        // sensor_data_[i].temperature.value = sensor_data_[i].temperature.raw_value / 256.0f + 25.0f;
    }

    return LSM6DSR_OK;
}

// float LSM6DSRArray::get_temperature(size_t index) { return array_data_.temperature[index] / 256.0f + 25.0f; }
int16_t LSM6DSRArray::get_temperature_raw(size_t index) { return sensor_data_[index].temperature.raw_value; }
