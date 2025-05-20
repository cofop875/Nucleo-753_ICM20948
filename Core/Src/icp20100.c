#include "icp20100.h"
#include <stdio.h> // 用於除錯 printf

// I2C 通訊逾時時間
#define I2C_TIMEOUT 100 // 單位 ms

// 內部輔助函式原型
static ICP20100_StatusTypeDef icp20100_read_register(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *data);
static ICP20100_StatusTypeDef icp20100_write_register(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t data);
static ICP20100_StatusTypeDef icp20100_read_burst(I2C_HandleTypeDef *hi2c, uint8_t start_reg_addr, uint8_t *data, uint16_t len);

/**
  * @brief  讀取 ICP-20100 單一暫存器。
  * @param  hi2c: I2C 控制代碼。
  * @param  reg_addr: 要讀取的暫存器位址。
  * @param  data: 指向儲存讀取數據的緩衝區。
  * @retval ICP20100_StatusTypeDef 狀態碼。
  */
static ICP20100_StatusTypeDef icp20100_read_register(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t *data) {
    if (HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, data, 1, I2C_TIMEOUT)!= HAL_OK) {
        return ICP20100_ERROR_I2C;
    }
    return ICP20100_OK;
}

/**
  * @brief  寫入 ICP-20100 單一暫存器。
  * @param  hi2c: I2C 控制代碼。
  * @param  reg_addr: 要寫入的暫存器位址。
  * @param  data: 要寫入的數據。
  * @retval ICP20100_StatusTypeDef 狀態碼。
  */
static ICP20100_StatusTypeDef icp20100_write_register(I2C_HandleTypeDef *hi2c, uint8_t reg_addr, uint8_t data) {
    if (HAL_I2C_Mem_Write(hi2c, ICP20100_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, I2C_TIMEOUT)!= HAL_OK) {
        return ICP20100_ERROR_I2C;
    }
    return ICP20100_OK;
}

/**
  * @brief  從 ICP-20100 連續讀取多個暫存器。
  * @param  hi2c: I2C 控制代碼。
  * @param  start_reg_addr: 起始暫存器位址。
  * @param  data: 指向儲存讀取數據的緩衝區。
  * @param  len: 要讀取的位元組數。
  * @retval ICP20100_StatusTypeDef 狀態碼。
  */
static ICP20100_StatusTypeDef icp20100_read_burst(I2C_HandleTypeDef *hi2c, uint8_t start_reg_addr, uint8_t *data, uint16_t len) {
    if (HAL_I2C_Mem_Read(hi2c, ICP20100_I2C_ADDR, start_reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT)!= HAL_OK) {
        return ICP20100_ERROR_I2C;
    }
    return ICP20100_OK;
}

/**
  * @brief  初始化 ICP-20100 感測器。
  *         此為簡化版初始化，主要針對 Version B 晶片。
  *         完整的 Version A 初始化較複雜，涉及 OTP 讀取與配置，此處未完全實作。
  * @param  hi2c: I2C_HandleTypeDef 指標，指向 I2C 控制代碼。
  * @retval ICP20100_StatusTypeDef 狀態碼。
  */
ICP20100_StatusTypeDef icp20100_init(I2C_HandleTypeDef *hi2c) {
    uint8_t device_id = 0;
    uint8_t version = 0;
    uint8_t otp_status2 = 0;
    uint8_t mode_sync_status = 0;

    // 1. 檢查 I2C 通訊是否正常 (嘗試讀取 DEVICE_ID)
    if (icp20100_read_register(hi2c, ICP20100_REG_DEVICE_ID, &device_id)!= ICP20100_OK) {
        printf("Error: Failed to communicate with ICP-20100.\r\n");
        return ICP20100_ERROR_I2C;
    }

    // 2. 核對 DEVICE_ID
    if (device_id!= ICP20100_EXPECTED_DEVICE_ID) {
        printf("Error: ICP-20100 Device ID mismatch. Expected 0x%02X, Got 0x%02X\r\n", ICP20100_EXPECTED_DEVICE_ID, device_id);
        return ICP20100_ERROR_ID_MISMATCH;
    }
    printf("ICP-20100 Device ID: 0x%02X - OK\r\n", device_id);

    // 3. 讀取 ASIC 版本
    if (icp20100_read_register(hi2c, ICP20100_REG_VERSION, &version)!= ICP20100_OK) {
        printf("Error: Failed to read ICP-20100 version.\r\n");
        return ICP20100_ERROR_I2C;
    }
    printf("ICP-20100 ASIC Version: 0x%02X\r\n", version);

    // 4. 根據資料手冊 S6.5 Boot Sequence 進行簡化初始化
    //    對於 Version B (version == 0xB2)，大部分 OTP 配置已完成。
    //    對於 Version A (version == 0x00)，需要更複雜的 OTP 處理。
    //    本簡化範例假設 Version B 或 Version A 已完成 OTP boot。

    if (version == ICP20100_ASIC_VERSION_B) {
        printf("ICP-20100 Version B detected. Simplified initialization.\r\n");
        // Version B 通常不需要額外的 OTP 載入步驟
    } else if (version == 0x00) { // Version A
        printf("ICP-20100 Version A detected. Checking BOOT_UP_STATUS.\r\n");
        // 檢查 BOOT_UP_STATUS (OTP_STATUS2)
        if (icp20100_read_register(hi2c, ICP20100_REG_OTP_STATUS2, &otp_status2)!= ICP20100_OK) {
            return ICP20100_ERROR_I2C;
        }
        if (!(otp_status2 & 0x01)) {
            // BOOT_UP_STATUS 為 0，表示可能需要完整的 OTP 初始化序列。
            // 由於使用者要求程式盡量簡單，此處不實作完整的 Version A OTP 序列。
            // 實際應用中，應參考資料手冊 S6.5 完成 Version A 的初始化。
            printf("Warning: ICP-20100 Version A may require full OTP initialization (BOOT_UP_STATUS is 0).\r\n");
            // return ICP20100_ERROR_VERSION_A_UNSUPPORTED; // 可選擇在此回傳錯誤
        } else {
            printf("ICP-20100 Version A BOOT_UP_STATUS is 1. Assuming OTP config is done.\r\n");
        }
    } else {
        printf("Warning: Unknown ICP-20100 ASIC Version: 0x%02X\r\n", version);
        // 可視為錯誤或繼續嘗試
    }

    // 5. 解鎖主暫存器 (寫入任意值，然後寫入 0x00 以鎖定)
    //    根據資料手冊 S6.5，初始化序列中會操作 MASTER_LOCK。
    //    為確保可配置模式，先解鎖。
    if (icp20100_write_register(hi2c, ICP20100_REG_MASTER_LOCK, 0x01)!= ICP20100_OK) { // 寫入非0值解鎖
         printf("Error: Failed to unlock master registers.\r\n");
        return ICP20100_ERROR_I2C;
    }

    // 6. 等待模式同步狀態 (MODE_SYNC_STATUS in DEVICE_STATUS) 變為 1
    //    根據資料手冊 S6.6，寫入 MODE_SELECT 前，MODE_SYNC_STATUS 需為 1。
    uint8_t retry_count = 10;
    do {
        if (icp20100_read_register(hi2c, ICP20100_REG_DEVICE_STATUS, &mode_sync_status)!= ICP20100_OK) {
            return ICP20100_ERROR_I2C;
        }
        if (mode_sync_status & 0x01) break; // MODE_SYNC_STATUS is bit 0
        HAL_Delay(10); // 短暫延遲
        retry_count--;
    } while (retry_count > 0);

    if (!(mode_sync_status & 0x01)) {
        printf("Error: Timed out waiting for MODE_SYNC_STATUS.\r\n");
        // 即使超時，仍嘗試配置模式，某些情況下可能仍能運作
        // return ICP20100_ERROR_I2C;
    } else {
        printf("MODE_SYNC_STATUS is 1.\r\n");
    }


    // 7. 設定操作模式為 Mode0 (連續量測, 壓力與溫度, 約 25Hz ODR)
    //    MODE_SELECT (0xC0):
    //    Bits 7:5 MEAS_CONFIG = 000 (Mode0)
    //    Bit  4 FORCED_MEAS_TRIGGER = 0 (N/A for continuous)
    //    Bit  3 MEAS_MODE = 1 (Continuous measurements)
    //    Bit  2 POWER_MODE = 0 (Normal mode: active during meas, else standby)
    //    Bits 1:0 FIFO_READOUT_MODE = 00 (Pressure first)
    //    Value = 0b00001000 = 0x08
    uint8_t mode_select_val = 0x08;
    if (icp20100_write_register(hi2c, ICP20100_REG_MODE_SELECT, mode_select_val)!= ICP20100_OK) {
        printf("Error: Failed to set measurement mode.\r\n");
        return ICP20100_ERROR_I2C;
    }
    printf("ICP-20100 set to Mode0 continuous measurement.\r\n");

    // 8. 鎖定主暫存器 (寫入 0x00)
    if (icp20100_write_register(hi2c, ICP20100_REG_MASTER_LOCK, 0x00)!= ICP20100_OK) {
        printf("Error: Failed to lock master registers.\r\n");
        return ICP20100_ERROR_I2C;
    }

    HAL_Delay(10); // 等待模式穩定

    return ICP20100_OK;
}

/**
  * @brief  從 ICP-20100 讀取大氣壓力與溫度數據。
  * @param  hi2c: I2C_HandleTypeDef 指標，指向 I2C 控制代碼。
  * @param  pressure: float 指標，用於儲存讀取到的大氣壓力值 (單位: Pa)。
  * @param  temperature: float 指標，用於儲存讀取到的溫度值 (單位: Celsius)。
  * @retval ICP20100_StatusTypeDef 狀態碼。
  */
ICP20100_StatusTypeDef icp20100_get_pressure_temp(I2C_HandleTypeDef *hi2c, float *pressure, float *temperature) {
    uint8_t raw_data; // 用於儲存壓力 (3 bytes) 和溫度 (3 bytes) 的原始數據

    // 連續讀取 6 個位元組的數據 (PRESS_DATA_0 到 TEMP_DATA_2)
    // 順序：P0, P1, P2, T0, T1, T2
    if (icp20100_read_burst(hi2c, ICP20100_REG_PRESS_DATA_0, raw_data, 6)!= ICP20100_OK) {
        printf("Error: Failed to read P/T data burst.\r\n");
        return ICP20100_ERROR_I2C;
    }

    // 解析 20 位元壓力數據 (二補數)
    // P_OUT[19:0] = {PRESS_DATA_2[3:0], PRESS_DATA_1[7:0], PRESS_DATA_0[7:0]}
    int32_t p_out_raw = ((int32_t)(raw_data & 0x0F) << 16) | \
                        ((int32_t)raw_data << 8) | \
                        ((int32_t)raw_data);

    // 符號擴展 (如果第 20 位元為 1，則為負數)
    if (p_out_raw & (1 << 19)) {
        p_out_raw |= ~((1 << 20) - 1); // 將高位置為 1 以進行符號擴展
    }

    // 解析 20 位元溫度數據 (二補數)
    // T_OUT[19:0] = {TEMP_DATA_2[3:0], TEMP_DATA_1[7:0], TEMP_DATA_0[7:0]}
    int32_t t_out_raw = ((int32_t)(raw_data & 0x0F) << 16) | \
                        ((int32_t)raw_data << 8) | \
                        ((int32_t)raw_data);

    // 符號擴展
    if (t_out_raw & (1 << 19)) {
        t_out_raw |= ~((1 << 20) - 1);
    }

    // 轉換為實際物理值
    // 壓力轉換公式: P_Pa = * 1000
    *pressure = ((((float)p_out_raw / (float)(1 << 17)) * 40.0f) + 70.0f) * 1000.0f;

    // 溫度轉換公式: T_C = (T_OUT / 2^18) * 65 + 25
    *temperature = (((float)t_out_raw / (float)(1 << 18)) * 65.0f) + 25.0f;

    return ICP20100_OK;
}
