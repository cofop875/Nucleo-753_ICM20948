/*
 * icm20948_driver.h
 *
 *  Created on: May 9, 2025
 *      Author: User
 */

#ifndef INC_ICM20948_DRIVER_H_
#define INC_ICM20948_DRIVER_H_

#include "main.h"         // STM32CubeIDE 自動生成的主標頭檔，包含 HAL 驅動程式定義
#include <stdint.h>       // 標準整數類型定義

// HAL SPI Handle (由 main.c 或 stm32h7xx_hal_msp.c 初始化)
// 声明 SPI_HandleTypeDef 结构体变量 hspi1，用于 SPI1 通讯。
// extern 关键字表示该变量在其他文件中定义。
extern SPI_HandleTypeDef hspi1;

// --- ICM-20948 暫存器位址定義 ---
// --- 使用者庫 0 (User Bank 0) ---
#define ICM20948_WHO_AM_I       0x00  // WHO_AM_I 暫存器位址 (唯讀)，用於識別裝置，預期值 0xEA
#define ICM20948_USER_CTRL      0x03  // 使用者控制暫存器，用於啟用/禁用 I2C 主機介面、FIFO 等
#define ICM20948_PWR_MGMT_1     0x06  // 電源管理 1 暫存器，用於裝置喚醒、時脈選擇、睡眠模式
#define ICM20948_INT_PIN_CFG    0x0F  // 中斷引腳配置暫存器
#define ICM20948_INT_ENABLE_1   0x11  // 中斷致能 1 暫存器，用於致能各種中斷源 (如資料就緒)
#define ICM20948_INT_STATUS_1   0x1A  // 中斷狀態 1 暫存器 (唯讀)，指示資料就緒等中斷狀態
#define ICM20948_ACCEL_XOUT_H   0x2D  // 加速計 X 軸數據高位元組
#define ICM20948_ACCEL_XOUT_L   0x2E  // 加速計 X 軸數據低位元組
#define ICM20948_ACCEL_YOUT_H   0x2F  // 加速計 Y 軸數據高位元組
#define ICM20948_ACCEL_YOUT_L   0x30  // 加速計 Y 軸數據低位元組
#define ICM20948_ACCEL_ZOUT_H   0x31  // 加速計 Z 軸數據高位元組
#define ICM20948_ACCEL_ZOUT_L   0x32  // 加速計 Z 軸數據低位元組
#define ICM20948_GYRO_XOUT_H    0x33  // 陀螺儀 X 軸數據高位元組
#define ICM20948_GYRO_XOUT_L    0x34  // 陀螺儀 X 軸數據低位元組
#define ICM20948_GYRO_YOUT_H    0x35  // 陀螺儀 Y 軸數據高位元組
#define ICM20948_GYRO_YOUT_L    0x36  // 陀螺儀 Y 軸數據低位元組
#define ICM20948_GYRO_ZOUT_H    0x37  // 陀螺儀 Z 軸數據高位元組
#define ICM20948_GYRO_ZOUT_L    0x38  // 陀螺儀 Z 軸數據低位元組
#define ICM20948_EXT_SLV_SENS_DATA_00 0x3B // 外部從裝置感測器數據暫存器起始位址 (用於讀取磁力計數據)
#define ICM20948_REG_BANK_SEL   0x7F  // 暫存器庫選擇暫存器 (所有庫中皆可存取)

// --- 使用者庫 2 (User Bank 2) ---
#define ICM20948_GYRO_SMPLRT_DIV  0x00  // 陀螺儀取樣率分頻值
#define ICM20948_GYRO_CONFIG_1  0x01  // 陀螺儀配置 1 (FSR, DLPF)
#define ICM20948_ACCEL_SMPLRT_DIV_1 0x10 // 加速計取樣率分頻值高位元組
#define ICM20948_ACCEL_SMPLRT_DIV_2 0x11 // 加速計取樣率分頻值低位元組
#define ICM20948_ACCEL_CONFIG     0x14  // 加速計配置 (FSR, DLPF)

// --- 使用者庫 3 (User Bank 3) ---
#define ICM20948_I2C_MST_CTRL   0x01  // I2C 主機控制 (時脈頻率等)
#define ICM20948_I2C_SLV0_ADDR  0x03  // I2C 從裝置 0 位址
#define ICM20948_I2C_SLV0_REG   0x04  // I2C 從裝置 0 內部暫存器位址
#define ICM20948_I2C_SLV0_CTRL  0x05  // I2C 從裝置 0 傳輸控制
#define ICM20948_I2C_SLV0_DO    0x06  // 寫入 I2C 從裝置 0 的數據

// --- AK09916 (磁力計) 相關定義 ---
#define AK09916_I2C_ADDR      0x0C  // AK09916 的 I2C 從裝置位址
#define AK09916_WIA2          0x01  // AK09916 公司識別碼暫存器位址 (唯讀)，預期值 0x09
#define AK09916_ST1           0x10  // AK09916 狀態 1 暫存器 (DRDY 位元)
#define AK09916_HXL           0x11  // AK09916 X 軸磁場數據低位元組起始位址
// AK09916_HXH           0x12  // AK09916 X 軸磁場數據高位元組
// AK09916_HYL           0x13  // AK09916 Y 軸磁場數據低位元組
// AK09916_HYH           0x14  // AK09916 Y 軸磁場數據高位元組
// AK09916_HZL           0x15  // AK09916 Z 軸磁場數據低位元組
// AK09916_HZH           0x16  // AK09916 Z 軸磁場數據高位元組
#define AK09916_ST2           0x18  // AK09916 狀態 2 暫存器 (數據溢位，讀取此暫存器以觸發下次量測)
#define AK09916_CNTL2         0x31  // AK09916 控制暫存器 2 (操作模式設定)
#define AK09916_CNTL3         0x32  // AK09916 控制暫存器 3 (軟復位)

// --- 感測器原始數據陣列 (16位元帶正負號整數) ---
extern int16_t accel_raw[3]; // 加速計 X, Y, Z 軸原始數據
extern int16_t gyro_raw[3];  // 陀螺儀 X, Y, Z 軸原始數據
extern int16_t mag_raw[3];   // 磁力計 X, Y, Z 軸原始數據

// --- 感測器轉換後數據陣列 (浮點數) ---
extern float accel_g[3];   // 加速計 X, Y, Z 軸數據 (單位: g)
extern float gyro_dps[3]; // 陀螺儀 X, Y, Z 軸數據 (單位: dps - 度/秒)
extern float mag_uT[3];   // 磁力計 X, Y, Z 軸數據 (單位: µT - 微特斯拉)

// --- 靈敏度定義 (根據全量程範圍 FSR 設定) ---
#define ACCEL_SENSITIVITY_2G    16384.0f // 加速計 ±2g 量程下的靈敏度 (LSB/g)
#define ACCEL_SENSITIVITY_4G    8192.0f  // 加速計 ±4g 量程下的靈敏度 (LSB/g)
#define ACCEL_SENSITIVITY_8G    4096.0f  // 加速計 ±8g 量程下的靈敏度 (LSB/g)
#define ACCEL_SENSITIVITY_16G   2048.0f  // 加速計 ±16g 量程下的靈敏度 (LSB/g)

#define GYRO_SENSITIVITY_250DPS 131.0f   // 陀螺儀 ±250 dps 量程下的靈敏度 (LSB/dps)
#define GYRO_SENSITIVITY_500DPS 65.5f    // 陀螺儀 ±500 dps 量程下的靈敏度 (LSB/dps)
#define GYRO_SENSITIVITY_1000DPS 32.8f   // 陀螺儀 ±1000 dps 量程下的靈敏度 (LSB/dps)
#define GYRO_SENSITIVITY_2000DPS 16.4f   // 陀螺儀 ±2000 dps 量程下的靈敏度 (LSB/dps)

#define MAG_SENSITIVITY_UT_LSB  0.15f    // AK09916 磁力計的靈敏度 (µT/LSB)

// --- 函式原型宣告 ---

/**
 * @brief 選取 ICM-20948 的 SPI 片選 (CS) 引腳 (設為低電位)。
 */
void ICM20948_CS_Select(void);

/**
 * @brief 取消選取 ICM-20948 的 SPI 片選 (CS) 引腳 (設為高電位)。
 */
void ICM20948_CS_Deselect(void);

/**
 * @brief 透過 SPI 寫入單一位元組數據至 ICM-20948 指定暫存器。
 * @param reg_addr: 目標暫存器位址。
 * @param data:     要寫入的數據。
 * @return HAL_StatusTypeDef: HAL 操作狀態。
 */
HAL_StatusTypeDef ICM20948_WriteByte(uint8_t reg_addr, uint8_t data);

/**
 * @brief 透過 SPI 從 ICM-20948 指定暫存器讀取單一位元組數據。
 * @param reg_addr: 目標暫存器位址。
 * @param pData:    指向儲存讀取數據的緩衝區指標。
 * @return HAL_StatusTypeDef: HAL 操作狀態。
 */
HAL_StatusTypeDef ICM20948_ReadByte(uint8_t reg_addr, uint8_t *pData);

/**
 * @brief 透過 SPI 從 ICM-20948 指定起始暫存器連續讀取多個位元組數據。
 * @param reg_addr: 起始目標暫存器位址。
 * @param pData:    指向儲存讀取數據的緩衝區指標。
 * @param count:    要讀取的位元組數量。
 * @return HAL_StatusTypeDef: HAL 操作狀態。
 */
HAL_StatusTypeDef ICM20948_ReadBytes(uint8_t reg_addr, uint8_t *pData, uint16_t count);

/**
 * @brief 選擇 ICM-20948 的使用者庫 (User Bank)。
 * @param bank: 要選擇的庫編號 (0, 1, 2, 或 3)。
 * @return HAL_StatusTypeDef: HAL 操作狀態。
 */
HAL_StatusTypeDef ICM20948_SelectUserBank(uint8_t bank);

/**
 * @brief 透過 ICM-20948 的 I2C 主機介面，寫入單一位元組數據至 AK09916 磁力計指定暫存器。
 * @param reg_addr: AK09916 的目標暫存器位址。
 * @param data:     要寫入的數據。
 * @return HAL_StatusTypeDef: HAL 操作狀態。
 */
HAL_StatusTypeDef AK09916_WriteByteViaICM(uint8_t reg_addr, uint8_t data);

/**
 * @brief 透過 ICM-20948 的 I2C 主機介面，從 AK09916 磁力計指定暫存器讀取單一位元組數據。
 * @param reg_addr: AK09916 的目標暫存器位址。
 * @param pData:    指向儲存讀取數據的緩衝區指標。
 * @return HAL_StatusTypeDef: HAL 操作狀態。
 */
HAL_StatusTypeDef AK09916_ReadByteViaICM(uint8_t reg_addr, uint8_t *pData);

/**
 * @brief 初始化 ICM-20948 感測器 (包括加速計、陀螺儀和磁力計)。
 * @return HAL_StatusTypeDef: 初始化成功則返回 HAL_OK，否則返回 HAL_ERROR。
 */
HAL_StatusTypeDef ICM20948_Init(void);

/**
 * @brief 從 ICM-20948 讀取加速計和陀螺儀的原始數據。
 * @param pAccel: 指向儲存加速計原始數據的陣列 (int16_t[3])。
 * @param pGyro:  指向儲存陀螺儀原始數據的陣列 (int16_t[3])。
 * @return HAL_StatusTypeDef: HAL 操作狀態。
 */
HAL_StatusTypeDef ICM20948_ReadAccelGyroRaw(int16_t* pAccel, int16_t* pGyro);

/**
 * @brief 從 ICM-20948 (透過其 I2C 主機從 AK09916) 讀取磁力計的原始數據。
 * @param pMag:   指向儲存磁力計原始數據的陣列 (int16_t[3])。
 * @return HAL_StatusTypeDef: HAL 操作狀態。
 */
HAL_StatusTypeDef ICM20948_ReadMagRaw(int16_t* pMag);

#endif /* INC_ICM20948_DRIVER_H_ */
