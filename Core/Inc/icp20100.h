#ifndef INC_ICP20100_H_
#define INC_ICP20100_H_

#include "stm32h7xx_hal.h" // 引入 STM32 HAL 庫

// ICP-20100 I2C 從裝置位址 (7位元，假設 AD0 接腳為低電位)
#define ICP20100_I2C_ADDR       (0x63 << 1) // HAL 函式通常需要左移一位的8位元地址

// ICP-20100 重要暫存器位址
#define ICP20100_REG_DEVICE_ID  0x0C // 裝置 ID 暫存器
#define ICP20100_REG_VERSION    0xD3 // 版本暫存器 (用於區分 Version A/B)
#define ICP20100_REG_MODE_SELECT 0xC0 // 模式選擇暫存器
#define ICP20100_REG_MASTER_LOCK 0xBE // 主鎖定暫存器
#define ICP20100_REG_OTP_STATUS2 0xBF // OTP 狀態暫存器 2
#define ICP20100_REG_DEVICE_STATUS 0xCD // 裝置狀態暫存器

#define ICP20100_REG_PRESS_DATA_0 0xFA // 壓力數據 LSB
#define ICP20100_REG_PRESS_DATA_1 0xFB // 壓力數據
#define ICP20100_REG_PRESS_DATA_2 0xFC // 壓力數據 MSB (低4位有效)
#define ICP20100_REG_TEMP_DATA_0  0xFD // 溫度數據 LSB
#define ICP20100_REG_TEMP_DATA_1  0xFE // 溫度數據
#define ICP20100_REG_TEMP_DATA_2  0xFF // 溫度數據 MSB (低4位有效)

// 預期裝置 ID
#define ICP20100_EXPECTED_DEVICE_ID 0x63
// ASIC 版本 B 的值
#define ICP20100_ASIC_VERSION_B 0xB2

// 函式回傳狀態碼
typedef enum {
    ICP20100_OK = 0,
    ICP20100_ERROR_I2C,
    ICP20100_ERROR_ID_MISMATCH,
    ICP20100_ERROR_VERSION_A_UNSUPPORTED // 簡化範例，不完整支援 Version A 初始化
} ICP20100_StatusTypeDef;

// 函式原型宣告

/**
  * @brief  初始化 ICP-20100 感測器。
  * @param  hi2c: I2C_HandleTypeDef 指標，指向 I2C 控制代碼。
  * @retval ICP20100_StatusTypeDef 狀態碼。
  */
ICP20100_StatusTypeDef icp20100_init(I2C_HandleTypeDef *hi2c);

/**
  * @brief  從 ICP-20100 讀取大氣壓力與溫度數據。
  * @param  hi2c: I2C_HandleTypeDef 指標，指向 I2C 控制代碼。
  * @param  pressure: float 指標，用於儲存讀取到的大氣壓力值 (單位: Pa)。
  * @param  temperature: float 指標，用於儲存讀取到的溫度值 (單位: Celsius)。
  * @retval ICP20100_StatusTypeDef 狀態碼。
  */
ICP20100_StatusTypeDef icp20100_get_pressure_temp(I2C_HandleTypeDef *hi2c, float *pressure, float *temperature);

#endif /* INC_ICP20100_H_ */
