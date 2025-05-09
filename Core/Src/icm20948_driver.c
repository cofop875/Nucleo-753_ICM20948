/*
 * icm20948_driver.c
 *
 * Created on: May 9, 2025
 * Author: User
 */

#include "icm20948_driver.h"
#include "main.h" // For CS Pin definitions if made there, and HAL_Delay

// SPI Timeout
#define SPI_TIMEOUT 100 // ms

// --- Chip Select Pin Definition (User must adapt this) ---
// These should ideally be defined in main.h or a project configuration file
// if set up via STM32CubeMX with these specific names.
// Example:
#ifndef ICM_CS_GPIO_Port
#define ICM_CS_GPIO_Port GPIOB // Example GPIO Port
#endif
#ifndef ICM_CS_Pin
#define ICM_CS_Pin GPIO_PIN_12   // Example GPIO Pin
#endif

// --- Global sensor data arrays (defined here) ---
int16_t accel_raw[3]; // Accelerometer X, Y, Z axis raw data
int16_t gyro_raw[3];  // Gyroscope X, Y, Z axis raw data
int16_t mag_raw[3];   // Magnetometer X, Y, Z axis raw data

float accel_g[3];   // Accelerometer X, Y, Z axis data (unit: g)
float gyro_dps[3]; // Gyroscope X, Y, Z axis data (unit: dps - degrees/second)
float mag_uT[3];   // Magnetometer X, Y, Z axis data (unit: µT - microTesla)

// --- Sensitivity values (can be updated based on FSR settings in Init) ---
static float current_accel_sensitivity = ACCEL_SENSITIVITY_2G;
static float current_gyro_sensitivity = GYRO_SENSITIVITY_250DPS;
#define CURRENT_MAG_SENSITIVITY MAG_SENSITIVITY_UT_LSB


/**
 * @brief Selects ICM-20948 SPI Chip Select (CS) pin (set to low).
 */
void ICM20948_CS_Select(void) {
    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);
    // A small delay might be needed after CS assert for some devices/speeds
    // for (volatile int i = 0; i < 10; i++); // Basic short delay
}

/**
 * @brief Deselects ICM-20948 SPI Chip Select (CS) pin (set to high).
 */
void ICM20948_CS_Deselect(void) {
    HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);
    // A small delay might be needed after CS de-assert
    // for (volatile int i = 0; i < 10; i++);
}

/**
 * @brief Writes a single byte to the specified register of ICM-20948 via SPI.
 * @param reg_addr: Target register address.
 * @param data: Data to be written.
 * @return HAL_StatusTypeDef: HAL operation status.
 */
HAL_StatusTypeDef ICM20948_WriteByte(uint8_t reg_addr, uint8_t data) {
    uint8_t tx_data[2];
    tx_data[0] = reg_addr & 0x7F; // MSB=0 for write
    tx_data[1] = data;

    ICM20948_CS_Select();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, tx_data, 2, SPI_TIMEOUT);
    ICM20948_CS_Deselect();
    return status;
}

/**
 * @brief Reads a single byte from the specified register of ICM-20948 via SPI.
 * @param reg_addr: Target register address.
 * @param pData: Pointer to store the read data.
 * @return HAL_StatusTypeDef: HAL operation status.
 */
HAL_StatusTypeDef ICM20948_ReadByte(uint8_t reg_addr, uint8_t *pData) {
    uint8_t tx_byte = reg_addr | 0x80; // MSB=1 for read
    uint8_t rx_byte;

    ICM20948_CS_Select();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, &tx_byte, &rx_byte, 1, SPI_TIMEOUT); // Send reg addr
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(&hspi1, pData, 1, SPI_TIMEOUT); // Receive data
    }
    ICM20948_CS_Deselect();
    return status;
}

/**
 * @brief Reads multiple bytes from ICM-20948 starting from reg_addr via SPI.
 * @param reg_addr: Starting target register address.
 * @param pData: Pointer to buffer to store read data.
 * @param count: Number of bytes to read.
 * @return HAL_StatusTypeDef: HAL operation status.
 */
HAL_StatusTypeDef ICM20948_ReadBytes(uint8_t reg_addr, uint8_t *pData, uint16_t count) {
    uint8_t tx_byte = reg_addr | 0x80; // MSB=1 for read

    ICM20948_CS_Select();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, &tx_byte, 1, SPI_TIMEOUT); // Send reg addr
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(&hspi1, pData, count, SPI_TIMEOUT); // Receive data
    }
    ICM20948_CS_Deselect();
    return status;
}

/**
 * @brief Selects the user bank on ICM-20948.
 * @param bank: User bank number (0, 1, 2, or 3).
 * @return HAL_StatusTypeDef: HAL operation status.
 */
HAL_StatusTypeDef ICM20948_SelectUserBank(uint8_t bank) {
    if (bank > 3) return HAL_ERROR;
    return ICM20948_WriteByte(ICM20948_REG_BANK_SEL, (bank << 4));
}

/**
 * @brief Writes a byte to AK09916 magnetometer via ICM-20948's I2C Master interface.
 * @param reg_addr: AK09916 target register address.
 * @param data: Data to write.
 * @return HAL_StatusTypeDef: HAL operation status.
 */
HAL_StatusTypeDef AK09916_WriteByteViaICM(uint8_t reg_addr, uint8_t data) {
    HAL_StatusTypeDef status;

    status = ICM20948_SelectUserBank(3);
    if (status != HAL_OK) return status;

    status = ICM20948_WriteByte(ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR); // Slave address for write
    if (status != HAL_OK) { ICM20948_SelectUserBank(0); return status; }

    status = ICM20948_WriteByte(ICM20948_I2C_SLV0_REG, reg_addr); // Register to write to
    if (status != HAL_OK) { ICM20948_SelectUserBank(0); return status; }

    status = ICM20948_WriteByte(ICM20948_I2C_SLV0_DO, data); // Data to write
    if (status != HAL_OK) { ICM20948_SelectUserBank(0); return status; }

    status = ICM20948_WriteByte(ICM20948_I2C_SLV0_CTRL, 0x81); // Enable SLV0, 1 byte transfer
    if (status != HAL_OK) { ICM20948_SelectUserBank(0); return status; }

    HAL_Delay(1); // Wait for I2C transaction to complete.
                  // A more robust method would be to check I2C_MST_STATUS or an interrupt.

    // Check for NACK (optional, requires reading I2C_MST_STATUS)
    // uint8_t i2c_mst_status;
    // ICM20948_ReadByte(0x00, &i2c_mst_status); // I2C_MST_STATUS in UB3
    // if (i2c_mst_status & 0x10) status = HAL_ERROR; // I2C_SLV0_NACK

    status = ICM20948_SelectUserBank(0);
    return status;
}

/**
 * @brief Reads a byte from AK09916 magnetometer via ICM-20948's I2C Master interface.
 * @param reg_addr: AK09916 target register address.
 * @param pData: Pointer to store read data.
 * @return HAL_StatusTypeDef: HAL operation status.
 */
HAL_StatusTypeDef AK09916_ReadByteViaICM(uint8_t reg_addr, uint8_t *pData) {
    HAL_StatusTypeDef status;

    status = ICM20948_SelectUserBank(3);
    if (status != HAL_OK) return status;

    status = ICM20948_WriteByte(ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80); // Slave address for read
    if (status != HAL_OK) { ICM20948_SelectUserBank(0); return status; }

    status = ICM20948_WriteByte(ICM20948_I2C_SLV0_REG, reg_addr); // Register to read from
    if (status != HAL_OK) { ICM20948_SelectUserBank(0); return status; }

    status = ICM20948_WriteByte(ICM20948_I2C_SLV0_CTRL, 0x81); // Enable SLV0, 1 byte read
    if (status != HAL_OK) { ICM20948_SelectUserBank(0); return status; }

    HAL_Delay(2); // Wait for I2C transaction and data to appear in EXT_SLV_SENS_DATA

    status = ICM20948_SelectUserBank(0); // Switch to UB0 to read EXT_SLV_SENS_DATA
    if (status != HAL_OK) return status;

    status = ICM20948_ReadByte(ICM20948_EXT_SLV_SENS_DATA_00, pData);
    return status;
}


/**
 * @brief Initializes the ICM-20948 sensor.
 * @return HAL_StatusTypeDef: HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef ICM20948_Init(void) {
    HAL_StatusTypeDef status;
    uint8_t who_am_i_val;
    uint8_t ak09916_wia_val;

    // 0. Make sure CS is high initially
    ICM20948_CS_Deselect();
    HAL_Delay(10); // Power-on delay

    // 1. Select User Bank 0
    status = ICM20948_SelectUserBank(0);
    if (status != HAL_OK) return status;

    // 2. Check WHO_AM_I
    status = ICM20948_ReadByte(ICM20948_WHO_AM_I, &who_am_i_val);
    if (status != HAL_OK || who_am_i_val != 0xEA) {
        return HAL_ERROR; // WHO_AM_I check failed
    }

    // 3. Reset device
    status = ICM20948_WriteByte(ICM20948_PWR_MGMT_1, 0x80); // Set DEVICE_RESET bit
    if (status != HAL_OK) return status;
    HAL_Delay(100); // Wait for reset to complete

    // 4. Wake up sensor and set clock source to auto select
    status = ICM20948_WriteByte(ICM20948_PWR_MGMT_1, 0x01); // CLKSEL = 1 (Auto), SLEEP = 0
    if (status != HAL_OK) return status;
    HAL_Delay(50);

    // 5. Disable I2C Master interface initially if not needed yet by AK09916 setup
    //    Enable SPI interface (done by default, this register can disable it for I2C only mode)
    //    Ensure DMP is disabled
    status = ICM20948_WriteByte(ICM20948_USER_CTRL, 0x00); // Clear I2C_MST_EN, DMP_EN etc.
    if (status != HAL_OK) return status;

    // --- Configure Accelerometer and Gyroscope (User Bank 2) ---
    status = ICM20948_SelectUserBank(2);
    if (status != HAL_OK) return status;

    // 6. Configure Gyroscope:
    //    FS_SEL = 0 (+/-250 dps), GYRO_DLPFCFG = 1 (196.6Hz BW, 1.1kHz Fs_internal)
    //    Update current_gyro_sensitivity accordingly
    current_gyro_sensitivity = GYRO_SENSITIVITY_250DPS;
    status = ICM20948_WriteByte(ICM20948_GYRO_CONFIG_1, (0x00 << 1) | (0x01 << 3)); // FS_SEL=0, DLPFCFG=1
    if (status != HAL_OK) { ICM20948_SelectUserBank(0); return status; }
    //    Set Gyro Sample Rate Divider: ODR = 1.1kHz / (1 + GYRO_SMPLRT_DIV)
    //    For ~100Hz: 1100 / (1+DIV) = 100 => 1+DIV=11 => DIV=10 (0x0A)
    status = ICM20948_WriteByte(ICM20948_GYRO_SMPLRT_DIV, 0x0A);
    if (status != HAL_OK) { ICM20948_SelectUserBank(0); return status; }

    // 7. Configure Accelerometer:
    //    ACCEL_FS_SEL = 0 (+/-2g), ACCEL_DLPFCFG = 1 (246Hz BW, 1.125kHz Fs_internal)
    //    Update current_accel_sensitivity accordingly
    current_accel_sensitivity = ACCEL_SENSITIVITY_2G;
    status = ICM20948_WriteByte(ICM20948_ACCEL_CONFIG, (0x00 << 1) | (0x01 << 3)); // ACCEL_FS_SEL=0, ACCEL_DLPFCFG=1
    if (status != HAL_OK) { ICM20948_SelectUserBank(0); return status; }
    //    Set Accel Sample Rate Divider: ODR = 1.125kHz / (1 + ACCEL_SMPLRT_DIV)
    //    For ~100Hz: 1125 / (1+DIV) = 100 => 1+DIV=11.25 => DIV=10 (0x000A)
    status = ICM20948_WriteByte(ICM20948_ACCEL_SMPLRT_DIV_1, 0x00); // High byte
    if (status != HAL_OK) { ICM20948_SelectUserBank(0); return status; }
    status = ICM20948_WriteByte(ICM20948_ACCEL_SMPLRT_DIV_2, 0x0A); // Low byte
    if (status != HAL_OK) { ICM20948_SelectUserBank(0); return status; }

    // Return to User Bank 0
    status = ICM20948_SelectUserBank(0);
    if (status != HAL_OK) return status;

    // --- Initialize AK09916 Magnetometer ---
    // 8. Enable I2C Master interface on ICM20948
    uint8_t user_ctrl_val;
    status = ICM20948_ReadByte(ICM20948_USER_CTRL, &user_ctrl_val);
    if (status != HAL_OK) return status;
    status = ICM20948_WriteByte(ICM20948_USER_CTRL, user_ctrl_val | 0x20); // Set I2C_MST_EN
    if (status != HAL_OK) return status;
    HAL_Delay(10);

    // 9. Configure I2C Master Clock Speed (User Bank 3)
    status = ICM20948_SelectUserBank(3);
    if (status != HAL_OK) { ICM20948_SelectUserBank(0); return status; }
    //    Set I2C_MST_CLK to ~400 kHz. Value 0x07 -> 345.6 kHz, 50% duty cycle
    status = ICM20948_WriteByte(ICM20948_I2C_MST_CTRL, 0x07);
    if (status != HAL_OK) { ICM20948_SelectUserBank(0); return status; }
    // Return to User Bank 0 for AK09916 communication functions which handle bank switching
    status = ICM20948_SelectUserBank(0);
    if (status != HAL_OK) return status;

    // 10. Reset AK09916
    status = AK09916_WriteByteViaICM(AK09916_CNTL3, 0x01); // Soft Reset
    if (status != HAL_OK) return status;
    HAL_Delay(100); // Wait for AK09916 reset

    // 11. Check AK09916 WIA2 (Who I Am / Company ID)
    status = AK09916_ReadByteViaICM(AK09916_WIA2, &ak09916_wia_val);
    if (status != HAL_OK || ak09916_wia_val != 0x09) { // Expected 0x48 for ID1, 0x09 for ID2
        return HAL_ERROR; // AK09916 WIA2 check failed
    }

    // 12. Set AK09916 to Continuous Measurement Mode
    //     Mode 4: 100Hz (0x08)
    status = AK09916_WriteByteViaICM(AK09916_CNTL2, 0x08);
    if (status != HAL_OK) return status;
    HAL_Delay(10); // Delay after mode set

    // 13. Configure SLV0 to continuously read magnetometer data
    status = ICM20948_SelectUserBank(3);
    if (status != HAL_OK) { ICM20948_SelectUserBank(0); return status; }

    //    Set SLV0 address to AK09916 for reading
    status = ICM20948_WriteByte(ICM20948_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80);
    if (status != HAL_OK) { ICM20948_SelectUserBank(0); return status; }
    //    Set SLV0 register to start reading from AK09916_HXL
    status = ICM20948_WriteByte(ICM20948_I2C_SLV0_REG, AK09916_HXL);
    if (status != HAL_OK) { ICM20948_SelectUserBank(0); return status; }
    //    Enable SLV0 to read 7 bytes (HXL to HZH, plus ST2)
    status = ICM20948_WriteByte(ICM20948_I2C_SLV0_CTRL, 0x80 | 7); // I2C_SLV0_EN | 7 bytes (0x87)
    if (status != HAL_OK) { ICM20948_SelectUserBank(0); return status; }

    // Return to User Bank 0
    status = ICM20948_SelectUserBank(0);
    if (status != HAL_OK) return status;

    HAL_Delay(50); // Allow I2C master to make its first read.

    return HAL_OK;
}


/**
 * @brief Reads raw accelerometer and gyroscope data from ICM-20948.
 * @param pAccel: Pointer to array to store raw accelerometer data (int16_t[3]).
 * @param pGyro:  Pointer to array to store raw gyroscope data (int16_t[3]).
 * @return HAL_StatusTypeDef: HAL operation status.
 */
HAL_StatusTypeDef ICM20948_ReadAccelGyroRaw(int16_t* pAccel, int16_t* pGyro) {
    uint8_t data_buf[12]; // 6 bytes for accel, 6 bytes for gyro
    HAL_StatusTypeDef status;

    status = ICM20948_SelectUserBank(0); // Data is in User Bank 0
    if (status != HAL_OK) return status;

    // Read ACCEL_XOUT_H to GYRO_ZOUT_L (12 bytes)
    status = ICM20948_ReadBytes(ICM20948_ACCEL_XOUT_H, data_buf, 12);
    if (status != HAL_OK) return status;

    // Parse accelerometer data
    pAccel[0] = (int16_t)((data_buf[0] << 8) | data_buf[1]);  // Accel X
    pAccel[1] = (int16_t)((data_buf[2] << 8) | data_buf[3]);  // Accel Y
    pAccel[2] = (int16_t)((data_buf[4] << 8) | data_buf[5]);  // Accel Z

    // Parse gyroscope data
    pGyro[0] = (int16_t)((data_buf[6] << 8) | data_buf[7]);   // Gyro X
    pGyro[1] = (int16_t)((data_buf[8] << 8) | data_buf[9]);   // Gyro Y
    pGyro[2] = (int16_t)((data_buf[10] << 8) | data_buf[11]); // Gyro Z

    return HAL_OK;
}

/**
 * @brief Reads raw magnetometer data from AK09916 (via ICM-20948 I2C Master).
 * @param pMag:   Pointer to array to store raw magnetometer data (int16_t[3]).
 * @return HAL_StatusTypeDef: HAL operation status.
 */
HAL_StatusTypeDef ICM20948_ReadMagRaw(int16_t* pMag) {
    uint8_t mag_data_buf[7]; // HXL, HXH, HYL, HYH, HZL, HZH, ST2
    HAL_StatusTypeDef status;

    // Magnetometer data is read by I2C_SLV0 and placed in EXT_SLV_SENS_DATA registers (UB0)
    // This assumes I2C_SLV0 is configured in Init to continuously poll the magnetometer

    // Optional: Check DRDY bit in AK09916_ST1 register before reading from EXT_SLV_SENS_DATA.
    // This would require an explicit single read via AK09916_ReadByteViaICM(AK09916_ST1, &st1_val);
    // For simplicity with continuous polling by SLV0, we read directly.
    // The ST2 register is read as the last byte to ensure AK09916 continues measurements.

    status = ICM20948_SelectUserBank(0); // Data is in User Bank 0
    if (status != HAL_OK) return status;

    status = ICM20948_ReadBytes(ICM20948_EXT_SLV_SENS_DATA_00, mag_data_buf, 7);
    if (status != HAL_OK) return status;

    // Check ST1 DRDY (Data Ready) from the fetched data if needed.
    // AK09916_ST1 is usually fetched into EXT_SLV_SENS_DATA if SLV0 is configured to read it.
    // However, our current setup reads HXL to ST2. ST1 is not part of this continuous read.
    // A robust way for continuous mode is to ensure ST2 is read.
    // If mag_data_buf[6] (ST2) shows overflow (HOFL bit), data might be compromised.

    // Parse magnetometer data (Little Endian from AK09916)
    // HXL (LSB), HXH (MSB)
    pMag[0] = (int16_t)((mag_data_buf[1] << 8) | mag_data_buf[0]); // Mag X
    pMag[1] = (int16_t)((mag_data_buf[3] << 8) | mag_data_buf[2]); // Mag Y
    pMag[2] = (int16_t)((mag_data_buf[5] << 8) | mag_data_buf[4]); // Mag Z

    // The 7th byte read (mag_data_buf[6]) is AK09916_ST2. Reading it is necessary
    // for continuous measurement modes of AK09916.

    return HAL_OK;
}

/**
 * @brief Converts raw accelerometer data to g.
 */
void ICM20948_ConvertAccelRawToG(void) {
    for (int i = 0; i < 3; i++) {
        accel_g[i] = (float)accel_raw[i] / current_accel_sensitivity;
    }
}

/**
 * @brief Converts raw gyroscope data to dps.
 */
void ICM20948_ConvertGyroRawToDps(void) {
    for (int i = 0; i < 3; i++) {
        gyro_dps[i] = (float)gyro_raw[i] / current_gyro_sensitivity;
    }
}

/**
 * @brief Converts raw magnetometer data to microTesla.
 */
void ICM20948_ConvertMagRawToUT(void) {
    for (int i = 0; i < 3; i++) {
        mag_uT[i] = (float)mag_raw[i] * CURRENT_MAG_SENSITIVITY;
    }
}
