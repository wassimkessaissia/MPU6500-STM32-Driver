/**
  ******************************************************************************
  * @file           : example_usage.c
  * @brief          : Example showing how to use MPU6500 driver
  * @author         : Wassim Kessaissia
  ******************************************************************************
  */

#include "main.h"
#include "mpu6500.h"
#include <stdio.h>

/* ==================== EXAMPLE 1: BASIC READING ==================== */

void example_basic_reading(void) {
    MPU6500_Data sensor_data;
    int16_t acc_x, acc_y, acc_z;
    int16_t gyr_x, gyr_y, gyr_z;
    
    // Initialize sensor
    MPU6500_Init();
    
    // Verify device
    uint8_t device_id = MPU6500_WhoAmI();
    if (device_id != MPU6050_WHO_AM_I && device_id != MPU6500_WHO_AM_I) {
        printf("Error: Wrong sensor!\n");
        return;
    }
    
    // Calibrate (keep sensor still!)
    MPU6500_Calibrate();
    
    // Read and display data
    while (1) {
        // Read raw data
        MPU6500_ReadAll(&sensor_data);
        
        // Get calibrated values
        MPU6500_GetCalibratedAccel(&sensor_data, &acc_x, &acc_y, &acc_z);
        MPU6500_GetCalibratedGyro(&sensor_data, &gyr_x, &gyr_y, &gyr_z);
        
        // Display
        printf("ACC: %6d %6d %6d  ", acc_x, acc_y, acc_z);
        printf("GYR: %6d %6d %6d\n", gyr_x, gyr_y, gyr_z);
        
        HAL_Delay(100);
    }
}

/* ==================== EXAMPLE 2: WITH MAIN FUNCTION ==================== */

int main(void) {
    // HAL initialization
    HAL_Init();
    SystemClock_Config();
    
    // Initialize peripherals
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    
    printf("=== MPU6500 Example ===\n\n");
    
    // Scan I2C bus
    uint8_t address = MPU6500_Scan();
    if (address == 0) {
        printf("No sensor found!\n");
        while (1);
    }
    
    // Initialize MPU6500
    if (MPU6500_Init() != 0) {
        printf("Initialization failed!\n");
        while (1);
    }
    
    // Identify device
    uint8_t id = MPU6500_WhoAmI();
    if (id == MPU6050_WHO_AM_I) {
        printf("Device: MPU6050\n");
    } else if (id == MPU6500_WHO_AM_I) {
        printf("Device: MPU6500\n");
    } else {
        printf("Unknown device: 0x%02X\n", id);
        while (1);
    }
    
    // Calibrate sensor
    printf("\nStarting calibration in 3 seconds...\n");
    printf("Place sensor on flat surface and don't move it!\n");
    HAL_Delay(3000);
    
    MPU6500_Calibrate();
    
    printf("Starting continuous reading...\n\n");
    
    // Continuous reading
    MPU6500_Data sensor_data;
    int16_t acc_x, acc_y, acc_z;
    int16_t gyr_x, gyr_y, gyr_z;
    
    while (1) {
        // Read sensor
        MPU6500_ReadAll(&sensor_data);
        
        // Get calibrated data
        MPU6500_GetCalibratedAccel(&sensor_data, &acc_x, &acc_y, &acc_z);
        MPU6500_GetCalibratedGyro(&sensor_data, &gyr_x, &gyr_y, &gyr_z);
        
        // Display
        printf("ACC: %6d %6d %6d  ", acc_x, acc_y, acc_z);
        printf("GYR: %6d %6d %6d\n", gyr_x, gyr_y, gyr_z);
        
        HAL_Delay(100);  // 10 Hz update rate
    }
}

/* ==================== EXAMPLE 3: RAW VALUES (NO CALIBRATION) ==================== */

void example_raw_values(void) {
    MPU6500_Data sensor_data;
    
    MPU6500_Init();
    
    while (1) {
        MPU6500_ReadAll(&sensor_data);
        
        printf("Raw ACC: %6d %6d %6d  ", 
               sensor_data.accel_x, 
               sensor_data.accel_y, 
               sensor_data.accel_z);
        
        printf("Raw GYR: %6d %6d %6d\n", 
               sensor_data.gyro_x, 
               sensor_data.gyro_y, 
               sensor_data.gyro_z);
        
        HAL_Delay(100);
    }
}
