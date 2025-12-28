# MPU6500/MPU6050 I2C Driver for STM32

A lightweight, easy-to-use I2C driver for MPU6500 and MPU6050 IMU sensors on STM32 microcontrollers.

## 📋 Features

- ✅ Compatible with both **MPU6500** and **MPU6050**
- ✅ I2C communication using STM32 HAL
- ✅ Automatic sensor detection
- ✅ Built-in calibration routine (1000 samples)
- ✅ Read accelerometer and gyroscope in single transaction
- ✅ Offset compensation for accurate measurements
- ✅ Clean, documented code
- ✅ Example usage included

## 🔧 Hardware Requirements

- **STM32 microcontroller** (tested on STM32F401RE)
- **MPU6500** or **MPU6050** sensor module
- I2C connection (SDA, SCL)
- UART (for printf debug output)

## 📐 Wiring

```
STM32           MPU6500/MPU6050
─────           ───────────────
PB7 (SDA)   →   SDA
PB6 (SCL)   →   SCL
3.3V        →   VCC
GND         →   GND
```

**Note:** Add 4.7kΩ pull-up resistors on SDA and SCL lines if not already present on your module.

## 🚀 Quick Start

### 1. Add Files to Your Project

Copy these files to your STM32 project:
- `mpu6500.h` - Header file
- `mpu6500.c` - Implementation

### 2. Configure STM32CubeMX

Enable the following peripherals:
- **I2C1**: Standard mode (100 kHz)
- **USART2**: 115200 baud (for printf)

### 3. Include Header

```c
#include "mpu6500.h"
```

### 4. Basic Usage

```c
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    
    // Initialize sensor
    MPU6500_Init();
    
    // Verify device
    uint8_t id = MPU6500_WhoAmI();
    
    // Calibrate (keep sensor still!)
    MPU6500_Calibrate();
    
    // Read data
    MPU6500_Data sensor_data;
    int16_t acc_x, acc_y, acc_z;
    int16_t gyr_x, gyr_y, gyr_z;
    
    while (1) {
        MPU6500_ReadAll(&sensor_data);
        MPU6500_GetCalibratedAccel(&sensor_data, &acc_x, &acc_y, &acc_z);
        MPU6500_GetCalibratedGyro(&sensor_data, &gyr_x, &gyr_y, &gyr_z);
        
        printf("ACC: %d %d %d  GYR: %d %d %d\n", 
               acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z);
        
        HAL_Delay(100);
    }
}
```

## 📚 API Reference

### Initialization Functions

#### `MPU6500_Scan()`
Scans I2C bus for MPU6500/MPU6050.
```c
uint8_t address = MPU6500_Scan();
```
**Returns:** I2C address (0x68) or 0 if not found

#### `MPU6500_Init()`
Wakes up sensor from sleep mode.
```c
uint8_t status = MPU6500_Init();
```
**Returns:** 0 on success, 1 on failure

#### `MPU6500_WhoAmI()`
Reads device ID to verify sensor type.
```c
uint8_t device_id = MPU6500_WhoAmI();
```
**Returns:** 
- `0x68` - MPU6050
- `0x70` - MPU6500

### Data Reading Functions

#### `MPU6500_ReadAll()`
Reads all sensor data (accel + gyro) in one I2C transaction.
```c
MPU6500_Data data;
MPU6500_ReadAll(&data);
```

#### `MPU6500_GetCalibratedAccel()`
Gets calibrated accelerometer values.
```c
int16_t x, y, z;
MPU6500_GetCalibratedAccel(&data, &x, &y, &z);
```

#### `MPU6500_GetCalibratedGyro()`
Gets calibrated gyroscope values.
```c
int16_t x, y, z;
MPU6500_GetCalibratedGyro(&data, &x, &y, &z);
```

### Calibration Function

#### `MPU6500_Calibrate()`
Calibrates sensor offsets (takes ~20 seconds).
```c
MPU6500_Calibrate();
```
**⚠️ Important:** Sensor must be completely still during calibration!

## 📊 Data Structure

```c
typedef struct {
    int16_t accel_x;    // Raw accelerometer X
    int16_t accel_y;    // Raw accelerometer Y
    int16_t accel_z;    // Raw accelerometer Z (16384 = 1g)
    int16_t gyro_x;     // Raw gyroscope X
    int16_t gyro_y;     // Raw gyroscope Y
    int16_t gyro_z;     // Raw gyroscope Z
} MPU6500_Data;
```

## 🔢 Understanding the Values

### Accelerometer (±2g range)
- **Range:** ±32768
- **Sensitivity:** 16384 LSB/g
- **Example:** 
  - `16384` = 1g (sensor flat, Z-axis)
  - `0` = 0g (free fall or sensor tilted)
  - `-16384` = -1g (sensor upside down)

### Gyroscope (±250°/s range)
- **Range:** ±32768
- **Sensitivity:** 131 LSB/(°/s)
- **Example:**
  - `0` = Not rotating
  - `131` = Rotating at 1°/s
  - `13100` = Rotating at 100°/s

## 🎯 Example Output

### After Calibration
```
=== CALIBRATION ===
Keep sensor COMPLETELY STILL!
Collecting 1000 samples...
Progress: 100/1000
Progress: 200/1000
...
Progress: 1000/1000

Calibration complete!
Offsets:
  Accel: X=-350, Y=-150, Z=1066
  Gyro:  X=320, Y=600, Z=45
```

### Normal Operation
```
ACC:      5     -3  16380  GYR:      2      1     -4
ACC:     -8     10  16390  GYR:     -1      3      2
ACC:     12     -5  16375  GYR:      4     -2      0
```

Values near 0 indicate sensor is calibrated and still!

## 🐛 Troubleshooting

### "No I2C device found"
- Check wiring (SDA, SCL, VCC, GND)
- Verify pull-up resistors (4.7kΩ) on SDA and SCL
- Check I2C speed (100 kHz recommended)
- Try scanning with different address: `0x69` (if AD0 pin is HIGH)

### "Wrong device ID"
- MPU6050 returns `0x68`
- MPU6500 returns `0x70`
- Both are supported by this library

### Large offset values after calibration
- Sensor was moving during calibration
- Recalibrate with sensor completely still
- Place sensor on stable, flat surface

### Noisy readings
- Normal for cheap IMU sensors
- Values may fluctuate ±50 even when still
- Use filtering or averaging for smoother data

## 📖 Additional Resources

- [MPU6050 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
- [MPU6500 Datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6500-Datasheet2.pdf)
- [STM32 HAL I2C Documentation](https://www.st.com/resource/en/user_manual/um1725-description-of-stm32f4-hal-and-lowlayer-drivers-stmicroelectronics.pdf)

## 🤝 Contributing

Contributions are welcome! Feel free to:
- Report bugs
- Suggest features
- Submit pull requests

## 👨‍💻 Author

**Wassim Kessaissia**
- GitHub: [@wassimkessaissia](https://github.com/wassimkessaissia)
- Project Link: [MPU6500-STM32-Driver](https://github.com/wassimkessaissia/MPU6500-STM32-Driver)

## 📝 License

This project is open source and available under the MIT License.

## ⭐ Acknowledgments

- Built learning embedded systems from scratch
- Developed using STM32 HAL library
- Tested on STM32F401RE Nucleo board with MPU6500

---

**If you found this useful, please give it a ⭐!**
