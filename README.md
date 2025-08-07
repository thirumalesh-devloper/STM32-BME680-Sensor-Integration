# STM32-BME680-Sensor-Integration
 This project demonstrates interfacing the Bosch BME680 environmental sensor with the STM32L4R9AI Discovery board using I2C3 for sensor communication and USART2 for serial data output.
# BME680 Environmental Sensor Driver for STM32L4R9AI Discovery Board

A complete BME680 4-in-1 environmental sensor driver implementation for the STM32L4R9AI Discovery board using STM32CubeIDE and HAL library with Bosch BME680 official API.

## Features

- ✅ Read temperature data (°C)
- ✅ Read atmospheric pressure (hPa)  
- ✅ Read relative humidity (%)
- ✅ Read gas resistance for air quality (kΩ)
- ✅ I2C3 communication interface
- ✅ UART2 serial output for data monitoring
- ✅ Automatic I2C address detection (0x76/0x77)
- ✅ I2C bus scanning functionality
- ✅ Configurable sensor settings and filters
- ✅ 1-second sampling rate
- ✅ Real-time environmental data display

## Hardware Requirements

### Components
- STM32L4R9AI Discovery Board
- BME680 environmental sensor module (Bosch)
- Jumper wires for connections

### Pin Connections

| BME680 Pin | STM32L4R9AI Pin | Arduino Pin | Function |
|------------|-----------------|-------------|----------|
| VCC        | 3.3V           | -           | Power Supply |
| GND        | GND            | -           | Ground |
| SDA        | PG8            | ARD14       | I2C Data |
| SCL        | PG7            | ARD15       | I2C Clock |
| SDO        | GND or 3.3V    | -           | Address Select (0x76/0x77) |

**Note:** SDO pin determines I2C address:
- SDO to GND = 0x76 (Primary address)
- SDO to 3.3V = 0x77 (Secondary address)

## Software Configuration

### STM32CubeMX Settings

1. **I2C3 Configuration:**
   - Mode: I2C
   - I2C Speed: Standard Mode (100 KHz)
   - SDA Pin: PG8 (ARD14)
   - SCL Pin: PG7 (ARD15)

2. **UART2 Configuration:**
   - Mode: Asynchronous
   - Baud Rate: 115200
   - Word Length: 8 bits
   - Stop Bits: 1
   - Parity: None

3. **GPIO Configuration:**
   - Enable GPIO ports for I2C3 and UART2
   - Configure pins as alternate function

### Required Libraries
- Bosch BME680 Official API (included in project)
- STM32 HAL Library

## Installation & Usage

### 1. Clone Repository
```bash
git clone https://github.com/thirumalesh-devloper/STM32-BME680-Sensor-Integration.git
cd STM32-BME680-Sensor-Integration
```

### 2. STM32CubeIDE Setup
1. Open STM32CubeIDE
2. Import the project: `File > Import > Existing Projects into Workspace`
3. Select the cloned repository folder
4. Ensure BME680 API files are included in the project
5. Build the project (`Ctrl + B`)

### 3. Hardware Setup
1. Connect BME680 to STM32L4R9AI according to the pin table above
2. Ensure proper power supply (3.3V) to BME680
3. Connect SDO pin to set desired I2C address
4. Connect ST-Link debugger/programmer

### 4. Programming & Monitoring
1. Flash the program to STM32L4R9AI board
2. Open serial terminal (PuTTY, Tera Term, or Arduino Serial Monitor)
3. Configure: 115200 baud, 8N1
4. Connect to the appropriate COM port
5. Reset the board to start data transmission

## Sample Output

```
Scanning I2C bus:
0x77 

Initializing BME680...
Init attempt 1: result -2
Init attempt 2: result -2
Init attempt 3: result -2
Init attempt 4: result -2
Init attempt 5: result -2
Secondary Init attempt 1: result 0
BME680 initialized successfully!

Temperature: 28.0 °C, Pressure: 906.9 hPa, Humidity: 59.4 %, Gas: 66.7 kOhms
Temperature: 28.0 °C, Pressure: 906.9 hPa, Humidity: 59.3 %, Gas: 23.1 kOhms
Temperature: 28.1 °C, Pressure: 906.9 hPa, Humidity: 59.2 %, Gas: 25.3 kOhms
Temperature: 28.1 °C, Pressure: 906.9 hPa, Humidity: 59.1 %, Gas: 27.6 kOhms
Temperature: 28.2 °C, Pressure: 906.9 hPa, Humidity: 58.9 %, Gas: 29.8 kOhms
Temperature: 28.2 °C, Pressure: 906.9 hPa, Humidity: 58.8 %, Gas: 31.9 kOhms
Temperature: 28.2 °C, Pressure: 906.9 hPa, Humidity: 58.7 %, Gas: 34.1 kOhms
Temperature: 28.3 °C, Pressure: 906.9 hPa, Humidity: 58.6 %, Gas: 36.0 kOhms
```

## Code Structure

```
├── Core/
│   ├── Inc/
│   │   ├── main.h                    # Main header file
│   │   └── stm32l4xx_hal_conf.h
│   └── Src/
│       ├── main.c                    # Main application with BME680 driver
│       ├── stm32l4xx_hal_msp.c
│       └── stm32l4xx_it.c
├── BME680_API/                       # Bosch BME680 official API
│   ├── bme680.h
│   ├── bme680.c
│   └── bme680_defs.h
├── Drivers/                          # STM32 HAL drivers
└── README.md
```

## API Functions

### I2C Interface Functions
```c
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
void user_delay_ms(uint32_t period);
```

### Utility Functions
```c
void I2C_Scan(void);                  // Scan I2C bus for devices
```

### BME680 Sensor Structure
```c
struct bme680_field_data {
    uint8_t status;                   // Status of measurement
    uint8_t gas_valid;               // Gas measurement valid flag
    uint8_t heat_stab;               // Heater stability flag
    int16_t temperature;             // Temperature in °C * 100
    uint32_t pressure;               // Pressure in Pa
    uint32_t humidity;               // Humidity in % * 1000
    uint32_t gas_resistance;         // Gas resistance in Ohm
};
```

## Sensor Configuration

### Current Settings
```c
// Temperature, Pressure, Humidity oversampling
gas_sensor.tph_sett.os_temp = BME680_OS_8X;    // 8x oversampling
gas_sensor.tph_sett.os_pres = BME680_OS_4X;    // 4x oversampling  
gas_sensor.tph_sett.os_hum = BME680_OS_2X;     // 2x oversampling

// IIR Filter
gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;

// Gas sensor settings
gas_sensor.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
gas_sensor.gas_sett.heatr_temp = 320;          // 320°C heater temperature
gas_sensor.gas_sett.heatr_dur = 150;           // 150ms heating duration

// Power mode
gas_sensor.power_mode = BME680_FORCED_MODE;    // Forced mode for single measurements
```

### Available Oversampling Options
| Setting | Options |
|---------|---------|
| Temperature | BME680_OS_NONE, BME680_OS_1X, BME680_OS_2X, BME680_OS_4X, BME680_OS_8X, BME680_OS_16X |
| Pressure | BME680_OS_NONE, BME680_OS_1X, BME680_OS_2X, BME680_OS_4X, BME680_OS_8X, BME680_OS_16X |
| Humidity | BME680_OS_NONE, BME680_OS_1X, BME680_OS_2X, BME680_OS_4X, BME680_OS_8X, BME680_OS_16X |

### Available Filter Options
| Filter Size | Coefficient |
|-------------|-------------|
| BME680_FILTER_SIZE_0 | No filtering |
| BME680_FILTER_SIZE_1 | 2 |
| BME680_FILTER_SIZE_3 | 4 |
| BME680_FILTER_SIZE_7 | 8 |
| BME680_FILTER_SIZE_15 | 16 |
| BME680_FILTER_SIZE_31 | 32 |
| BME680_FILTER_SIZE_63 | 64 |
| BME680_FILTER_SIZE_127 | 128 |

## Data Interpretation

### Temperature
- Range: -40°C to +85°C
- Accuracy: ±1.0°C (typical)
- Resolution: 0.01°C

### Pressure  
- Range: 300 to 1100 hPa
- Accuracy: ±1.0 hPa (typical)
- Resolution: 0.18 Pa

### Humidity
- Range: 0 to 100% RH
- Accuracy: ±3% RH (typical)
- Resolution: 0.008% RH

### Gas Resistance
- Range: 1 to 500 kΩ
- Used for air quality indication
- Higher values typically indicate better air quality
- Requires calibration for absolute air quality measurements

## Troubleshooting

### Common Issues

1. **"BME680 initialization failed!" with result -2**
   - Check wiring connections (SDA/SCL)
   - Verify 3.3V power supply
   - Ensure I2C pullup resistors (usually built-in on modules)
   - Try different I2C address by changing SDO connection

2. **I2C Scan shows no devices**
   - Check physical connections
   - Verify I2C3 pins (PG7/PG8) are correctly configured
   - Check if sensor is powered correctly

3. **No Serial Output**
   - Check UART2 connections and configuration
   - Verify baud rate (115200)
   - Ensure correct COM port selection

4. **Unstable Gas Readings**
   - Allow 20-30 minutes for sensor stabilization after power-on
   - Gas sensor heating element needs time to reach steady state
   - Consider environmental factors (airflow, temperature changes)

### Debug Tips
- Use I2C_Scan() function to verify sensor connection
- Check initialization result codes:
  - `0` = Success
  - `-1` = Null pointer error
  - `-2` = Communication error
  - `-3` = Invalid length error
- Monitor heater stability flag in gas measurements
- Use STM32CubeIDE debugger for detailed troubleshooting

## Development Environment

- **IDE:** STM32CubeIDE 1.x
- **Framework:** STM32 HAL Library + Bosch BME680 API
- **Target:** STM32L4R9AIIx microcontroller
- **Board:** STM32L4R9AI Discovery Kit
- **Sensor API:** Bosch Sensortec BME680 Official API

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/improvement`)
3. Commit changes (`git commit -am 'Add new feature'`)
4. Push to branch (`git push origin feature/improvement`)
5. Create Pull Request



## Acknowledgments

- Bosch Sensortec for BME680 sensor and official API
- STMicroelectronics for STM32 HAL library
- STM32 community for reference implementations

## Additional Resources

- [BME680 Datasheet](https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme680/)
- [Bosch BME680 API GitHub](https://github.com/BoschSensortec/BME680_driver)
- [STM32L4R9AI Discovery Board User Manual](https://www.st.com/resource/en/user_manual/dm00368330-discovery-kit-with-stm32l4r9ai-mcu-stmicroelectronics.pdf)

---

**⭐ If this project helped you measure environmental conditions, please star the repository!**