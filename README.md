# FSAE EV Powertrain Dashboard 

Master Arduino integration for the EV Powertrain. This system reads live CAN-BUS data from the Orion BMS and DTI Inverter, logs 6 physical driver inputs to an SD card at 50Hz, and pushes real-time telemetry to a Nextion touchscreen interface at 10Hz.

## ⚙️ Hardware Architecture
* **Microcontroller:** Arduino Uno R3
* **CAN Interface:** SparkFun CAN-BUS Shield
* **Display:** Nextion Touchscreen (HMI)
* **Storage:** MicroSD Card (FAT32 formatted)

## Wiring & Pinout Guide
If you are wiring this into the chassis, use the following pin mappings. **Note: Unplug the Nextion TX/RX wires before flashing new code to the Arduino!**

**Communication & Shields**
* `Pin 10` - CAN SPI Chip Select (CS)
* `Pin 9`  - SD Card SPI Chip Select (CS)
* `Pin 2`  - CAN Interrupt (INT)
* `Pin 0 (RX)` - Nextion TX (Blue Wire)
* `Pin 1 (TX)` - Nextion RX (Yellow Wire)

**Analog Sensors (5V Max)**
* `A0` - Throttle Potentiometer 1
* `A1` - Throttle Potentiometer 2
* `A2` - Brake Pressure Sensor 1
* `A3` - Brake Pressure Sensor 2
* `A4` - Front Left Suspension Potentiometer
* `A5` - Front Right Suspension Potentiometer

## Network Speeds
* **CAN-BUS:** `500 kbps` (Standard 11-bit IDs)
* **Nextion UART:** `115200 baud`

## SD Card Data Logging
The system features an auto-incrementing file namer. Every time the Arduino is powered on, it creates a brand new log file (`LOG_000.CSV`, `LOG_001.CSV`, etc.) so previous runs are never overwritten. 

**CSV Columns Output:**
`Time_ms, T1_Raw, T2_Raw, B1_Raw, B2_Raw, Susp1_Raw, Susp2_Raw, Bat_SOC_%, Bat_Max_C, Bat_Avg_C, Bat_Amps, Bat_Volts, Motor_RPM, Motor_C, Inv_C, Bat_Imbal`
*(Note: Analog inputs are logged as raw 10-bit ADC values 0-1023. Conversion to engineering units must be done in post-processing).*
