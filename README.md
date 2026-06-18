# FSAE EV Powertrain Dashboard 

Master Arduino integration for the EV Powertrain. This system reads live CAN-BUS data from the Orion BMS and DTI Inverter, logs 6 physical driver inputs to an SD card at 50Hz, and pushes real-time telemetry to a Nextion touchscreen interface at 10Hz.

## Hardware Architecture
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

## Network Speeds
* **CAN-BUS:** `500 kbps` (Standard 11-bit IDs)
* **Nextion UART:** `115200 baud`

## SD Card Data Logging
The system features an auto-incrementing file namer. Every time the Arduino is powered on, it creates a brand new log file (`LOG_000.CSV`, `LOG_001.CSV`, etc.) so previous runs are never overwritten. 

**CSV Columns Output:**

*(Note: Analog inputs are logged as raw 10-bit ADC values 0-1023. Conversion to engineering units must be done in post-processing).*
* Time_ms
* Inverter_Enabled
* Bat_SOC_%
* Bat_Max_C
* Bat_Avg_C
* Bat_Min_C
* Bat_Amps
* Bat_Volts
* LowCell_V
* HighCell_V
* X_Accel
* Y_Accel
* Z_Accel
* Roll_Gyro
* Pitch_Gyro
* Yaw_Gyro
* Roll_Angle
* Pitch_Angle
* Yaw_Angle
* DTI_ControlMode
* DTI_TargetIq_Apk
* DTI_MotorPos_deg
* DTI_IsMotorStill
* DTI_ERPM
* DTI_MotorRPM
* DTI_Duty_pct
* DTI_InputVoltage_V
* DTI_ACCurrent_Apk
* DTI_DCCurrent_Adc
* DTI_CtrlTemp_C
* DTI_MotorTemp_C
* DTI_FaultCode
* DTI_Id_Apk
* DTI_Iq_Apk
* DTI_Throttle_pct
* DTI_Brake_pct
* DTI_DI1
* DTI_DI2
* DTI_DI3
* DTI_DI4
* DTI_DO1
* DTI_DO2
* DTI_DO3
* DTI_DO4
* DTI_DriveEnable
* DTI_CapTempLim
* DTI_DCCurrLim
* DTI_DriveEnLim
* DTI_IGBTAccLim
* DTI_IGBTTempLim
* DTI_VinLim
* DTI_MotAccLim
* DTI_MotTempLim
* DTI_RPMMinLim
* DTI_RPMMaxLim
* DTI_PowerLim
* DTI_CANMapVer
* DTI_MaxAC_Conf
* DTI_AvailMaxAC
* DTI_MinAC_Conf
* DTI_AvailMinAC
* DTI_MaxDC_Conf
* DTI_AvailMaxDC
* DTI_MinDC_Conf
* DTI_AvailMinDC
