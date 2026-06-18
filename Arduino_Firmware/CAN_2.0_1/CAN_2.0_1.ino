#include <SPI.h>
#include <mcp_can.h>
#include <SD.h>

// DTI Inverter config
#define DTI_POLE_PAIRS   (10)
#define DTI_EXTENDED_ID  (true)   // confirmed: Commander uses extended IDs, (packetId<<8)|nodeId
#define DTI_NODE_ID      (0x13)   // confirmed: Node ID = 19 (0x13) from Commander sketch

// Pin Definitions
#define CAN_CS_PIN              (10) // CAN CS pin
#define CAN_INT_PIN             (2)  // Interrupt pin for CAN bus
#define SD_CS_PIN               (9)  // SD card CS pin
#define RTD_PLAUSIBILITY_PIN    (3)  // Input from RTD for APPS/Brake plausibility check
#define PRECHARGE_PRECHARGE_PIN (4)  // Output to precharge board to indicate precharge completion
#define PRECHARGE_RTD_PIN       (5)  // Output to RTD to indicate precharge completion

// Precharge Constants
#define PRECHARGE_THRESHOLD_PRECHARGE (0.92) // Percentage to send signal to precharge board
#define PRECHARGE_THRESHOLD_RTD       (0.97) // Percentage to send signal to RTD

MCP_CAN CAN(CAN_CS_PIN);
bool sdReady = false;

char logFileName[13];

// Sensor Data Structure
struct CANSensorData {

  // --- BMS (Orion) ---
  int batSoc       = 0;
  int batMaxTemp   = 0;
  int batAvgTemp   = 0;
  int batMinTemp   = 0;
  float batCurrent = 0.0;
  float batVolt    = 0.0;
  int lowCellVolt  = 0;
  int highCellVolt = 0;

  // --- IMU ---
  int xAccel = 0, yAccel = 0, zAccel = 0;
  int rollGyro = 0, pitchGyro = 0, yawGyro = 0;
  int rollAngle = 0, pitchAngle = 0, yawAngle = 0;

  // --- DTI 0x1F ---
  uint8_t dtiControlMode   = 0;
  float   dtiTargetIq      = 0.0;
  float   dtiMotorPosition = 0.0;
  uint8_t dtiIsMotorStill  = 0;

  // --- DTI 0x20 ---
  long  dtiErpm         = 0;
  float dtiDuty         = 0.0;
  int   dtiInputVoltage = 0;

  // --- DTI 0x21 ---
  float dtiAcCurrent = 0.0;
  float dtiDcCurrent = 0.0;

  // --- DTI 0x22 ---
  float   dtiCtrlTemp  = 0.0;
  float   dtiMotorTemp = 0.0;
  uint8_t dtiFaultCode = 0;

  // --- DTI 0x23 ---
  float dtiIdActual = 0.0;
  float dtiIqActual = 0.0;

  // --- DTI 0x24 ---
  int8_t  dtiThrottle    = 0;
  int8_t  dtiBrake       = 0;
  uint8_t dtiDI          = 0;
  uint8_t dtiDO          = 0;
  uint8_t dtiDriveEnable = 0;
  uint8_t dtiLimitFlags4 = 0;
  uint8_t dtiLimitFlags5 = 0;
  uint8_t dtiCanMapVer   = 0;

  // --- DTI 0x25 ---
  float dtiMaxAcConf  = 0.0;
  float dtiAvailMaxAc = 0.0;
  float dtiMinAcConf  = 0.0;
  float dtiAvailMinAc = 0.0;

  // --- DTI 0x26 ---
  float dtiMaxDcConf  = 0.0;
  float dtiAvailMaxDc = 0.0;
  float dtiMinDcConf  = 0.0;
  float dtiAvailMinDc = 0.0;

} dashData;


// Timing
unsigned long lastLogTime     = 0;
unsigned long lastDisplayTime = 0;
const int LOG_RATE_MS     = 20;   // 50 Hz
const int DISPLAY_RATE_MS = 100;  // 10 Hz


// ---- Big-endian decode helpers ----

int16_t parseInt16BE(unsigned char* buf, int offset) {
  return (int16_t)(((uint16_t)buf[offset] << 8) | (uint16_t)buf[offset + 1]);
}

uint16_t parseUint16BE(unsigned char* buf, int offset) {
  return (uint16_t)(((uint16_t)buf[offset] << 8) | (uint16_t)buf[offset + 1]);
}

int32_t parseInt32BE(unsigned char* buf, int offset) {
  return (int32_t)(((uint32_t)buf[offset]     << 24) |
                   ((uint32_t)buf[offset + 1] << 16) |
                   ((uint32_t)buf[offset + 2] <<  8) |
                    (uint32_t)buf[offset + 3]);
}


// ---- DTI Drive Enable command ----
// Sends DTI packet 0x0C: byte0 = 1 (enable) or 0 (disable)
void sendDriveEnable(bool enable) {
  unsigned char payload[8] = {
    (uint8_t)(enable ? 1 : 0),
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };

  uint32_t canId;
  if (DTI_EXTENDED_ID) {
    canId = ((uint32_t)0x0C << 8) | DTI_NODE_ID;
    CAN.sendMsgBuf(canId, 1, 8, payload);  // 1 = extended frame
  } else {
    canId = ((uint32_t)0x0C << 5) | DTI_NODE_ID;
    CAN.sendMsgBuf(canId, 0, 8, payload);  // 0 = standard frame
  }
}


// ---- Setup ----

void setup() {
  pinMode(RTD_PLAUSIBILITY_PIN, INPUT);

  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  pinMode(PRECHARGE_PRECHARGE_PIN, OUTPUT);
  digitalWrite(PRECHARGE_PRECHARGE_PIN, LOW);

  pinMode(PRECHARGE_RTD_PIN, OUTPUT);
  digitalWrite(PRECHARGE_RTD_PIN, LOW);

 // Serial.begin(115200);

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK) {
  //  Serial.println("CAN Init OK");
    CAN.setMode(MCP_NORMAL);
  } else {
//    Serial.println("CAN Init FAILED - Check CS Pin and wiring");
    while (1);
  }

  pinMode(CAN_INT_PIN, INPUT_PULLUP);

  if (SD.begin(SD_CS_PIN)) {
    bool foundSlot = false;

    for (int i = 0; i < 999; i++) {  // stop at 998 so LOG_999 is never silently overwritten
      sprintf(logFileName, "LOG_%03d.CSV", i);
      if (!SD.exists(logFileName)) {
        foundSlot = true;
        break;
      }
    }
    if (!foundSlot) {
     // Serial.println("SD: No free log slots!");
    } else {
      File dataLog = SD.open(logFileName, FILE_WRITE);
      if (dataLog) {
        dataLog.println(F(
          "Time_ms,"
          "Inverter_Enabled,"
          "Bat_SOC_%,Bat_Max_C,Bat_Avg_C,Bat_Min_C,Bat_Amps,Bat_Volts,LowCell_V,HighCell_V,"
          "X_Accel,Y_Accel,Z_Accel,Roll_Gyro,Pitch_Gyro,Yaw_Gyro,Roll_Angle,Pitch_Angle,Yaw_Angle,"
          "DTI_ControlMode,DTI_TargetIq_Apk,DTI_MotorPos_deg,DTI_IsMotorStill,"
          "DTI_ERPM,DTI_MotorRPM,DTI_Duty_pct,DTI_InputVoltage_V,"
          "DTI_ACCurrent_Apk,DTI_DCCurrent_Adc,"
          "DTI_CtrlTemp_C,DTI_MotorTemp_C,DTI_FaultCode,"
          "DTI_Id_Apk,DTI_Iq_Apk,"
          "DTI_Throttle_pct,DTI_Brake_pct,"
          "DTI_DI1,DTI_DI2,DTI_DI3,DTI_DI4,"
          "DTI_DO1,DTI_DO2,DTI_DO3,DTI_DO4,"
          "DTI_DriveEnable,"
          "DTI_CapTempLim,DTI_DCCurrLim,DTI_DriveEnLim,DTI_IGBTAccLim,DTI_IGBTTempLim,DTI_VinLim,DTI_MotAccLim,DTI_MotTempLim,"
          "DTI_RPMMinLim,DTI_RPMMaxLim,DTI_PowerLim,"
          "DTI_CANMapVer,"
          "DTI_MaxAC_Conf,DTI_AvailMaxAC,DTI_MinAC_Conf,DTI_AvailMinAC,"
          "DTI_MaxDC_Conf,DTI_AvailMaxDC,DTI_MinDC_Conf,DTI_AvailMinDC"
        ));
        dataLog.close();
        sdReady = true;
     //   Serial.print("Logging to: ");
     //   Serial.println(logFileName);
      }
    }
  } else {
   // Serial.println("SD Init FAILED");
  }

  delay(1000);  // Let Nextion boot
}


// ---- Nextion helper ----
void sendNextionText(const char* component, String value) {
  Serial.print(component);
  Serial.print(".txt=\"");
  Serial.print(value);
  Serial.print("\"");
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
}


// ---- Main Loop ----

void loop() {
  unsigned long currentTime = millis();

  // ---- CAN Receive ----
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];
    CAN.readMsgBuf(&rxId, &len, rxBuf);

    uint8_t packetId;
    if (DTI_EXTENDED_ID) {
      packetId = (uint8_t)(rxId >> 8);
    } else {
      packetId = (uint8_t)(rxId >> 5);
    }

    // ---- BMS (Orion) ----
    switch (rxId) {
      case 0x6B0:
        dashData.batCurrent  = parseInt16BE(rxBuf, 0) / 10.0;
        dashData.batVolt     = parseUint16BE(rxBuf, 2) / 10.0;
        dashData.batSoc      = rxBuf[4];
        break;

      case 0x6B1:
        dashData.batAvgTemp   = rxBuf[0];
        dashData.lowCellVolt  = rxBuf[2];
        dashData.highCellVolt = rxBuf[3];
        dashData.batMaxTemp   = rxBuf[4];
        dashData.batMinTemp   = rxBuf[5];
        break;

      // ---- IMU ----
      case 0x100:
        dashData.xAccel = parseInt16BE(rxBuf, 0);
        dashData.yAccel = parseInt16BE(rxBuf, 2);
        dashData.zAccel = parseInt16BE(rxBuf, 4);
        break;

      case 0x101:
        dashData.rollGyro  = parseInt16BE(rxBuf, 0);
        dashData.pitchGyro = parseInt16BE(rxBuf, 2);
        dashData.yawGyro   = parseInt16BE(rxBuf, 4);
        break;

      case 0x102:
        dashData.rollAngle  = parseInt16BE(rxBuf, 0);
        dashData.pitchAngle = parseInt16BE(rxBuf, 2);
        dashData.yawAngle   = parseInt16BE(rxBuf, 4);
        break;
    }

    // ---- DTI Inverter (no node filter) ----
    switch (packetId) {

      case 0x1F:
        dashData.dtiControlMode   = rxBuf[0];
        dashData.dtiTargetIq      = parseInt16BE(rxBuf, 1) / 10.0;
        dashData.dtiMotorPosition = parseUint16BE(rxBuf, 3) / 10.0;
        dashData.dtiIsMotorStill  = rxBuf[5];
        break;

      case 0x20:
        dashData.dtiErpm         = parseInt32BE(rxBuf, 0);
        dashData.dtiDuty         = parseInt16BE(rxBuf, 4) / 10.0;
        dashData.dtiInputVoltage = (int)parseUint16BE(rxBuf, 6);
        break;

      case 0x21:
        dashData.dtiAcCurrent = parseInt16BE(rxBuf, 0) / 10.0;
        dashData.dtiDcCurrent = parseInt16BE(rxBuf, 2) / 10.0;
        break;

      case 0x22:
        dashData.dtiCtrlTemp  = parseInt16BE(rxBuf, 0) / 10.0;
        dashData.dtiMotorTemp = parseInt16BE(rxBuf, 2) / 10.0;
        dashData.dtiFaultCode = rxBuf[4];
        break;

      case 0x23:
        dashData.dtiIdActual = parseInt32BE(rxBuf, 0) / 100.0;
        dashData.dtiIqActual = parseInt32BE(rxBuf, 4) / 100.0;
        break;

      case 0x24:
        dashData.dtiThrottle    = (int8_t)rxBuf[0];
        dashData.dtiBrake       = (int8_t)rxBuf[1];
        dashData.dtiDI          = rxBuf[2] & 0x0F;
        dashData.dtiDO          = (rxBuf[2] >> 4) & 0x0F;
        dashData.dtiDriveEnable = rxBuf[3] & 0x01;
        dashData.dtiLimitFlags4 = rxBuf[4];
        dashData.dtiLimitFlags5 = rxBuf[5] & 0x07;
        dashData.dtiCanMapVer   = rxBuf[7];
        break;

      case 0x25:
        dashData.dtiMaxAcConf  = parseInt16BE(rxBuf, 0) / 10.0;
        dashData.dtiAvailMaxAc = parseInt16BE(rxBuf, 2) / 10.0;
        dashData.dtiMinAcConf  = parseInt16BE(rxBuf, 4) / 10.0;
        dashData.dtiAvailMinAc = parseInt16BE(rxBuf, 6) / 10.0;
        break;

      case 0x26:
        dashData.dtiMaxDcConf  = parseInt16BE(rxBuf, 0) / 10.0;
        dashData.dtiAvailMaxDc = parseInt16BE(rxBuf, 2) / 10.0;
        dashData.dtiMinDcConf  = parseInt16BE(rxBuf, 4) / 10.0;
        dashData.dtiAvailMinDc = parseInt16BE(rxBuf, 6) / 10.0;
        break;
    }
  }
  
  // APPS to brake plausibility check input from RTD
  bool enableInverter = digitalRead(RTD_PLAUSIBILITY_PIN) == HIGH;
  sendDriveEnable(enableInverter);

  // Precharge completion calculations
  double prechargePercentage = dashData.dtiInputVoltage / dashData.batVolt;
  digitalWrite(PRECHARGE_PRECHARGE_PIN, (prechargePercentage > PRECHARGE_THRESHOLD_PRECHARGE) ? HIGH : LOW);
  digitalWrite(PRECHARGE_RTD_PIN, (prechargePercentage > PRECHARGE_THRESHOLD_RTD) ? HIGH : LOW);

  // ---- SD Logging at 50 Hz ----
  if (currentTime - lastLogTime >= LOG_RATE_MS) {
    lastLogTime += LOG_RATE_MS;

    if (sdReady) {
      File dataLog = SD.open(logFileName, FILE_WRITE);
      if (dataLog) {

        dataLog.print(currentTime);           dataLog.print(",");

        // APPS to brake plausibility
        dataLog.print(enableInverter ? 1 : 0); dataLog.print(",");

        // BMS
        dataLog.print(dashData.batSoc);        dataLog.print(",");
        dataLog.print(dashData.batMaxTemp);    dataLog.print(",");
        dataLog.print(dashData.batAvgTemp);    dataLog.print(",");
        dataLog.print(dashData.batMinTemp);    dataLog.print(",");
        dataLog.print(dashData.batCurrent, 2); dataLog.print(",");
        dataLog.print(dashData.batVolt, 2);    dataLog.print(",");
        dataLog.print(dashData.lowCellVolt);   dataLog.print(",");
        dataLog.print(dashData.highCellVolt);  dataLog.print(",");

        // IMU
        dataLog.print(dashData.xAccel);       dataLog.print(",");
        dataLog.print(dashData.yAccel);       dataLog.print(",");
        dataLog.print(dashData.zAccel);       dataLog.print(",");
        dataLog.print(dashData.rollGyro);     dataLog.print(",");
        dataLog.print(dashData.pitchGyro);    dataLog.print(",");
        dataLog.print(dashData.yawGyro);      dataLog.print(",");
        dataLog.print(dashData.rollAngle);    dataLog.print(",");
        dataLog.print(dashData.pitchAngle);   dataLog.print(",");
        dataLog.print(dashData.yawAngle);     dataLog.print(",");

        // DTI 0x1F
        dataLog.print(dashData.dtiControlMode);      dataLog.print(",");
        dataLog.print(dashData.dtiTargetIq, 1);      dataLog.print(",");
        dataLog.print(dashData.dtiMotorPosition, 1); dataLog.print(",");
        dataLog.print(dashData.dtiIsMotorStill);     dataLog.print(",");

        // DTI 0x20
        dataLog.print(dashData.dtiErpm);                   dataLog.print(",");
        dataLog.print(dashData.dtiErpm / DTI_POLE_PAIRS);  dataLog.print(",");
        dataLog.print(dashData.dtiDuty, 1);                dataLog.print(",");
        dataLog.print(dashData.dtiInputVoltage);           dataLog.print(",");

        // DTI 0x21
        dataLog.print(dashData.dtiAcCurrent, 1); dataLog.print(",");
        dataLog.print(dashData.dtiDcCurrent, 1); dataLog.print(",");

        // DTI 0x22
        dataLog.print(dashData.dtiCtrlTemp, 1);  dataLog.print(",");
        dataLog.print(dashData.dtiMotorTemp, 1); dataLog.print(",");
        dataLog.print(dashData.dtiFaultCode);    dataLog.print(",");

        // DTI 0x23
        dataLog.print(dashData.dtiIdActual, 2); dataLog.print(",");
        dataLog.print(dashData.dtiIqActual, 2); dataLog.print(",");

        // DTI 0x24
        dataLog.print(dashData.dtiThrottle); dataLog.print(",");
        dataLog.print(dashData.dtiBrake);    dataLog.print(",");

        dataLog.print((dashData.dtiDI >> 0) & 1); dataLog.print(",");
        dataLog.print((dashData.dtiDI >> 1) & 1); dataLog.print(",");
        dataLog.print((dashData.dtiDI >> 2) & 1); dataLog.print(",");
        dataLog.print((dashData.dtiDI >> 3) & 1); dataLog.print(",");

        dataLog.print((dashData.dtiDO >> 0) & 1); dataLog.print(",");
        dataLog.print((dashData.dtiDO >> 1) & 1); dataLog.print(",");
        dataLog.print((dashData.dtiDO >> 2) & 1); dataLog.print(",");
        dataLog.print((dashData.dtiDO >> 3) & 1); dataLog.print(",");

        dataLog.print(dashData.dtiDriveEnable); dataLog.print(",");

        dataLog.print((dashData.dtiLimitFlags4 >> 0) & 1); dataLog.print(",");
        dataLog.print((dashData.dtiLimitFlags4 >> 1) & 1); dataLog.print(",");
        dataLog.print((dashData.dtiLimitFlags4 >> 2) & 1); dataLog.print(",");
        dataLog.print((dashData.dtiLimitFlags4 >> 3) & 1); dataLog.print(",");
        dataLog.print((dashData.dtiLimitFlags4 >> 4) & 1); dataLog.print(",");
        dataLog.print((dashData.dtiLimitFlags4 >> 5) & 1); dataLog.print(",");
        dataLog.print((dashData.dtiLimitFlags4 >> 6) & 1); dataLog.print(",");
        dataLog.print((dashData.dtiLimitFlags4 >> 7) & 1); dataLog.print(",");

        dataLog.print((dashData.dtiLimitFlags5 >> 0) & 1); dataLog.print(",");
        dataLog.print((dashData.dtiLimitFlags5 >> 1) & 1); dataLog.print(",");
        dataLog.print((dashData.dtiLimitFlags5 >> 2) & 1); dataLog.print(",");

        dataLog.print(dashData.dtiCanMapVer); dataLog.print(",");

        dataLog.print(dashData.dtiMaxAcConf, 1);  dataLog.print(",");
        dataLog.print(dashData.dtiAvailMaxAc, 1); dataLog.print(",");
        dataLog.print(dashData.dtiMinAcConf, 1);  dataLog.print(",");
        dataLog.print(dashData.dtiAvailMinAc, 1); dataLog.print(",");

        dataLog.print(dashData.dtiMaxDcConf, 1);  dataLog.print(",");
        dataLog.print(dashData.dtiAvailMaxDc, 1); dataLog.print(",");
        dataLog.print(dashData.dtiMinDcConf, 1);  dataLog.print(",");
        dataLog.println(dashData.dtiAvailMinDc, 1);

        dataLog.close();
      }
    }
  }

  // ---- Nextion Display at 10 Hz ----
  if (currentTime - lastDisplayTime >= DISPLAY_RATE_MS) {
    lastDisplayTime += DISPLAY_RATE_MS;

    sendNextionText("batPct",      String(dashData.batSoc));
    sendNextionText("batMaxTemp",  String(dashData.batMaxTemp));
    sendNextionText("batMinTemp",  String(dashData.batMinTemp));
    sendNextionText("avgCellTemp", String(dashData.batAvgTemp));
    sendNextionText("batCurrent",  String(dashData.batCurrent));
    sendNextionText("batVolt",     String(dashData.batVolt));
    sendNextionText("rpm",         String(dashData.dtiErpm / DTI_POLE_PAIRS));
    sendNextionText("motTemp",     String(dashData.dtiMotorTemp));
    sendNextionText("invTemp",     String(dashData.dtiCtrlTemp));
    sendNextionText("batImbalance",String(dashData.batMaxTemp - dashData.batMinTemp));
  }
}
