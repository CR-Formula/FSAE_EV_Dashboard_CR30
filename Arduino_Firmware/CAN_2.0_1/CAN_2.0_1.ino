#include <SPI.h>
#include <mcp_can.h>
#include <SD.h>

// Pin Definitions
const int CAN_CS_PIN = 10;
const int SD_CS_PIN = 9;
const int CAN_INT_PIN = 2;

// Analog Sensor Pins
const int THROTTLE_1_PIN = A0;
const int THROTTLE_2_PIN = A1;
const int BRAKE_1_PIN = A2;
const int BRAKE_2_PIN = A3;
const int SUSP_1_PIN = A4;
const int SUSP_2_PIN = A5;

MCP_CAN CAN(CAN_CS_PIN);
bool sdReady = false;

// Global variable to hold our unique filename for this drive
char logFileName[13];

// Structure for Powertrain CAN Data
struct CANSensorData {
  int batSoc = 100;
  int batMaxTemp = 25;
  int batAvgTemp = 22;
  int batCurrent = 15;
  int batImbalance = 5;
  int batVolt = 400;
  int motorRpm = 0;
  int motorTemp = 45;
  int invTemp = 40;
} dashData;

// --- Timing Variables ---
unsigned long lastLogTime = 0;
unsigned long lastDisplayTime = 0;
const int LOG_RATE_MS = 20;       // 50Hz Logging rate for SD card
const int DISPLAY_RATE_MS = 100;  // 10Hz Display refresh rate

void setup() {
  Serial.begin(115200);

  pinMode(CAN_INT_PIN, INPUT_PULLUP);


  // Initialize SD Card
  if (SD.begin(SD_CS_PIN)) {

    // Scan the SD card for the next available filename
    for (int i = 0; i < 1000; i++) {
      sprintf(logFileName, "LOG_%03d.CSV", i);
      if (!SD.exists(logFileName)) {
        break;  // Found an empty slot!
      }
    }

    // Create that specific file and write the master header
    File dataLog = SD.open(logFileName, FILE_WRITE);
    if (dataLog) {
      dataLog.println("Time_ms,T1_Raw,T2_Raw,B1_Raw,B2_Raw,Susp1_Raw,Susp2_Raw,Bat_SOC_%,Bat_Max_C,Bat_Avg_C,Bat_Amps,Bat_Volts,Motor_RPM,Motor_C,Inv_C,Bat_Imbal");
      dataLog.close();
      sdReady = true;
    }
  }


  // Initialize CAN Bus
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {  //Varify this is 500KBPS
    CAN.setMode(MCP_NORMAL);
  }

  delay(1000);  // Let Nextion boot
}

// Helper to push string values to Nextion Text components (.txt)
void sendNextionText(const char* component, String value) {
  Serial.print(component);
  Serial.print(".txt=\"");
  Serial.print(value);
  Serial.print("\"");
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
}

void loop() {
  unsigned long currentTime = millis();

  // Read CAN Bus

  if (!digitalRead(CAN_INT_PIN)) {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];
    CAN.readMsgBuf(&rxId, &len, rxBuf);

    switch (rxId) {  //Default CAN ID's (are they different?)
      // --- ORION BMS ---
      case 0x6B0:
        dashData.batCurrent = ((rxBuf[0] << 8) | rxBuf[1]) / 10;
        dashData.batVolt = ((rxBuf[2] << 8) | rxBuf[3]) / 10;
        dashData.batSoc = rxBuf[4];
        break;

      case 0x6B1:
        dashData.batMaxTemp = rxBuf[4];
        dashData.batAvgTemp = rxBuf[5];
        break;

      // --- DTI INVERTER ---
      case 0x2A4:  // ERPM
        {
          long erpm = ((long)rxBuf[0] << 24) | ((long)rxBuf[1] << 16) | ((long)rxBuf[2] << 8) | rxBuf[3];
          dashData.motorRpm = erpm / 10;
        }
        break;

      case 0x2C4:  // Temps
        {
          dashData.invTemp = ((rxBuf[0] << 8) | rxBuf[1]) / 10;
          dashData.motorTemp = ((rxBuf[2] << 8) | rxBuf[3]) / 10;
        }
        break;
    }
  }


  //Read Analog & Log to SD (50Hz)

  if (currentTime - lastLogTime >= LOG_RATE_MS) {
    lastLogTime = currentTime;

    // Read the 6 local sensors
    int t1 = analogRead(THROTTLE_1_PIN);
    int t2 = analogRead(THROTTLE_2_PIN);
    int b1 = analogRead(BRAKE_1_PIN);
    int b2 = analogRead(BRAKE_2_PIN);
    int susp1 = analogRead(SUSP_1_PIN);
    int susp2 = analogRead(SUSP_2_PIN);

    // Write to the specific logFileName created in setup()
    if (sdReady) {
      File dataLog = SD.open(logFileName, FILE_WRITE); // right now it is printing raw values into the sd card
      if (dataLog) {
        dataLog.print(currentTime);
        dataLog.print(",");
        dataLog.print(t1);
        dataLog.print(",");
        dataLog.print(t2);
        dataLog.print(",");
        dataLog.print(b1);
        dataLog.print(",");
        dataLog.print(b2);
        dataLog.print(",");
        dataLog.print(susp1);
        dataLog.print(",");
        dataLog.print(susp2);
        dataLog.print(",");
        dataLog.print(dashData.batSoc);
        dataLog.print(",");
        dataLog.print(dashData.batMaxTemp);
        dataLog.print(",");
        dataLog.print(dashData.batAvgTemp);
        dataLog.print(",");
        dataLog.print(dashData.batCurrent);
        dataLog.print(",");
        dataLog.print(dashData.batVolt);
        dataLog.print(",");
        dataLog.print(dashData.motorRpm);
        dataLog.print(",");
        dataLog.print(dashData.motorTemp);
        dataLog.print(",");
        dataLog.print(dashData.invTemp);
        dataLog.print(",");
        dataLog.println(dashData.batImbalance);

        dataLog.close();
      }
    }
  }


  // TASK 3: Update Nextion Display (10Hz)

  if (currentTime - lastDisplayTime >= DISPLAY_RATE_MS) {
    lastDisplayTime = currentTime;

    sendNextionText("batPct", String(dashData.batSoc));
    sendNextionText("batMaxTemp", String(dashData.batMaxTemp));
    sendNextionText("batAvgTemp", String(dashData.batAvgTemp));
    sendNextionText("batCurrent", String(dashData.batCurrent));
    sendNextionText("batVolt", String(dashData.batVolt));
    sendNextionText("rpm", String(dashData.motorRpm));
    sendNextionText("motTemp", String(dashData.motorTemp));
    sendNextionText("invTemp", String(dashData.invTemp));
    sendNextionText("batImbalance", String(dashData.batImbalance));
  }
}