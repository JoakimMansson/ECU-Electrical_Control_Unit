#include <SPI.h>
#include <Wire.h>

#define CAN_2515
// #define CAN_2518FD

// Set SPI CS Pin according to your hardware

#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)
// For Wio Terminal w/ MCP2518FD RPi Hat：
// Channel 0 SPI_CS Pin: BCM 8
// Channel 1 SPI_CS Pin: BCM 7
// Interupt Pin: BCM25
const int SPI_CS_PIN = BCM8;
const int CAN_INT_PIN = BCM25;
#else

// For Arduino MCP2515 Hat:
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;
#endif


#ifdef CAN_2518FD
#include "mcp2518fd_can.h"
mcp2518fd CAN(SPI_CS_PIN);  // Set CS pin
#endif

#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN);  // Set CS pin
#endif

// 1 for DEBUGGING
#define DEBUG 1

#if DEBUG == 1
#define debug(x) Serial.print(x);
#define debugln(x) Serial.println(x);
#else
#define debug(x)
#define debugln(x)
#endif


unsigned char DRIVE_ARR[8] = { 0, 0, 250, 68, 0, 0, 0, 0 }; // 2000 RPM
unsigned char REVERSE_ARR[8] = { 0, 0, 250, 196, 0, 0, 0, 0 }; // -2000 RPM
unsigned char NEUTRAL_ARR[8] = { 0, 0, 250, 68, 0, 0, 0, 0 }; // 2000 RPM
unsigned char BRAKE_ARR[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned char BUS_VOLTAGE[8] = { 0, 0, 0, 0, 0, 0, 128, 63};

bool isNeutral = true;
bool isDriving = false;
bool isReversing = false;
bool isBraking = false;

int DRIVE_PIN = 7;
int REVERSE_PIN = 8;
int NEUTRAL_PIN = 6;
int BRAKE_PIN = 5;

int INPUT_BRAKE_PIN = A0;
int INPUT_GAS_PIN = A1;


unsigned char CAN_buf[8]; // Storing of incoming CAN data

// Initialization of CAN
void init_CAN() {
  while (CAN_OK != CAN.begin(CAN_500KBPS)) {  // init can bus : baudrate = 500k
    Serial.println("CAN init fail, retry...");
    delay(100);
  }
  Serial.println("CAN init ok!");
}

// ---------------- DRIVING CAR ----------------------------

void sendCAN(int channel, unsigned char msg[8]) 
{
  CAN.sendMsgBuf(channel, 0, 8, msg);
}


void readPanel() {
  if (digitalRead(DRIVE_PIN) == LOW) {
    isDriving = true;

    isBraking = false;
    isReversing = false;
    isNeutral = false;
    return;
  } else if (digitalRead(REVERSE_PIN) == LOW) {
    isReversing = true;

    isDriving = false;
    isBraking = false;
    isNeutral = false;
    return;
  } else if (digitalRead(NEUTRAL_PIN) == LOW) {
    isNeutral = true;

    isBraking = false;
    isDriving = false;
    isReversing = false;
    return;
  } else if (digitalRead(BRAKE_PIN) == LOW) {
    isBraking = true;

    isNeutral = false;
    isDriving = false;
    isReversing = false;
    return;
  }
}

void resetArrays()
{
  for(int i = 4; i < 8; i++)
  {
    BRAKE_ARR[i] = 0;
    DRIVE_ARR[i] = 0;
    REVERSE_ARR[i] = 0;
  }
}

void IEEE754ToArray(unsigned char (&brake_drive_reverse)[8], String ieee754)
{
  for (int i = 0; i < 4; i++) {
      if (i == 0) {
        String byte = ieee754.substring(0, 2);
        unsigned char unsignedByte = hexStringToInt(byte.c_str());
        brake_drive_reverse[4] = unsignedByte;
        //debugln("Byte " + String(i) + ": " + String(unsignedByte));
      } else {
        String byte = ieee754.substring(i * 2, i * 2 + 2);
        unsigned char unsignedByte = hexStringToInt(byte.c_str());
        brake_drive_reverse[4 + i] = unsignedByte;
        //debugln("Byte " + String(i) + ": " + String(unsignedByte));
      }
    }
}

void brake(double brakePot)
{
  String ieee754 = IEEE754(brakePot);
  IEEE754ToArray(BRAKE_ARR, ieee754);
  Serial.print("BRAKE: ");
  Serial.println(brakePot);
  //sendCAN(0x501, BRAKE_ARR);
}

void driveCAR(double driveReversePot, double brakePot) {
  // Input potential will be between 0 - 1024
  driveReversePot = driveReversePot / 1024.0;
  brakePot = brakePot / 1024.0;

  sendCAN(0x502, BUS_VOLTAGE);
  /* ---------- NEUTRAL ----------- */
  if (isNeutral) {
    //debugln("IS NEUTRAL!!");
    sendCAN(0x501, DRIVE_ARR);
  /* ------------ DRIVING ------------- */
  } else if (isDriving) {
    debugln("IS DRIVING, POT: " + String(driveReversePot));

    // Drive potential to IEEE754 string (float point)
    String ieee754 = IEEE754(driveReversePot);

    //Inserting ieee754 values in DRIVE_ARR
    IEEE754ToArray(DRIVE_ARR, ieee754);

    // Apply brake if driving = 0 & brake > 0
    if(driveReversePot == 0 && brakePot > 0) brake(brakePot);
    else sendCAN(0x501, DRIVE_ARR);
    resetArrays();
    /* --------- REVERSING --------- */
  } else if (isReversing) {
    debugln("IS REVERSING, POT: " + String(driveReversePot));

    // Drive potential to IEEE754 string (float point)
    String ieee754 = IEEE754(driveReversePot);

    //Inserting ieee754 values in DRIVE_ARR
    IEEE754ToArray(REVERSE_ARR, ieee754);

    // Apply brake if reversing = 0 else drive
    if(driveReversePot == 0 && brakePot > 0) brake(brakePot);
    else sendCAN(0x501, REVERSE_ARR);
    resetArrays();
  }
}


/* ---------------------- IEEE754 ----------------------------------------*/
String IEEE754(const double potential) {
  double f = static_cast<double>(potential);
  char *bytes = reinterpret_cast<char *>(&f);

  String ieee754 = "";
  for (int i = 0; i < 4; i++) {
    String tempByte = String(bytes[i], HEX);
    //debugln("Byte " + String(i) + ": " + tempByte);

    
    if (tempByte.length() > 2) { //Edgecase for when tempByte is ex: ffffc0 and we only need c0
      //debugln("Byte.length>2: " + tempByte);
      ieee754 += tempByte.substring(tempByte.length() - 2, tempByte.length() - 1) + tempByte.substring(tempByte.length() - 1, tempByte.length());
    } else {
      ieee754 += String(bytes[i], HEX);
      // debugln("Hex " + String(i) + ": " + String(bytes[i], HEX));  //4668
    }
  }

  int remainingZeros = 8 - ieee754.length(); // For when IEEE754.length() < 8
  for (int i = 0; i < remainingZeros; i++) {
    ieee754 = "0" + ieee754;
  }

  return ieee754;
}

double hexStringToInt(const char *hexString) {
  return (double)strtol(hexString, NULL, 16);
}


void setup() {
  Serial.begin(19200);
  while (!Serial) {};
  SPI.begin();
  pinMode(SPI_CS_PIN, OUTPUT);

  init_CAN();

  // Setting controls pins to input
  pinMode(DRIVE_PIN, INPUT_PULLUP);
  pinMode(REVERSE_PIN, INPUT_PULLUP);
  pinMode(NEUTRAL_PIN, INPUT_PULLUP);
  pinMode(BRAKE_PIN, INPUT_PULLUP);

  pinMode(INPUT_BRAKE_PIN, INPUT);
  pinMode(INPUT_GAS_PIN, INPUT);
  
  Wire.begin(8);
}

void loop() {

  // Reads and updates isDriving, isReversing & isBraking
  readPanel();

  int gas_N_reverse_pot = analogRead(INPUT_GAS_PIN);
  int brake_pot = analogRead(INPUT_BRAKE_PIN);

  // Sends drive commands
  //driveCAR(gas_N_reverse_pot, brake_pot);

  unsigned char CAN_available = !digitalRead(CAN_INT_PIN);
  if(CAN_available > 0){

    unsigned long start_time = millis();
    while (CAN_MSGAVAIL == CAN.checkReceive() && millis() - start_time < 0.1) {
        
        // read data,  len: data length, buf: data buf
        unsigned char len = 0;
        
        CAN.readMsgBuf(&len, CAN_buf);

        uint32_t CAN_ID = CAN.getCanId();
        String CAN_data = String(CAN_ID);
        // print the data
        for (int i = 0; i < len; i++) 
        {
           CAN_data += " " + String(CAN_buf[i]);
           //debug(CAN_buf[i]); debug("\t");
        }
        CAN_data += " ";
        Serial.println(CAN_data);
        //Sending CAN_data over I2C
        Wire.beginTransmission(8);
        Wire.write(CAN_data.c_str());
        Wire.endTransmission();   
    }
  }

}

