#include <SPI.h>
#include <Wire.h>
//#include <PID_v1.h>

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

bool inStartScreen = true;

int drivingMode = 0; // 0: reversing, 1: driving, 2: neutral
bool isNeutral = true;
bool isDriving = false;
bool isReversing = false;
bool isBraking = false;

float last_brake_potential = 1/1337; 
float last_gas_N_reverse_potential = 1/1337;

// Input for driving modes ECO/RACING
bool inECO = true;
bool lastInECO = true;
bool hasAppliedECO = false;
int ECOPotential = 0;

// Input for cruise control and buttons to decrease and increase cruise speed
bool inCruiseControl = false;
bool lastInCruiseControl = false;
float potentialCruiseControl = 0.0; // Regulated when in limits of set cruise velocity
float velocityCruiseControl = 0.0; // Set at the initiation of cruise
float cruiseBrakeApplied = 0.0; // Used for accelerating brake if speed does not decrease
float cruiseGasApplied = 0.0; // Used for accelerating gas if speed does not increase
int cruiseSpeedIncDec = 100;
int lastCruiseSpeedIncDec = 100;
const int INPUT_CRUISE_CONTROL_INCREASE_PIN = A3; // PIN INCREASE CRUISE SPEED
const int INPUT_CRUISE_CONTROL_DECREASE_PIN = A5; // PIN DECREASE CRUISE SPEED

double vehicleVelocity = 0.0; // Velocity of vehicle
double lastVehicleVelocity = 0.0; // Previous iteration of vehicle velocity
unsigned long lastTimePointVelocityFetched = millis();

double maxBrakeBusCurrent = -50;
double busCurrent = 0.0;
unsigned long lastTimePointBusCurrentFetched = millis();

unsigned char CAN_buf[8]; // Storing of incoming CAN data


// extractBytesToDecimal() extracts a specified number of bytes (int numBytes) 
// from a IEEE754 string (String data) and return normal decimal number
double extractBytesToDecimal(String data, int startByte, int numBytes) {
  // Extract the specified number of bytes from the string
  String byteStr = data.substring(startByte, startByte + numBytes);

  // Calculate startbyte index position ex. startByte: 4 = index: 14 (65 160 0 0 68 (250 0 0 1027))
  int startIndex = 0;
  int byteCounter = 0; // Bytes inc. for each " "
  for(int i = 0; i < data.length(); i++)
  {

    if(byteCounter == startByte)
    {
      startIndex = i;
      break;
    }

    if(data.substring(i, i+1) == " ")
    {
      byteCounter++;
    }
  
  }
  //debugln("Start index: " + String(startIndex));

  byte bytes[numBytes];
  byteCounter = 0;
  String byte_data = "";
  for(int i = startIndex; i < data.length(); i++)
  {

    String data_substr = data.substring(i, i+1);

    if(byteCounter == numBytes)
    {
      break;
    }
    else if(data_substr == " ")
    {
      //debugln(byte_data);
      bytes[byteCounter] = (byte) strtoul(byte_data.c_str(), NULL, 10);
      byteCounter++;
      byte_data = "";
    }
    else
    {
      byte_data += data_substr; 
    }

  }

  float value;
  memcpy(&value, bytes, numBytes);
  // Return the decimal value
  return value;
}


// Initialization of CAN shield to 500KBPS
void init_CAN() {
  while (CAN_OK != CAN.begin(CAN_500KBPS)) {  // init can bus : baudrate = 500k
    Serial.println("CAN init fail, retry...");
    delay(100);
  }
  Serial.println("CAN init ok!");
}

// ---------------- DRIVING CAR ----------------------------

// Sends a CAN message to MC
void sendCAN(int channel, unsigned char msg[8]) 
{
  CAN.sendMsgBuf(channel, 0, 8, msg);
}


// Reads @DRIVE_PIN, REVERSE_PIN & NEUTRAL_PIN 
// readPanel() changes the current car state (drive, reverse, neutral)
void readPanel() {
  if (drivingMode == 1) {
    isDriving = true;

    isBraking = false;
    isReversing = false;
    isNeutral = false;
    return;
  } else if (drivingMode == 0) {
    isReversing = true;

    isDriving = false;
    isBraking = false;
    isNeutral = false;
    return;
  } else if (drivingMode == 2) {
    isNeutral = true;

    isBraking = false;
    isDriving = false;
    isReversing = false;
    return;
  }

}

// Sets the values in DRIVE_ARR, BRAKE_ARR or REVERSE_ARR to zero.
// This stops the car from braking, reversing and reversing
void resetArrays()
{
  for(int i = 4; i < 8; i++)
  {
    BRAKE_ARR[i] = 0;
    DRIVE_ARR[i] = 0;
    REVERSE_ARR[i] = 0;
  }
}

// Pass reference to DRIVE_ARR, BRAKE_ARR or REVERSE_ARR to the value in @ieee754 string 
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

// Used to apply break to the vehicle
void brake(double brakePot)
{

  if(busCurrent < maxBrakeBusCurrent)
  {
    brakePot = 0;
  }

  String ieee754 = IEEE754(brakePot);
  IEEE754ToArray(BRAKE_ARR, ieee754);
  
  Serial.print("BRAKE: ");
  Serial.println(brakePot);
  sendCAN(0x501, BRAKE_ARR);
}

void driveCAR(float driveReversePot, float brakePot) {
  // Input potential will be between 0 - 1024
  driveReversePot = driveReversePot / 1023;//max_gas_potential;
  brakePot = brakePot / 1023;//max_brake_potential;

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


  // SAFETY PIN
  pinMode(4, INPUT_PULLUP);

  Wire.begin(); // For I2C communication to LoRa
}


// -------------- CRUISE CONTROL ---------------------

void enterCruiseControl()
{
  // ------------------------------------ TEEEEEESSSSSSSSSTTTTTTTTTTT 
  if(lastInCruiseControl == 1 && inCruiseControl == 0)
  {
    inCruiseControl = false;
    velocityCruiseControl = 0;
  }
  if(lastInCruiseControl == 0 && inCruiseControl == 1)
  {
    inCruiseControl = true;
    velocityCruiseControl = abs(vehicleVelocity);
  }
}





// Read input values and update cruise control values if cruise is activated
void applyCruiseControl(float& gas_N_reverse_potential, float& brake_potential)
{
  double gas_increment = 3; // CHANGE THIS FOR HARDER ACCELERATION
  double brake_increment = 2;

  // If cruise control is activated and mode is not in reverse
  if(inCruiseControl && drivingMode != 0)
  {
    // ---------------------------------------- TESTTTTTTTTTTTTTTTTTTTTTTTTTTTTT --------------------
    int velocityIncreaseCruise = 2; // For how much to increase cruising speed when pressing increase/decrease
    velocityCruiseControl += (cruiseSpeedIncDec-lastCruiseSpeedIncDec)*velocityIncreaseCruise;
    if(velocityCruiseControl < 0){velocityCruiseControl = 0;} // if velocity 


    // Compute the error between desired and current speed
    float velocityError = velocityCruiseControl - vehicleVelocity; // SetVelocity - CurrentVelocity
    float velocityErrorOffset = 0.1;
    float velocityBrakeErrorOffset = -15;

    if(velocityCruiseControl == 0)
    {
      gas_N_reverse_potential = 0;
      return;
    }

    // If the speed is too low, apply more gas
    if(velocityError > velocityErrorOffset && millis() - lastTimePointVelocityFetched < 1000)
    {
      cruiseBrakeApplied = 0.0;
      brake_potential = 0.0;
      cruiseGasApplied += gas_increment;
      gas_N_reverse_potential = cruiseGasApplied;
    }
    // If the speed is too high, apply more brake
    else if(velocityError < velocityBrakeErrorOffset && millis() - lastTimePointVelocityFetched < 1000)
    {
      cruiseGasApplied = 0.0;
      gas_N_reverse_potential = 0.0;
      cruiseBrakeApplied += brake_increment;
      brake_potential = cruiseBrakeApplied;
    }
    // If the speed is within a tolerance range, maintain the current speed
    else
    {
      float velocityGapError = vehicleVelocity - lastVehicleVelocity;
      cruiseGasApplied = 0.0;
      cruiseBrakeApplied = 0.0;
      brake_potential = 0;

      if(velocityGapError > 0.02)
      {
        potentialCruiseControl -= 1;
        gas_N_reverse_potential = potentialCruiseControl;
      }
      else if(velocityGapError < (-0.02))
      {
        potentialCruiseControl += 1;
        gas_N_reverse_potential = potentialCruiseControl;
      }
      else
      {
        gas_N_reverse_potential = potentialCruiseControl;
      }
      
    
    }

    // Set the gas and brake potentials based on the applied values
    //gas_N_reverse_potential = potentialCruiseControl + cruiseGasApplied;
    //brake_potential = cruiseBrakeApplied;

    /* If potentials are greater or smaller then MIN and MAX
    if(gas_N_reverse_potential > max_gas_potential){gas_N_reverse_potential=max_gas_potential;}
    if(gas_N_reverse_potential < min_gas_potential){gas_N_reverse_potential=min_gas_potential;}
    if(brake_potential > max_brake_potential){brake_potential=max_brake_potential;}
    if(brake_potential < min_brake_potential){brake_potential=min_brake_potential;}*/

    if(gas_N_reverse_potential > 1023){gas_N_reverse_potential=1023;}
    if(gas_N_reverse_potential < 0){gas_N_reverse_potential=0;}
    if(brake_potential > 1023){brake_potential=1023;}
    if(brake_potential < 0){brake_potential=0;}

  }
}

// -------------- // CRUISE CONTROL ---------------------

// ------------------ ECO CONTROL ------------------------
// Read input values and update cruise control values if cruise is activated
void applyECOControl(float& gas_N_reverse_potential)
{ 
  float potential_gap_error = gas_N_reverse_potential - last_gas_N_reverse_potential;

  if(potential_gap_error > 10 && hasAppliedECO)
  {
    int ECOIncrementStep = 1; // Increase this for harder ACCELERATION in ECO

    /*
    debug("GAS BEFORE ECO: " + String(gas_N_reverse_potential));
    */
    ECOPotential += ECOIncrementStep;
    gas_N_reverse_potential = ECOPotential;
    /*
    debug(", GAS AFTER ECO: " + String(gas_N_reverse_potential));
    debugln();
    */
  }
  else if(potential_gap_error > 5 && !hasAppliedECO)
  {
    hasAppliedECO = true;
    ECOPotential = last_gas_N_reverse_potential;
  }
  else
  {
    ECOPotential = 0;
    hasAppliedECO = false;
  }

}

// ------------------ // ECO CONTROL ------------------------



void loop() 
{
  float gas_N_reverse_potential = 0;
  float brake_potential = 0;
  Wire.requestFrom(8, 7); // Requesting data from screen arduino containing 6 bytes
  while(Wire.available())
  {
    gas_N_reverse_potential = Wire.read(); // 5* since the value is divided by 5 before sending
    brake_potential = Wire.read();
    drivingMode = Wire.read();
    inCruiseControl = Wire.read();
    inECO = Wire.read();
    inStartScreen = Wire.read();
    cruiseSpeedIncDec = Wire.read();
    /*
    debugln("Requested Gas: " + String(5*gas_N_reverse_potential));
    debugln("Requested Brake: " + String(5*brake_potential));
    debugln("Requested Drivingmode: " + String(drivingMode));
    debugln("Requested Cruisecontrol: " + String(inCruiseControl));
    debugln("Requested Eco: " + String(inECO));
    debugln("Requested inStartScreen: " + String(inStartScreen));
    debugln("Requested CruiseIncDec: " + String(cruiseSpeedIncDec));
    */
  }

  unsigned char CAN_available = !digitalRead(CAN_INT_PIN);
  if(CAN_available > 0){
    unsigned long start_time = millis();
    while (CAN_MSGAVAIL == CAN.checkReceive() && millis() - start_time < 120) {
        // read data,  len: data length, buf: data buf
        unsigned char len = 0;
        CAN.readMsgBuf(&len, CAN_buf);
        String CAN_ID = String(CAN.getCanId());
        String CAN_data = "";
        // print the data
        for (int i = 0; i < len; i++) 
        {
           CAN_data += String(CAN_buf[i]) + ' ';
           //debug(CAN_buf[i]); debug("\t");
        }
        //Serial.println(CAN_data);
        String full_CAN_data = CAN_ID + ' ' + CAN_data;
        //debugln("Full CAN data: " + full_CAN_data);
        //Sending CAN_data over I2C
        Wire.beginTransmission(8);
        Wire.write(full_CAN_data.c_str());
        Wire.endTransmission();
        Wire.beginTransmission(9);
        Wire.write(full_CAN_data.c_str());
        Wire.endTransmission();
        // Extract vehicle velocity speed
        if(CAN_ID == "1027")
        {
          vehicleVelocity = extractBytesToDecimal(CAN_data, 4, 4);
          lastTimePointVelocityFetched = millis();
          //debugln("Vehicle velocity: " + String(vehicleVelocity));
        }
        // Extract bus_current
        if(CAN_ID == "1026")
        {
          busCurrent = extractBytesToDecimal(CAN_data, 4, 4);
          //debugln("Bus current: " + String(busCurrent));
          //delay(800);
        }
    }
  }
  //delay(10); IF I2C NOT WORKING UN-COMMENT THIS <------------
  readPanel();
  enterCruiseControl();
  
  /*
  debugln("gas_N_reverse_potential before: " + String(gas_N_reverse_potential) + ", brake: " + String(brake_potential));
  */

  applyCruiseControl(gas_N_reverse_potential, brake_potential); // IF IN CRUISE CONTROL UPDATE gas_N_reverse_potential and brake_potential
  applyECOControl(gas_N_reverse_potential);

  // Sends drive commands if not in start screen
  if(!inStartScreen)
  {
    driveCAR(gas_N_reverse_potential, brake_potential);
  }
  last_gas_N_reverse_potential = gas_N_reverse_potential;
  last_brake_potential = brake_potential;
  lastVehicleVelocity = vehicleVelocity; // Set last vehicle velocity to current velocity
  /*
  debugln("gas_N_reverse_potential after: " + String(gas_N_reverse_potential) + ", brake: " + String(brake_potential));
  debugln("Vehicle velocity: " + String(vehicleVelocity) + ", velocityCruiseControl: " + String(velocityCruiseControl));
  */
  lastCruiseSpeedIncDec = cruiseSpeedIncDec;
  lastInCruiseControl = inCruiseControl;
}
