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

bool isNeutral = true;
bool isDriving = false;
bool isReversing = false;
bool isBraking = false;

// Input for drive, reverse & neutral
const int DRIVE_PIN = 7;
const int REVERSE_PIN = 8;
const int NEUTRAL_PIN = 6;

// Read gas and brake raw potentials
const int INPUT_BRAKE_PIN = A0; 
const int INPUT_GAS_PIN = A1;
float max_gas_potential = 0;
float min_gas_potential = 99999;
float max_brake_potential = 0;
float min_brake_potential = 99999;
float last_brake_potential = 1/1337; 
float last_gas_N_reverse_potential = 1/1337;


// Input for driving modes ECO/RACING
bool inECO = true;
const int INPUT_DRIVING_MODES_PIN = 3;

// Input for cruise control and buttons to decrease and increase cruise speed
bool inCruiseControl = false;
float potentialCruiseControl = 0.0; // Regulated when in limits of set cruise velocity
float velocityCruiseControl = 0.0; // Set at the initiation of cruise
float cruiseBrakeApplied = 0.0; // Used for accelerating brake if speed does not decrease
float cruiseGasApplied = 0.0; // Used for accelerating gas if speed does not increase
const int INPUT_CRUISE_CONTROL_PIN = A2; // PIN ACTIVATE/DEACTIVATE CRUISE
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

  /*
    DRIVE_PIN = 7, REVERSE_PIN = 8, NEUTRAL_PIN = 6
    INPUT_BRAKE_PIN = A0, INPUT_GAS_PIN = A1
    INPUT_CRUISE_CONTROL_PIN = 2, INPUT_CRUISE_CONTROL_INCREASE_PIN = 1, INPUT_CRUISE_CONTROL_DECREASE_PIN = 0
    INPUT_DRIVING_MODES_PIN = 3
  */

  // Pins for DRIVE, REVERSE & NEUTRAL
  pinMode(DRIVE_PIN, INPUT_PULLUP);
  pinMode(REVERSE_PIN, INPUT_PULLUP);
  pinMode(NEUTRAL_PIN, INPUT_PULLUP);

  // Pins for potential input GAS & BRAKE
  pinMode(INPUT_BRAKE_PIN, INPUT);
  pinMode(INPUT_GAS_PIN, INPUT);

  // Pins for CRUISE CONTROL
  pinMode(INPUT_CRUISE_CONTROL_PIN, INPUT_PULLUP);
  pinMode(INPUT_CRUISE_CONTROL_INCREASE_PIN, INPUT_PULLUP);
  pinMode(INPUT_CRUISE_CONTROL_DECREASE_PIN, INPUT_PULLUP);

  // SAFETY PIN
  pinMode(4, INPUT_PULLUP);

  // Pins for ECO & RACING mode
  pinMode(INPUT_DRIVING_MODES_PIN, INPUT_PULLUP);

  Wire.begin(9); // For I2C to screen
  Wire.begin(8); // For I2C communication to LoRa
  

  /* USE THIS WHEN SETTING POTENTIALS WITH SCREEN
  debugln("SET POTENTIALS NOW!")
  unsigned long start_time = millis();
  int mean_potential_counter = 0;
  float mean_gas_reverse_potential = 0;
  float mean_brake_potential = 0;
  while(millis() - start_time < 15000) // 15 seconds to put starting potentials
  {
    float gas_N_reverse_potential = analogRead(INPUT_GAS_PIN);
    float brake_potential = analogRead(INPUT_BRAKE_PIN);
    
    if(mean_potential_counter == 4)
    {
      gas_N_reverse_potential = mean_gas_reverse_potential/mean_potential_counter;
      brake_potential = mean_brake_potential/mean_potential_counter;

      if(gas_N_reverse_potential > max_gas_potential){max_gas_potential = gas_N_reverse_potential; debugln("New max_gas: " + String(max_gas_potential));}
      if(gas_N_reverse_potential < min_gas_potential){min_gas_potential = gas_N_reverse_potential; debugln("New min_gas: " + String(min_gas_potential));}
      if(brake_potential > max_brake_potential){max_brake_potential = brake_potential; debugln("New max_brake: " + String(max_brake_potential));}
      if(brake_potential < min_brake_potential){min_brake_potential = brake_potential; debugln("New min_brake: " + String(min_brake_potential));}

      mean_potential_counter = 0;
      mean_gas_reverse_potential = 0;
      mean_brake_potential = 0;
    }
    else
    {
      mean_gas_reverse_potential += gas_N_reverse_potential;
      mean_brake_potential += brake_potential;
      mean_potential_counter++;
    }
  }
  debugln("POTENTIALS SET:")
  debugln("Max gas: " + String(max_gas_potential) + ", Min gas: " + String(min_gas_potential));
  debugln("Max brake: " + String(max_brake_potential) + ", Min brake: " + String(min_brake_potential));
  delay(5000);*/

  // FOR TESTING
  isNeutral = false;
  isDriving = true;
  inCruiseControl = true;
  velocityCruiseControl = 12;
}

int currentECOButtonHoldIterations = 0;
void toggleECOMode()
{
  bool buttonPressECO = digitalRead(INPUT_DRIVING_MODES_PIN) == LOW;
  
  if (currentECOButtonHoldIterations == 1000) {
    currentECOButtonHoldIterations = 0;

    if (buttonPressECO == true) 
    {
      inECO = !inECO;
      debugln("inECO = " + String(inECO));
    } 

  }
  else
  {
    currentECOButtonHoldIterations++;
  }  
}

int currentCruiseButtonHoldIterations = 0;
void toggleCruiseMode()
{
  bool buttonPressCruise = digitalRead(INPUT_CRUISE_CONTROL_PIN) == LOW;
  
  if (currentCruiseButtonHoldIterations == 3) {
    currentCruiseButtonHoldIterations = 0;

    if (buttonPressCruise == true && isDriving == true) 
    {
      inCruiseControl = !inCruiseControl;

      debugln("inCruise = " + String(inCruiseControl));

      if(inCruiseControl == true)
      {
        velocityCruiseControl = vehicleVelocity;
        debugln("VelocityCruise = " + String(velocityCruiseControl));
      }
      
    } 

  }
  else
  {
    currentCruiseButtonHoldIterations++;
    debugln("CruiseControlButtonHoldIterations: " + String(currentCruiseButtonHoldIterations));
  }  
}

// -------------- ECO CONTROL ---------------------
// Read input values and update cruise control values if cruise is activated
void applyCruiseControl(float& gas_N_reverse_potential, float& brake_potential)
{
  double gas_increment = 3;
  double brake_increment = 2;
  // If cruise control is activated
  if(inCruiseControl)
  {
    // Increment or decrement cruise control speed if corresponding buttons are pressed
    if(digitalRead(INPUT_CRUISE_CONTROL_INCREASE_PIN) == 0)
    {
      velocityCruiseControl += 2;
      //debugln("New cruise_velocity: " + String(velocityCruiseControl));
    }
    if(digitalRead(INPUT_CRUISE_CONTROL_DECREASE_PIN) == 0)
    {
      velocityCruiseControl -= 2;
      if(velocityCruiseControl < 0){velocityCruiseControl = 0;}
      //debugln("New cruise_velocity: " + String(velocityCruiseControl));
    }

    // Compute the error between desired and current speed
    float velocityError = velocityCruiseControl - vehicleVelocity; // SetVelocity - CurrentVelocity
    float velocityErrorOffset = 0.1;
    float velocityBrakeErrorOffset = -2;

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
      else if(velocityGapError < -0.02)
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

// -------------- ECO CONTROL ---------------------
// Read input values and update cruise control values if cruise is activated
void applyECOControl(float& gas_N_reverse_potential, float& brake_potential)
{

}


void loop() {
  if(digitalRead(4) == LOW)
  {
  
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
        debugln("Full CAN data: " + full_CAN_data);
        
        //Sending CAN_data over I2C
        Wire.beginTransmission(8);
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
          debugln("Bus current: " + String(busCurrent));
          //delay(800);
        }
    }
  }

  // Reads and updates isDriving, isReversing & isBraking
  readPanel();

  // Get driving potentials
  float gas_N_reverse_potential = analogRead(INPUT_GAS_PIN);
  float brake_potential = analogRead(INPUT_BRAKE_PIN);

  // Transmit I2C potentials to screen
  Wire.beginTransmission(10);
  Wire.write(("." + String(int(gas_N_reverse_potential))).c_str());
  Wire.write((", " + String(int(vehicleVelocity))).c_str());
  Wire.write((String(int(brake_potential))).c_str());
  Wire.endTransmission();   
  
  debugln("IN_CRUISE=" + String(inCruiseControl));
  // Exit cruise if sudden gas or brake is applied
  debugln("Last_brake:" + String(last_brake_potential));
  if((brake_potential-250 > last_brake_potential) && last_brake_potential != 1/1337) {inCruiseControl = false;}
  last_gas_N_reverse_potential = gas_N_reverse_potential;
  last_brake_potential = brake_potential;

  debugln("gas_N_reverse_potential before: " + String(gas_N_reverse_potential) + ", brake: " + String(brake_potential));
  
  
  applyCruiseControl(gas_N_reverse_potential, brake_potential); // IF IN CRUISE CONTROL UPDATE gas_N_reverse_potential and brake_potential


  lastVehicleVelocity = vehicleVelocity; // Set last vehicle velocity to current velocity
  debugln("gas_N_reverse_potential after: " + String(gas_N_reverse_potential) + ", brake: " + String(brake_potential));
  debugln("Vehicle velocity: " + String(vehicleVelocity) + ", velocityCruiseControl: " + String(velocityCruiseControl));

  // Sends drive commands
  driveCAR(gas_N_reverse_potential, brake_potential);
  
  // LOGIC for CRUISE CONTROL
  toggleCruiseMode();

  // Check for input of ECO mode
  toggleECOMode();

  
  }
}

