#include <Arduino.h>
#include <mcp2515_can.h>
#include <SPI.h>
#include <string.h>


class SendCan
{
  private:

    mcp2515_can CAN;
  public:
  
    SendCan(int mSPI_CS_PIN): CAN(mSPI_CS_PIN)
    {
    }

    void initCAN()
    {
      while(CAN_OK != CAN.begin(CAN_500KBPS))
      {
      Serial.println("CAN BUS Shield init failed");
      Serial.println("Init CAN BUS Shield again");
      delay(100);
      }
      Serial.println("CAN BUS Shield init ok");
      //delay(5000);
      CAN.enableTxInterrupt(true);
      //CAN.ini
    }

    void sendCAN(long unsigned int channel, const unsigned char msg[8])
    {
      // byte mcp2515_can::sendMsgBuf(byte status, unsigned long id, byte ext, byte rtrBit, byte len, volatile const byte* buf) {
        /*
      for(int i = 4; i < 8; i++)
      {
        Serial.println("Byte " + String(i-4) + ": " + msg[i]);
      }
      */
      CAN.clearBufferTransmitIfFlags();
      byte status = CAN.sendMsgBuf(channel, 0, 0, 8, msg, false);
      //delay(5000);
    }

    mcp2515_can getCan()
    {
      return CAN;
    }
};

class ReceivePotential
{
  private:
    int brakePin = A0;
    int gasPin = A1;

  public:

    ReceivePotential()
    {
      pinMode(brakePin, INPUT);
      pinMode(gasPin, INPUT);
    }

    double getBrakePot()
    {
      return analogRead(brakePin);
    }

    double getGasPot()
    {
      return analogRead(gasPin);
    }
};

class Car
{
  private:    
    unsigned char DRIVE_ARR[8];
    unsigned char REVERSE_ARR[8] = {0, 128, 132, 44, 0, 0, 0, 0};
    unsigned char NEUTRAL_ARR[ 8] ={0, 128, 132, 44, 0, 0, 0, 0};
    unsigned char BRAKE_ARR[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    unsigned char BUS_VOLTAGE[8] = {0,0,0,0,0,0,128,63};

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

    SendCan *CANBUS;

  public:

    Car(const SendCan CAN)
    {
      // Setting controls pins to input
      pinMode(DRIVE_PIN, INPUT_PULLUP);
      pinMode(REVERSE_PIN, INPUT_PULLUP);
      pinMode(NEUTRAL_PIN, INPUT_PULLUP);
      pinMode(BRAKE_PIN, INPUT_PULLUP);

      pinMode(INPUT_BRAKE_PIN, INPUT);
      pinMode(INPUT_GAS_PIN, INPUT);
      
      CANBUS = &CAN;
    }

    int readGas()
    {
      return analogRead(INPUT_GAS_PIN);
    }

    int readBrake()
    {
      return analogRead(INPUT_BRAKE_PIN);
    }
    
    void readPanel()
    {
      if(digitalRead(DRIVE_PIN) == LOW)
      {
        isDriving = true;

        isBraking = false;
        isReversing = false;
        isNeutral = false;
        return;
      }
      else if(digitalRead(REVERSE_PIN) == LOW)
      {
        isReversing = true;

        isDriving = false;
        isBraking = false;
        isNeutral = false;
        return;
      }
      else if(digitalRead(NEUTRAL_PIN) == LOW)
      {
        isNeutral = true;
        
        isBraking = false;
        isDriving = false;
        isReversing = false;
        return;
      }
      else if(digitalRead(BRAKE_PIN) == LOW)
      {
        isBraking = true;
        
        isNeutral = false;
        isDriving = false;
        isReversing = false;
        return;
      }
    }


    void driveCAR(double driveReversePot, double brakePot)
    {
        driveReversePot = driveReversePot/1024.0;
        brakePot = brakePot/1024.0;

        CANBUS->sendCAN(0x502, BUS_VOLTAGE);
        if(isNeutral)
        {
          Serial.println("IS NEUTRAL!!");
          CANBUS->sendCAN(0x501, NEUTRAL_ARR);
        }
        else if(isDriving)
        {
          Serial.println("IS DRIVING, POT: " + String(driveReversePot));
          
          // Drive potential to IEEE754 string
          String ieee754 = IEEE754(driveReversePot);
          Serial.println("IEEE754: " + ieee754);
          delay(3000);

          //Inserting ieee754 values in DRIVE_ARR
          for(int i = 0; i < 4; i++)
          {
            if(i == 0)
            {
              String byte = ieee754.substring(0,2);
              unsigned char unsignedByte = hexStringToInt(byte.c_str());
              DRIVE_ARR[4] = unsignedByte;
              //Serial.println("Byte " + String(i) + ": " + String(unsignedByte));              
            }
            else
            {
              String byte = ieee754.substring(i*2,i*2+2);
              unsigned char unsignedByte = hexStringToInt(byte.c_str());
              DRIVE_ARR[4+i] = unsignedByte;
              //Serial.println("Byte " + String(i) + ": " + String(unsignedByte));
            }
          }

          /*
          for(int i = 0; i < 4; i++)
          {
            Serial.println("Byte " + String(i) + ": " + DRIVE_ARR[i]);
          }
          */

          //delay(5000);
          CANBUS->sendCAN(0x500, DRIVE_ARR);
        }
        else if(isReversing)
        {
          Serial.println("IS REVERSING, POT: " + String(driveReversePot));
        }
        else if(isBraking)
        {
          Serial.println("IS BRAKING, POT: " + String(brakePot));
        }
    }

    String IEEE754(const double potential)
    {
      double f = static_cast<double>(potential);
      char *bytes = reinterpret_cast<char *>(&f);
          
      String ieee754 = "";
      for(int i = 0; i < 4; i++)
        {
          String tempByte = String(bytes[i],HEX);
          //Serial.println("Byte " + String(i) + ": " + tempByte);

          //Edgecase for when tempByte is ex: ffffc0 and we only need c0
          if(tempByte.length() > 2)
          {
            //Serial.println("Byte.length>2: " + tempByte);
            ieee754 += tempByte.substring(tempByte.length()-2, tempByte.length()-1) + tempByte.substring(tempByte.length()-1, tempByte.length());
          }
          else
          {
            ieee754 += String(bytes[i], HEX);
           // Serial.println("Hex " + String(i) + ": " + String(bytes[i], HEX));  //4668
          }
        }

      // For when IEEE754.length() < 8
      int remainingZeros = 8 - ieee754.length();
      for(int i = 0; i < remainingZeros; i++)
      {
        ieee754 = "0" + ieee754;
      }

      return ieee754;
    }

    double hexStringToInt(const char* hexString) 
    {
      return (double)strtol(hexString, NULL, 16);
    }
  
};


int CS_PIN = 9;
SendCan sendCAN(CS_PIN);
Car car(sendCAN);
void setup() 
{
  Serial.begin(9600);
  Serial.println("Serial initalized");
  sendCAN.initCAN();
}


void loop()
{
  int gas_pot = car.readGas();
  int brake_pot = car.readBrake();
  car.readPanel();
  car.driveCAR(gas_pot, brake_pot);
}