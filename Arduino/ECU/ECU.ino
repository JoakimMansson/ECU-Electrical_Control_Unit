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
    }

    void sendMsg(unsigned int channel, unsigned char msg[8])
    {
      // byte mcp2518fd_sendMsg(const byte *buf, byte len, unsigned long id, byte ext, byte rtr, bool wait_sent);
      CAN.sendMsgBuf(msg, 8, channel, 0, 0, false);
      Serial.println("Skickar");
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

    int getBrakePot()
    {
      return analogRead(brakePin);
    }

    int getGasPot()
    {
      return analogRead(gasPin);
    }
};

class Car
{
  private:    
    unsigned char DRIVE_ARR[8] = {0, 128, 132, 68, 0, 0, 0, 0};
    unsigned char REVERSE_ARR[8] = {0, 128, 132, 44, 0, 0, 0, 0};
    unsigned char NEUTRAL_ARR[8] = {0, 128, 132, 44, 0, 0, 0, 0};
    unsigned char BRAKE_ARR[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    bool isNeutral = true;
    bool isDriving = false;
    bool isReversing = false;
    bool isBraking = false;

    int DRIVE_PIN = 3;
    int REVERSE_PIN = 4;
    int NEUTRAL_PIN = 5;
    int BRAKE_PIN = 6;

    SendCan *CANBUS;

  public:

    Car(const SendCan CAN)
    {
      // Setting controls pins to input
      pinMode(DRIVE_PIN, INPUT_PULLUP);
      pinMode(REVERSE_PIN, INPUT_PULLUP);
      pinMode(NEUTRAL_PIN, INPUT_PULLUP);
      pinMode(BRAKE_PIN, INPUT_PULLUP);

      CANBUS = &CAN;
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


    void driveCAR(int driveReversePot, int brakePot)
    {
        if(isNeutral)
        {
          CANBUS->sendMsg(0x501, NEUTRAL_ARR);
          Serial.println("IS NEUTRAL!!");
        }
        else if(isDriving)
        {
          Serial.println("IS DRIVING, POT: " + String(driveReversePot));
          
          // Drive potential to IEEE754 string
          String ieee754 = IEEE754(driveReversePot);
          Serial.println("IEEE754: " + ieee754);

          //Inserting ieee754 values in DRIVE_ARR
          for(int i = 0; i < 4; i++)
          {
            
            if(i == 0)
            {
              String byte = ieee754.substring(0,2);
              unsigned char intByte = hexStringToInt(byte.c_str());
              DRIVE_ARR[4] = intByte;
              Serial.println("Byte " + String(i) + ": " + intByte);              
            }
            else
            {
              String byte = ieee754.substring(i+2,2);
              unsigned char intByte = hexStringToInt(byte.c_str());
              DRIVE_ARR[4+i] = intByte;
              Serial.println("Byte " + String(i) + ": " + intByte);
            }
          }


          CANBUS->sendMsg(0x501, DRIVE_ARR);
        }
        else if(isReversing)
        {
          Serial.println("IS REVERSING!!");
          
        }
        else if(isBraking)
        {
          Serial.println("IS BRAKING!!");

        }
    }

    String IEEE754(const int potential)
    {
      float f = static_cast<float>(potential);
      char *bytes = reinterpret_cast<char *>(&f);
          
      String ieee754 = "";
      for(int i = 0; i < 4; i++)
        {
          String tempByte = String(bytes[i],HEX);

          //Edgecase for when tempByte is ex: ffffc0 and we only need c0
          if(tempByte.length() > 2)
          {
            //Serial.println("Byte.length>2: " + tempByte);
            ieee754 += tempByte.substring(tempByte.length()-2, 1) + tempByte.substring(tempByte.length()-1, 1);
          }
          else
          {
            ieee754 += String(bytes[i], HEX);
            //Serial.println("Hex " + String(i) + ": " + String(bytes[i], HEX));  //4668
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

    unsigned char hexStringToInt(const char* hexString) 
    {
      return (char)strtol(hexString, NULL, 16);
    }
  
};



int CS_PIN = 9;
SendCan sendCAN(CS_PIN);
Car car(sendCAN);
ReceivePotential receivePot;
void setup() 
{
  Serial.begin(9600);
  Serial.println("Serial initalized");
  sendCAN.initCAN();
}


void loop()
{
  car.readPanel();
  car.driveCAR(receivePot.getGasPot(), receivePot.getBrakePot());
}