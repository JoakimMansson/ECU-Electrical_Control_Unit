#include <Arduino.h>
#include <mcp2515_can.h>
#include <SPI.h>


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
    }

    void sendMsg(unsigned char msg[8])
    {
      CAN.sendMsgBuf(0, 0x700, 0, 0, 8, msg);
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

    Car(const SendCan &CAN)
    {
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
          //CANBUS->sendMsg(NEUTRAL_ARR);
          Serial.println("IS NEUTRAL!!");
        }
        else if(isDriving)
        {
     

          Serial.println("IS DRIVING, POT: " + String(driveReversePot));
          
          float f = static_cast<int>(driveReversePot);
          char *bytes = reinterpret_cast<char *>(&f);
          
          String ieee754 = "";
          for(int i = 0; i < 4; i++)
          {
            if(bytes[i] > 256)
            {
              String tempByte = String((bytes[i], HEX));
              ieee754 += tempByte.substring(tempByte.length()-2) + tempByte.substring(tempByte.length()-1);
            }
            else
            {
              ieee754 += String((bytes[i], HEX));
            }
            //DRIVE_ARR[7-i] = (bytes[i],HEX)
            Serial.println("Hex " + String(i) + ": " + String(bytes[i], HEX));  //4668
          }

          Serial.println("IEEE754: " + String(ieee754));
          
          //Serial.println(hex);
          //Serial.print(*bytes);
          delay(5000);
        }
        else if(isReversing)
        {
          Serial.println("IS REVERSING!!");
          
        }
        else if(isBraking)
        {
          Serial.println("IS BRAKING!!");

        }

        /*
        Serial.println("Brake potential: ");
        Serial.print(brakePot);
        Serial.println("");
        Serial.println("Gas potential: ");
        Serial.println(driveReversePot);
        delay(500);
        */
    }



/*
    void potToDriveArr(int drivePot, int &arr[8])
    {
      unsigned char *temp[8] = arr;
    
    }
*/
  
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