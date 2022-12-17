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

    void sendMsg(unsigned char msg[8])
    {
      //mCAN.sendMsgBuf(0, 0, 8, msg);
      Serial.println("hej");
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
    receivePotential()
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

int CS_PIN = 9;
ReceivePotential receivePot;
SendCan canSend(CS_PIN);
void setup() 
{
  Serial.begin(9600);
  Serial.println("Serial initalized");

  mcp2515_can CAN = canSend.getCan();
  while(CAN_OK != CAN.begin(CAN_500KBPS))
    {
      Serial.println("CAN BUS Shield init failed");
      Serial.println("Init CAN BUS Shield again");
      delay(100);
    }
    Serial.println("CAN BUS Shield init ok");
}


class Car()
{
  private:    
    int DRIVE_ARR[] = {0, 128, 132, 68, 0, 0, 0, 0};
    int REVERSE_ARR[] = {0, 128, 132, 44, 0, 0, 0, 0};
    int NEUTRAL_ARR[] = {0, 128, 132, 44, 0, 0, 0, 0};
    int BRAKE_ARR[] = {0, 0, 0, 0, 0, 0, 0, 0};
    bool isDriving = false;
    bool isBraking = false;
    bool isReversing = false;

  public:

    Car()
    {
      while(true):
      if()
    }
    void 
}




void loop()
{
  Serial.println("Brake potential: ");
  Serial.print(receivePot.getBrakePot());
  Serial.println("");
  Serial.println("Gas potential: ");
  Serial.println(receivePot.getGasPot());
  delay(500);
}
