#include <SPI.h>
#include <mcp2515_can.h>

int DRIVE_ARR[] = {0, 128, 132, 68, 0, 0, 0, 0};
int REVERSE_ARR[] = {0, 128, 132, 44, 0, 0, 0, 0};
int NEUTRAL_ARR[] = {0, 128, 132, 44, 0, 0, 0, 0};
int BRAKE_ARR[] = {0, 0, 0, 0, 0, 0, 0, 0};

class SendCan
{
  private:
    int SPI_CS_PIN = 9;
    mcp2515_can CAN;
  public:
    SendCan() : CAN(SPI_CS_PIN)
    {
      while(CAN_OK != CAN.begin(CAN_500KBPS))
      {
        Serial.println("CAN BUS Shield init failed");
        Serial.println("Init CAN BUS Shield again");
      }
      Serial.println("CAN BUS Shield init ok");
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

ReceivePotential receivePot;
//SendCan canSend;
void setup() 
{
  Serial.begin(9600);
  //receivePot = ReceivePotential();
  //canSend = SendCan();
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
