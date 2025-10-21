#include <Arduino.h>
#include <Dynamixel.h>
#include <argviz.h>
#include "Config.h"
#include "uMQ.h"

#define DXL_BAUDRATE 1000000
#define DXL_SERIAL dxlSerial
#define DXL_PROTOCOL 1
#define DXL_DIR_PIN 37

// HardwareSerial dxlSerial(0);

uMQ radio;

int servoID[5] = {1, 2, 3, 15, 5};
float servoPos[5] = {0, 0, 0, 0, 0};
float servoVoltage[5] = {0, 0, 0, 0, 0};
int servoTemp[5] = {0, 0, 0, 0, 0};
char buf[32] = {0};

short Position;
short Voltage;
short Temperature;
int servo_id = 0;
int counter_max = 5;
int counter_low = -1;
String send_data;
String data;
bool data_flag = false;

uint8_t counter = 0;
String motorModel;

int find_Pos(int servo_id)
{
  Position = Dynamixel.readPosition(servo_id);

  return Position;
}

int find_Volt(int servo_id)
{
  Voltage = Dynamixel.readVoltage(servo_id);

  return Voltage;
}

int find_Temp(int servo_id)
{
  Temperature = Dynamixel.readTemperature(servo_id);

  return Temperature;
}

void find_Model(int servo_id)
{
  motorModel = Dynamixel.mapMotorModel((Dynamixel.readModel(servo_id)));
}

SCREEN(DX1, {
  find_Model(servoID[counter]);
  //  ROW("Position[]: %d", ((servoPos[counter] * 360) / 1023) - 150)
  ROW("Pos: %03d", (int)servoPos[counter])
  ROW("Voltage[]: %d", (servoVoltage[counter]) / 10)
  ROW("Temperature[]: %d", servoTemp[counter])
  ROW("Model[]: %s", motorModel.c_str())
  CLICK_ROW([](CLICK_STATE state)
            {
        switch(state)
        {
        case CLICK_LEFT:
            counter--;
            break;
        case CLICK_RIGHT:
            counter++;
            break;
        case CLICK_DOWN:
            counter = 0;
            break;
        default:
            break;
        } }, "Servo_id: %3u", counter);
});

SCREEN(nrf, {ROW("data_sended: %s", send_data)})

void setup()
{

  // dxlSerial.begin(DXL_BAUDRATE, SERIAL_8N1, RX, TX);  // Define your RX and TX pins "Feather S3 44-RX 43-TX"
  Serial.begin(115200); // initialize serial communication at 115200 bps, Enable CDC on Boot

  Dynamixel.begin(&Serial2, DXL_BAUDRATE, DXL_DIR_PIN, DXL_PROTOCOL);
  delay(1000);

  radio.init(A2, A3);

  argviz_init(Serial);
  argviz_registerScreen(0, DX1);
  argviz_registerScreen(1, nrf);
  argviz_start();
}

void loop()
{
  static uint32_t timer = micros();
  while (micros() - timer < Ts_us)
    ;
  timer = micros();

  if (counter >= counter_max)
  {
    counter--;
  }
  data = radio.recv("DOBRY", 100);
  data.trim();
  if (data != "")
  {
    if (data == "mnp")
    {
      data_flag = true;
    }
    if (data == "end")
    {
      data_flag = false;
    }
  }
  if (data_flag == true)
  {
    // send_data = "MANIP:" +
    //             String(servoPos[0]) + ";" +
    //             String(servoPos[1]) + ";" +
    //             String(servoPos[2]) + ";" +
    //             String(servoPos[3]) + ";" +
    //             String(servoPos[4]);
    char buf[50] = {};
    sprintf(buf,
      "MANIP:%03d;%03d;%03d;%03d;%03d",
      (int)servoPos[0],
      (int)servoPos[1],
      (int)servoPos[2],
      (int)servoPos[3],
      (int)servoPos[4]);
    send_data = String(buf);

    radio.send(send_data, "SCADA", 100);
  }

  for (int i = 0; i < 4; i++)
  {
    servoPos[i] = (((find_Pos(servoID[i]) * 360.0) / 1023));
  }
}