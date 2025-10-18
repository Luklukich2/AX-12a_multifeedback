#include <Arduino.h>
#include <Dynamixel.h>
#include <argviz.h>
#include "Config.h"

#define DXL_BAUDRATE    1000000
#define DXL_SERIAL      dxlSerial
#define DXL_PROTOCOL    1
#define DXL_DIR_PIN     37

// HardwareSerial dxlSerial(0);    

int servoID[5] = {1, 2, 3, 15, 5};

short Position;
short Voltage;
short Temperature;
int servo_id = 1;
int counter_max = 5;
int counter_low = -1;

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
         ROW("Position[]: %d", ((find_Pos(servoID[counter]) * 360) / 1023))
         ROW("Voltage[]: %d", (find_Volt(servoID[counter])) / 10)
         ROW("Temperature[]: %d", find_Temp(servoID[counter]))
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

void setup() {

  // dxlSerial.begin(DXL_BAUDRATE, SERIAL_8N1, RX, TX);  // Define your RX and TX pins "Feather S3 44-RX 43-TX"
  Serial.begin(115200);                               // initialize serial communication at 115200 bps, Enable CDC on Boot

  Dynamixel.begin(&Serial2, DXL_BAUDRATE, DXL_DIR_PIN, DXL_PROTOCOL);
  delay(1000);

  argviz_init(Serial);
  argviz_registerScreen(0, DX1);
  argviz_start();
}

void loop() {
  static uint32_t timer = micros();
  while(micros() - timer < Ts_us)
  ;
  timer = micros();

  // if (counter >= counter_max)
  // {
  //   counter = 0;
  // }
  // if (counter >= counter_low)
  // {
  //   counter = 0;
  // }
  // Position = Dynamixel.readPosition(DXL_ID1);
  // Voltage = Dynamixel.readVoltage(DXL_ID1);
  // Temperature = Dynamixel.readTemperature(DXL_ID1);
  // motorModel = Dynamixel.mapMotorModel((Dynamixel.readModel(DXL_ID1)));

  // Serial.print(motorModel); 
  // Serial.print(" -> Position: ");Serial.print((Position*360) / 1023);
  // Serial.print("° Temperature: ");Serial.print(Temperature);
  // Serial.print("°C Voltage: ");Serial.print(Voltage/10);Serial.println("V.");

  // delay(100);
}