#include <Arduino.h>
#include "config.h"
#include <HardwareSerial.h>
#include <SPI.h>
#include <ModbusMaster.h>

ModbusMaster node;

// ====================================================================================================================================================================================

// Data Registers
uint16_t Vavg_Reg = 3035;

// ====================================================================================================================================================================================

// Holding Values
float Vavg;

// ====================================================================================================================================================================================

void setup() {

  pinMode(RS485_EN_PIN, OUTPUT);
  digitalWrite(RS485_EN_PIN, HIGH);

  pinMode(RS485_SE_PIN, OUTPUT);
  digitalWrite(RS485_SE_PIN, HIGH);

  pinMode(PIN_5V_EN, OUTPUT);
  digitalWrite(PIN_5V_EN, HIGH);

  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  delay(5);
  node.begin(2, Serial1);
  Serial.println("test");
}

// ====================================================================================================================================================================================

float HexTofloat(uint32_t x) {
  return (*(float*)&x);
}

// ====================================================================================================================================================================================

float Read_Data_Reg_Val(uint16_t data_reg) {

  uint8_t result = node.readHoldingRegisters(data_reg, 2);

  uint16_t buff[2];

  if (result == node.ku8MBSuccess) {

    for (uint8_t j = 0; j < 2; j++) {
      buff[j] = node.getResponseBuffer(j);
    }

    uint32_t value = (buff[0] << 16) + (buff[1]);
    return HexTofloat(value);

  } else {

    Serial.print("connection field");
    return 0.0;

  }
}

// ====================================================================================================================================================================================

void loop() {

  // Average Voltage ---------------------------------------

  float Vavg = Read_Data_Reg_Val(Vavg_Reg);
  Serial.print("Average Voltage ⚡ : ");
  Serial.println(Vavg);

  // -------------------------------------------------------

  delay(1000);
}
