#include <ModbusMaster.h>
#include <esp_task_wdt.h>

// =====================================================================================================================================================================================

#define PIN_5V_EN 16

#define RS485_EN_PIN 17 // 17 /RE
#define RS485_TX_PIN 22 // 21
#define RS485_RX_PIN 21 // 22
#define RS485_SE_PIN 19 // 22 /SHDN

// =====================================================================================================================================================================================

ModbusMaster node;

// =====================================================================================================================================================================================

#define WDT_TIMEOUT 300      // 5 Minute

// =====================================================================================================================================================================================

unsigned long timer = 60000;      // JSON Packet Sending time Counter 

// EA Data Registers  ===============================================================================================================================================================

#define VDR_L1 0x0061           // L1 Voltage Data Register

#define ADR_L1 0x0064           // L1 Current Data Register

// L1 Phase Parameters ==============================================================================================================================================================

float V_L1;     // L1 Voltage
float A_L1;     // L1 Current

// ====================================================================================================================================================================================

float getData( uint16_t dataRegister, float correction_factor ) {
  float result;
  uint8_t dataRegisterNode = node.readHoldingRegisters(dataRegister, 2);
  if ( dataRegisterNode == node.ku8MBSuccess )                       // [readHoldingRegisters(Address of the register, Number of registers want to be read from this address)]
  {
    uint16_t data_LSB = node.getResponseBuffer(0x00);
    uint16_t data_MSB = node.getResponseBuffer(0x01);

    uint16_t Data[2] = {data_LSB, data_MSB};
    uint16_t joinedData;
    
    memcpy(&joinedData, Data, 4);
    result = joinedData * correction_factor;
  }
  return result;
}

// ====================================================================================================================================================================================

void setup() {

  Serial.begin(9600);
  delay(1000);

  Serial.println("Configurating Watch Dog Timer ....");
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  pinMode(PIN_5V_EN, OUTPUT);
  pinMode(RS485_EN_PIN, OUTPUT);
  pinMode(RS485_SE_PIN, OUTPUT);

  digitalWrite(PIN_5V_EN, HIGH);
  digitalWrite(RS485_EN_PIN, HIGH);
  digitalWrite(RS485_SE_PIN, HIGH);

  Serial1.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  delay(50);
  node.begin(1, Serial1);
  
}

// ====================================================================================================================================================================================

void loop() { 
  V_L1 = getData(VDR_L1, 0.1);   
  Serial.print("L1 Voltage : ");    
  Serial.println(V_L1);
  delay(1000);
  A_L1 = getData(ADR_L1, 0.01*30);   
  Serial.print("L1 Current : ");    
  Serial.println(A_L1); 
  delay(1000);
  esp_task_wdt_reset();
}
