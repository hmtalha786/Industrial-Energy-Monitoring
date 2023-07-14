#include <ArduinoJson.h>
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

// Data Registers  ===============================================================================================================================================================

#define V_DR_L1 0x0061            // L1 Voltage Data Register
#define V_DR_L2 0x0062            // L2 Voltage Data Register
#define V_DR_L3 0x0063            // L3 Voltage Data Register

// --------------------------------------------------------------------------------

#define C_DR_L1 0x0064            // L1 Current Data Register
#define C_DR_L2 0x0065            // L2 Current Data Register
#define C_DR_L3 0x0066            // L3 Current Data Register

// --------------------------------------------------------------------------------

#define ACP_DR_L1 0x0067           // L1 Active Power Data Register
#define ACP_DR_L2 0x0068           // L2 Active Power Data Register
#define ACP_DR_L3 0x0069           // L3 Active Power Data Register

#define TACP_DR 0x006A             // Total Active Power

// --------------------------------------------------------------------------------

#define REP_DR_L1 0x006B           // L1 Reactive Power Data Register
#define REP_DR_L2 0x006C           // L2 Reactive Power Data Register
#define REP_DR_L3 0x006D           // L3 Reactive Power Data Register

#define TREP_DR 0x006E             // Total Reactive Power

// --------------------------------------------------------------------------------

#define APP_DR_L1 0x006F           // L1 Apparent Power Data Register
#define APP_DR_L2 0x0070           // L2 Apparent Power Data Register
#define APP_DR_L3 0x0071           // L3 Apparent Power Data Register

#define TAPP_DR 0x0072             // Total Apparent Power

// --------------------------------------------------------------------------------

#define PF_DR_L1 0x0073            // L1 Power Factor
#define PF_DR_L2 0x0074            // L2 Power Factor
#define PF_DR_L3 0x0075            // L3 Power Factor

#define TPF_DR 0x0076              // Total Power Factor

// --------------------------------------------------------------------------------

#define F_DR 0x0077                // Frequency Data Register

// --------------------------------------------------------------------------------

#define PV_DR_L12 0x0078           // L1 - L2 Phase Voltage Data Register
#define PV_DR_L23 0x0079           // L2 - L3 Phase Voltage Data Register
#define PV_DR_L31 0x007A           // L3 - L1 Phase Voltage Data Register

// --------------------------------------------------------------------------------

#define TACE_DR 0x6002             // Total Active Energy

// Voltage ==============================================================================================================================================================

float V_L1;     // L1 Voltage
float V_L2;     // L2 Voltage
float V_L3;     // L3 Voltage

// Current ==============================================================================================================================================================

float C_L1;     // L1 Current
float C_L2;     // L2 Current
float C_L3;     // L3 Current

// Active Power ==============================================================================================================================================================

float ACP_L1;    // L1 Active Power
float ACP_L2;    // L2 Active Power
float ACP_L3;    // L3 Active Power

// Apparent Power ==============================================================================================================================================================

float APP_L1;    // L1 Apparent Power
float APP_L2;    // L2 Apparent Power
float APP_L3;    // L3 Apparent Power

// Reactive Power ==============================================================================================================================================================

float REP_L1;    // L1 Reactive Power
float REP_L2;    // L2 Reactive Power
float REP_L3;    // L3 Reactive Power

// Power Factor ==============================================================================================================================================================

float PF_L1;    // L1 Power Factor
float PF_L2;    // L2 Power Factor
float PF_L3;    // L3 Power Factor

// Line to Line Voltages ==============================================================================================================================================================

float PV_12;    // L1-L2 Voltages
float PV_23;    // L2-L3 Voltages
float PV_31;    // L3-L1 Voltages

// 3 Phase Aggregated Parameters ======================================================================================================================================================

float OSF;      // Overall System Frequency
float TTP;      // Total Active Power
float TAP;      // Total Apparent Power
float TRP;      // Total Reactive Power
float TPF;      // Total Power Factor
float TAE;      // Total Active Energy
float CSE;      // Calculated System Energy
float ASE;      // Aggregated System Energy

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

float getData_fourbytes( uint16_t dataRegister, float correction_factor ) {
  float result;
  uint8_t dataRegisterNode = node.readHoldingRegisters(dataRegister, 4);
  if ( dataRegisterNode == node.ku8MBSuccess )                       // [readHoldingRegisters(Address of the register, Number of registers want to be read from this address)]
  {
    byte byte0 = node.getResponseBuffer(0x00);
    byte byte1 = node.getResponseBuffer(0x01);
    byte byte2 = node.getResponseBuffer(0x02);
    byte byte3 = node.getResponseBuffer(0x03);

    unsigned long joinedData = 0;
    joinedData = (byte0 << 32) |(byte1 << 16) | (byte2 << 8) | (byte3);
    
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
  if ( millis() >= timer ) { 
    Serial.println(" Sending Data Packet ");  
    timer = millis() + 10000UL;  
    Json_Packet_Sender(); 
  }
  delay(1000);
  esp_task_wdt_reset();
}

// ==================================================================================================================================================================================

void Json_Packet_Sender() {

  Read_EA_Param();  // Read Energy Analyzer Parameters

  StaticJsonBuffer<900> JSON_Packet;
  JsonObject& JSON_Entry = JSON_Packet.createObject();

  JSON_Entry["STS"] = millis();
  JSON_Entry["OSF"] = OSF;
  JSON_Entry["TAE"] = TAE;
  
  JSON_Entry["V1"] = V_L1;
  JSON_Entry["V2"] = V_L2;
  JSON_Entry["V3"] = V_L3;
  
  JSON_Entry["C1"] = C_L1;
  JSON_Entry["C2"] = C_L2;
  JSON_Entry["C3"] = C_L3;
  
  JSON_Entry["TP1"] = ACP_L1;
  JSON_Entry["TP2"] = ACP_L2;
  JSON_Entry["TP3"] = ACP_L3;
  
  JSON_Entry["AP1"] = APP_L1;
  JSON_Entry["AP2"] = APP_L2;
  JSON_Entry["AP3"] = APP_L3;
  
  JSON_Entry["RP1"] = REP_L1;
  JSON_Entry["RP2"] = REP_L2;
  JSON_Entry["RP3"] = REP_L3;
  
  JSON_Entry["PF1"] = PF_L1;
  JSON_Entry["PF2"] = PF_L2;
  JSON_Entry["PF3"] = PF_L3;

  JSON_Entry["TTP"] = TTP;
  JSON_Entry["TAP"] = TAP;
  JSON_Entry["TRP"] = TRP;
  JSON_Entry["TPF"] = TPF;

  JSON_Entry["L12"] = PV_12;
  JSON_Entry["L23"] = PV_23;
  JSON_Entry["L31"] = PV_31;

  char JSONmessageBuffer[900];
  JSON_Entry.prettyPrintTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.print(JSONmessageBuffer);
}

// ==================================================================================================================================================================================

void Read_EA_Param(){
  
  Serial.println(" ________________________________________________________ Voltage ________________________________________________________ ");
    
  V_L1 = getData(V_DR_L1, 0.1);   Serial.print("L1 Voltage : ");    Serial.println(V_L1);    delay(100);
  V_L2 = getData(V_DR_L2, 0.1);   Serial.print("L2 Voltage : ");    Serial.println(V_L2);    delay(100);
  V_L3 = getData(V_DR_L3, 0.1);   Serial.print("L3 Voltage : ");    Serial.println(V_L3);    delay(100);

  Serial.println(" ________________________________________________________ Current ________________________________________________________ ");
  
  C_L1 = getData(C_DR_L1, 0.01*30);      Serial.print("L1 Current : ");    Serial.println(C_L1);    delay(100);
  C_L2 = getData(C_DR_L2, 0.01*30);      Serial.print("L2 Current : ");    Serial.println(C_L2);    delay(100);
  C_L3 = getData(C_DR_L3, 0.01*30);      Serial.print("L3 Current : ");    Serial.println(C_L3);    delay(100);

  Serial.println(" ________________________________________________________ Active Power ________________________________________________________ ");
  
  ACP_L1 = getData(ACP_DR_L1, 0.001);   Serial.print("L1 True Power : ");    Serial.println(ACP_L1);    delay(100);
  ACP_L2 = getData(ACP_DR_L2, 0.001);   Serial.print("L2 True Power : ");    Serial.println(ACP_L2);    delay(100);
  ACP_L3 = getData(ACP_DR_L3, 0.001);   Serial.print("L3 True Power : ");    Serial.println(ACP_L3);    delay(100);

  Serial.println(" ________________________________________________________ Apparent Power ________________________________________________________ ");
  
  APP_L1 = getData(APP_DR_L1, 0.001);   Serial.print("L1 Apparent Power : ");    Serial.println(APP_L1);    delay(100);
  APP_L2 = getData(APP_DR_L2, 0.001);   Serial.print("L2 Apparent Power : ");    Serial.println(APP_L2);    delay(100);
  APP_L3 = getData(APP_DR_L3, 0.001);   Serial.print("L3 Apparent Power : ");    Serial.println(APP_L3);    delay(100);

  Serial.println(" ________________________________________________________ Reactive Power ________________________________________________________ ");
  
  REP_L1 = getData(REP_DR_L1, 0.001);   Serial.print("L1 Reactive Power : ");    Serial.println(REP_L1);    delay(100);
  REP_L2 = getData(REP_DR_L2, 0.001);   Serial.print("L2 Reactive Power : ");    Serial.println(REP_L2);    delay(100);
  REP_L3 = getData(REP_DR_L3, 0.001);   Serial.print("L3 Reactive Power : ");    Serial.println(REP_L3);    delay(100);

  Serial.println(" ________________________________________________________ Power Factor ________________________________________________________ ");
  
  PF_L1 = getData(PF_DR_L1, 0.001);   Serial.print("L1 Power Factor : ");    Serial.println(PF_L1);    delay(100);
  PF_L2 = getData(PF_DR_L2, 0.001);   Serial.print("L2 Power Factor : ");    Serial.println(PF_L2);    delay(100);
  PF_L3 = getData(PF_DR_L3, 0.001);   Serial.print("L3 Power Factor : ");    Serial.println(PF_L3);    delay(100);
 
  Serial.println(" ________________________________________________________ Frequency ________________________________________________________ ");

  OSF = getData(F_DR, 0.01);         Serial.print("Total Frequency : ");       Serial.println(OSF);     delay(100);

  Serial.println(" ________________________________________________________ Line to Line / Phase Voltages  ________________________________________________________ ");
  
  PV_12 = getData(PV_DR_L12, 0.1);   Serial.print("L1-L2 Phase Voltage : ");   Serial.println(PV_12);   delay(100);
  PV_23 = getData(PV_DR_L23, 0.1);   Serial.print("L2-L3 Phase Voltage : ");   Serial.println(PV_23);   delay(100);
  PV_31 = getData(PV_DR_L31, 0.1);   Serial.print("L3-L1 Phase Voltage : ");   Serial.println(PV_31);   delay(100);

  Serial.println(" ________________________________________________________ Total 3 Phase Aggregated Values ________________________________________________________ ");
  
  TTP = getData(TACP_DR, 0.001);   Serial.print("Total True Power : ");        Serial.println(TTP);    delay(100);
  TRP = getData(TREP_DR, 0.001);   Serial.print("Total Reactive Power : ");    Serial.println(TRP);    delay(100);
  TAP = getData(TAPP_DR, 0.001);   Serial.print("Total Apparent Power : ");    Serial.println(TAP);    delay(100);
  TPF = getData(TPF_DR, 0.001);    Serial.print("Total Power Factor : ");      Serial.println(TPF);    delay(100);

  TAE = getData_fourbytes(TACE_DR, 0.1);   Serial.print("Total Active Energy : ");     Serial.println(TAE);    delay(100);

}
