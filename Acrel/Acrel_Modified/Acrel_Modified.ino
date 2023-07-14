#include <WiFi.h>
#include <time.h>
#include <ESP32Ping.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
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

const char* ssid = "MBL";
const char* pass = "pc@54321";

String cstr = "https://api.datacake.co/integrations/api/312b92dc-0a05-4415-9ad2-e74a4b53040a/";

// =====================================================================================================================================================================================

// a string to hold NTP server to request epoch time
const char* ntpServer = "pool.ntp.org";

// Variable to hold current epoch timestamp
unsigned long Epoch_Time;

// Get_Epoch_Time() Function that gets current epoch time
unsigned long Get_Epoch_Time() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return (0);
  }
  time(&now);
  return now;
}

// =====================================================================================================================================================================================

#define WDT_TIMEOUT 300      // 5 Minute

// =====================================================================================================================================================================================

unsigned long Time_Checker = 0;
unsigned long timer = 60000;      // JSON Packet Sending time Counter 

// Data Registers  ===============================================================================================================================================================

#define TACE_DR 0x6002            // Total Active Energy

// --------------------------------------------------------------------------------

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
float CSP;      // Total Calculated Power
float ASP;      // Total Accumulated Power
unsigned long TAE;      // Total Active Energy

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
  if ( result == result ) {
    return result;
  } else {
    result = 0.0;
    return result;
  }
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
  if ( result == result ) {
    return result;
  } else {
    result = 0.0;
    return result;
  }
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

  Serial.print("WiFi_Username : "); Serial.println(ssid);
  Serial.print("WiFi_Password : "); Serial.println(pass);
  Serial.print("Conection Str : "); Serial.println(cstr);

    /* ----- Connect to WiFi through saved credentials ----- */
  Connect_To_WiFi(ssid, pass);

  // Configure NTP Server
  configTime(0, 0, ntpServer);

}

// ====================================================================================================================================================================================

void loop() {
  if ( millis() >= timer ) { 
    Serial.println(" Sending Data Packet ");  
    timer = millis() + 300000UL;  
    Json_Packet_Sender(); 
  }
  Read_EA_Param();  // Read Energy Analyzer Parameters
  delay(1000);
  esp_task_wdt_reset();
}

// ==================================================================================================================================================================================

void Json_Packet_Sender() {

  Epoch_Time = Get_Epoch_Time();    
  Serial.println( "Epoch Unix Timestamp : " + String(Epoch_Time) );    
  delay(100);

  StaticJsonBuffer<900> JSON_Packet;
  JsonObject& JSON_Entry = JSON_Packet.createObject();

  JSON_Entry["device"] = "PROCHECK-TESTING";
  JSON_Entry["UTS"] = Epoch_Time;
  JSON_Entry["STS"] = millis();
  JSON_Entry["OSF"] = OSF;
  JSON_Entry["KWH"] = TAE;
  JSON_Entry["CSP"] = CSP;
  JSON_Entry["ASP"] = ASP;
  
  JSON_Entry["V1"] = V_L1;
  JSON_Entry["V2"] = V_L2;
  JSON_Entry["V3"] = V_L3;
  
  JSON_Entry["C1"] = C_L1;
  JSON_Entry["C2"] = C_L2;
  JSON_Entry["C3"] = C_L3;
  
  JSON_Entry["TP1"] = ACP_L1;
  JSON_Entry["TP2"] = ACP_L2;
  JSON_Entry["TP3"] = ACP_L3;
  
//  JSON_Entry["AP1"] = APP_L1;
//  JSON_Entry["AP2"] = APP_L2;
//  JSON_Entry["AP3"] = APP_L3;
//  
//  JSON_Entry["RP1"] = REP_L1;
//  JSON_Entry["RP2"] = REP_L2;
//  JSON_Entry["RP3"] = REP_L3;
//  
//  JSON_Entry["PF1"] = PF_L1;
//  JSON_Entry["PF2"] = PF_L2;
//  JSON_Entry["PF3"] = PF_L3;

  JSON_Entry["TTP"] = TTP;
  JSON_Entry["TAP"] = TAP;
  JSON_Entry["TRP"] = TRP;
  JSON_Entry["TPF"] = TPF;

//  JSON_Entry["L12"] = PV_12;
//  JSON_Entry["L23"] = PV_23;
//  JSON_Entry["L31"] = PV_31;

  char JSONmessageBuffer[900];
  JSON_Entry.prettyPrintTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.print(JSONmessageBuffer);

  if (WiFi.status() == WL_CONNECTED) {                              // Check WiFi connection status

    HTTPClient http;
    http.begin(cstr);                                               // Specify destination for HTTP request
    http.addHeader("Content-Type", "application/json");             // Specify content-type header
    int httpResponseCode = http.POST(JSONmessageBuffer);            // Send the actual POST request

    if (httpResponseCode > 0) {
      String response = http.getString();                           // Get the response to the request
      Serial.println(httpResponseCode);                             // Print return code
      Serial.println(response);                                     // Print request answer
    } else {
      Serial.print("Error on sending POST: ");
      Serial.println(httpResponseCode);
      Serial.println("Re-Connecting to WiFi");
      WiFi.disconnect();
      Connect_To_WiFi(ssid, pass);
    }

    http.end();  // Free resources

  } else {

    Serial.println("Error in WiFi connection");
    Serial.println("Re-Connecting to WiFi");
    WiFi.disconnect();
    Connect_To_WiFi(ssid, pass);

  }
}

// ==================================================================================================================================================================================

void Read_EA_Param(){
  
  V_L1 = getData(V_DR_L1, 0.1);   
  Serial.println( "L1 Voltage : " + String(V_L1) );    
  delay(100);
  
  V_L2 = getData(V_DR_L2, 0.1);   
  Serial.println( "L2 Voltage : " + String(V_L2) );    
  delay(100);
  
  V_L3 = getData(V_DR_L3, 0.1);   
  Serial.println( "L3 Voltage : " + String(V_L3) );    
  delay(100);

  C_L1 = getData(C_DR_L1, 0.01*30);      
  Serial.println( "L1 Current : " + String(C_L1) );    
  delay(100);
  
  C_L2 = getData(C_DR_L2, 0.01*30);      
  Serial.println( "L2 Current : " + String(C_L2) );    
  delay(100);
  
  C_L3 = getData(C_DR_L3, 0.01*30);      
  Serial.println( "L3 Current : " + String(C_L3) );    
  delay(100);

  ACP_L1 = getData(ACP_DR_L1, 0.001);   
  Serial.println( "L1 Active Power : " + String(ACP_L1) );    
  delay(100);
  
  ACP_L2 = getData(ACP_DR_L2, 0.001);   
  Serial.println( "L2 Active Power : " + String(ACP_L2) );    
  delay(100);
  
  ACP_L3 = getData(ACP_DR_L3, 0.001);   
  Serial.println( "L3 Active Power : " + String(ACP_L3) );    
  delay(100);

  APP_L1 = getData(APP_DR_L1, 0.001);   
  Serial.println( "L1 Apparent Power : " + String(APP_L1) );    
  delay(100);
  
  APP_L2 = getData(APP_DR_L2, 0.001);   
  Serial.println( "L2 Apparent Power : " + String(APP_L2) );    
  delay(100);
  
  APP_L3 = getData(APP_DR_L3, 0.001);   
  Serial.println( "L3 Apparent Power : " + String(APP_L3) );    
  delay(100);

  REP_L1 = getData(REP_DR_L1, 0.001);   
  Serial.println( "L1 Reactive Power : " + String(REP_L1) );    
  delay(100);
  
  REP_L2 = getData(REP_DR_L2, 0.001);   
  Serial.println( "L2 Reactive Power : " + String(REP_L2) );    
  delay(100);
  
  REP_L3 = getData(REP_DR_L3, 0.001);   
  Serial.println( "L3 Reactive Power : " + String(REP_L3) );    
  delay(100);

  PF_L1 = getData(PF_DR_L1, 0.001);   
  Serial.println( "L1 Power Factor : " + String(PF_L1) );    
  delay(100);
  
  PF_L2 = getData(PF_DR_L2, 0.001);   
  Serial.println( "L2 Power Factor : " + String(PF_L2) );    
  delay(100);
  
  PF_L3 = getData(PF_DR_L3, 0.001);   
  Serial.println( "L3 Power Factor : " + String(PF_L3) );    
  delay(100);

  OSF = getData(F_DR, 0.01);         
  Serial.println( "Total Frequency : " + String(OSF) );     
  delay(100);

  PV_12 = getData(PV_DR_L12, 0.1);   
  Serial.println( "L1-L2 Phase Voltage : " + String(PV_12) );   
  delay(100);
  
  PV_23 = getData(PV_DR_L23, 0.1);   
  Serial.println( "L2-L3 Phase Voltage : " + String(PV_23) );   
  delay(100);
  
  PV_31 = getData(PV_DR_L31, 0.1);   
  Serial.println( "L3-L1 Phase Voltage : " + String(PV_31) );   
  delay(100);

  TTP = getData(TACP_DR, 0.001);   
  Serial.println( "Total True Power : " + String(TTP));   
  delay(100);
  
  TRP = getData(TREP_DR, 0.001);   
  Serial.println( "Total Reactive Power : " + String(TRP) );    
  delay(100);
  
  TAP = getData(TAPP_DR, 0.001);   
  Serial.println( "Total Apparent Power : " + String(TAP) );    
  delay(100);
  
  TPF = getData(TPF_DR, 0.001);    
  Serial.println( "Total Power Factor : " + String(TPF) );    
  delay(100);

  CSP = ( TTP / 60 );    
  Serial.println( "3-Phase Calculated Active Power : " + String(CSP) );    
  delay(100);

  ASP = ASP + CSP;     
  Serial.println( "3-Phase Aggregated Active Power : " + String(ASP) );    
  delay(100);

  TAE = getData_fourbytes(TACE_DR, 0.0001);   
  Serial.println( "Total Active Energy : " + String(TAE) );    
  delay(100);

}

// ====================================================================================================================================================================================

void Connect_To_WiFi(const char * ssid, const char * pwd) {

  Time_Checker = millis();

  Serial.println();
  Serial.println("Connecting to WiFi network: " + String(ssid));
  Serial.println();

  WiFi.begin(ssid, pwd);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting to " + String(ssid));
    delay(500);
    /* Restart ESP32 if not connected for 1 minutes */
    if ( millis() - Time_Checker > 60000 ) {
      ESP.restart();
    }
  }

  Serial.println();

  /* Printing Network Credentials */
  Serial.println("WiFi connected!");
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("Subnet Mask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("Gateway IP: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("Primary DNS: ");
  Serial.println(WiFi.dnsIP(0));
  Serial.print("Secondary DNS: ");
  Serial.println(WiFi.dnsIP(1));

  /* Making Ping to Google to Check Internet Connection */
  bool Ping_Success = Ping.ping("www.google.com", 3);

  if (!Ping_Success) {
    Serial.println("Failed to Ping www.google.com");
    WiFi.disconnect();
    Connect_To_WiFi(ssid, pass);
  } else {
    Serial.println();
    Serial.println("Ping successful to www.google.com");
    Serial.println();
  }
  
}
