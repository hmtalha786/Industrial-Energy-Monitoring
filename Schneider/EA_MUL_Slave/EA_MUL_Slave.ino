#include <WiFi.h>
#include <time.h>
#include <EEPROM.h>
#include <ESP32Ping.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <ModbusMaster.h>
#include <esp_task_wdt.h>

// =====================================================================================================================================================================================

#define MAX485_DE 19    // Driver Enable
#define MAX485_RE 18    // Receiver Enable

// =====================================================================================================================================================================================

ModbusMaster node;

int SSC = 1;             // Slave Switch Check
String device;           // Slave Energy Analyzer

// =====================================================================================================================================================================================

#define WDT_TIMEOUT 300      // 5 Minute

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
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}

// =====================================================================================================================================================================================

unsigned long Time_Checker = 0;
unsigned long timer = 60000;      // JSON Packet Sending time Counter 

// =====================================================================================================================================================================================

struct My_Object {
  char ssid[25];
  char pass[25];
  char cstr[150];
};

// =====================================================================================================================================================================================

My_Object customVarr;

// =====================================================================================================================================================================================

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// =====================================================================================================================================================================================

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

      if (value.length() > 0) {

        /* ----- Make object from struct to carry credentials ----- */
        My_Object Credentials;
        char WiFi_SSID[25] = "";
        char WiFi_PASS[25] = "";
        char Conec_STR[150] = "";

        /* ----- Extract Connection String ----- */
        if ( value[0] == '@' ) {

          Serial.print("Entered value: ");
          for (int i = 0; i < value.length(); i++) {
            Serial.print(value[i]);
          }

          for (int i = 1; i < value.length(); i++) {
            Conec_STR[i - 1] = value[i];
          }

          Serial.print("Connection String : ");
          Serial.println(Conec_STR);
          memcpy(Credentials.cstr, Conec_STR, sizeof(Credentials.cstr));

        } else {
          int x;
          Serial.print("Entered value: ");
          for (int i = 0; i < value.length(); i++) {
            if ( value[i] == ',' ) {
              x = i;
            }
            Serial.print(value[i]);
          }
          Serial.println("");
          Serial.print("Comma is at index : ");
          Serial.println(x);

          /* ----- Extract WiFi SSID ----- */

          for (int i = 0; i < x; i++) {
            WiFi_SSID[i] = value[i];
          }
          Serial.print("WiFi_Username : ");
          Serial.println(WiFi_SSID);
          memcpy(Credentials.ssid, WiFi_SSID, sizeof(Credentials.ssid));

          /* ----- Extract WiFi Password ----- */

          for (int i = x + 1; i < value.length(); i++) {
            WiFi_PASS[i - (x + 1)] = value[i];
          }
          Serial.print("WiFi_Password : ");
          Serial.println(WiFi_PASS);
          memcpy(Credentials.pass, WiFi_PASS, sizeof(Credentials.pass));

          /* ----- Keep the Connection String Same ----- */

          EEPROM.get(0, customVarr);
          Serial.println(customVarr.cstr);
          memcpy(Credentials.cstr, customVarr.cstr, sizeof(Credentials.cstr));
        }

        /* ----- Store WiFi Credentials to EEPROM Address 0 ----- */
        EEPROM.put(0, Credentials);
        EEPROM.commit();
      }
    }

};

// EA Data Registers  ===============================================================================================================================================================

#define VDR_L1 3036           // L1 Voltage Data Register
#define VDR_L2 2832           // L2 Voltage Data Register
#define VDR_L3 2832           // L3 Voltage Data Register

#define ADR_L1 3000           // L1 Current Data Register
#define ADR_L2 3002           // L2 Current Data Register
#define ADR_L3 3004           // L3 Current Data Register

#define TPDR_L1 3054          // L1 True Power Data Register
#define TPDR_L2 3056          // L2 True Power Data Register
#define TPDR_L3 3058          // L3 True Power Data Register

#define APDR_L1 3070          // L1 Apparent Power Data Register
#define APDR_L2 3072          // L2 Apparent Power Data Register
#define APDR_L3 3074          // L3 Apparent Power Data Register

#define PVDR_L12 3020         // L1 - L2 Phase Voltage Data Register
#define PVDR_L23 3022         // L2 - L3 Phase Voltage Data Register
#define PVDR_L31 3024         // L3 - L1 Phase Voltage Data Register

#define TTPDR 3060            // Total True Power Data Register
#define TAPDR 3076            // Total Apparent Power Data Register

#define TFDR 3110             // Total Frequency Data Register
#define KWHDR 2700            // Kilo Watt Hour Data Register

// L1 Phase Parameters ==============================================================================================================================================================

float V_L1;     // L1 Voltage
float A_L1;     // L1 Current
float TP_L1;    // L1 True Power
float AP_L1;    // L1 Apparent Power
float RP_L1;    // L1 Reactive Power
float PF_L1;    // L1 Power Factor

// L2 Phase Parameters ==============================================================================================================================================================

float V_L2;     // L2 Voltage
float A_L2;     // L2 Current
float TP_L2;    // L2 True Power
float AP_L2;    // L2 Apparent Power
float RP_L2;    // L2 Reactive Power
float PF_L2;    // L2 Power Factor

// L3 Phase Parameters ===============================================================================================================================================================

float V_L3;     // L3 Voltage
float A_L3;     // L3 Current
float TP_L3;    // L3 True Power
float AP_L3;    // L3 Apparent Power
float RP_L3;    // L3 Reactive Power
float PF_L3;    // L3 Power Factor

// 3 Phase Aggregated Parameters ======================================================================================================================================================

float TTP;      // Total True Power
float TAP;      // Total Apparent Power
float TRP;      // Total Reactive Power
float TPF;      // Total Power Factor
float TF;       // Total Frequency
float KWH;      // Total Killo Watt Hours
float ASP;      // Aggregated System Power

// Line to Line Voltages ==============================================================================================================================================================

float LV_12;    // L1-L2 Voltages
float LV_23;    // L2-L3 Voltages
float LV_31;    // L3-L1 Voltages

// ====================================================================================================================================================================================

void preTransmission(){  digitalWrite(MAX485_RE, 1);  digitalWrite(MAX485_DE, 1);  }

void postTransmission(){  digitalWrite(MAX485_RE, 0);  digitalWrite(MAX485_DE, 0); }

// ====================================================================================================================================================================================

float getData( uint16_t dataRegister, float correction_factor ) {

  float result;
  uint8_t dataRegisterNode = node.readHoldingRegisters( ( dataRegister - 1 ), 2);
  uint16_t buff[2];

  if ( dataRegisterNode == node.ku8MBSuccess ) {

    for (uint8_t j = 0; j < 2; j++) {
      buff[j] = node.getResponseBuffer(j);
    }

    uint32_t value = ( buff[0] << 16 ) + ( buff[1] );
    result = (*(float*)&value);
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
  EEPROM.begin(500);
  delay(1000);

  Serial.println("Configurating Watch Dog Timer ....");
  esp_task_wdt_init(WDT_TIMEOUT, true);
  esp_task_wdt_add(NULL);

  pinMode(MAX485_RE, OUTPUT);
  pinMode(MAX485_DE, OUTPUT); 
  
  digitalWrite(MAX485_RE, 0);
  digitalWrite(MAX485_DE, 0);

  Serial1.begin(9600, SERIAL_8N1, 16, 17);  // ( Baud rate = 9600, Configuration = [8 Data bits,No parity,1 Stop bit], Rx pin, Tx pin)

  //  node.begin(2, Serial1);                   // Modbus slave ID 2

  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  /* ----- Retrieve WiFi Credentials from EEPROM ----- */
  EEPROM.get(0, customVarr);
  Serial.print("WiFi_Username : "); Serial.println(customVarr.ssid);
  Serial.print("WiFi_Password : "); Serial.println(customVarr.pass);
  Serial.print("Conection Str : "); Serial.println(customVarr.cstr);

  /* ----- Turn ON the bluetooth ----- */
  if (digitalRead(21) == HIGH) {
    BLEDevice::init("Cotbus WiFi Gateway");
    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService(SERVICE_UUID);
    BLECharacteristic *pCharacteristic = pService->createCharacteristic( CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE );
    pCharacteristic->setCallbacks(new MyCallbacks());
    pCharacteristic->setValue("Hello World");
    pService->start();
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
  }

  /* ----- Connect to WiFi through saved credentials ----- */
  Connect_To_WiFi(customVarr.ssid, customVarr.pass);

  // Configure NTP Server
  configTime(0, 0, ntpServer);

}

// ====================================================================================================================================================================================

void loop() {

  if ( millis() >= timer ) { 
    Serial.println(" Sending Data Packet ");  
    timer = millis() + 60000UL; 
    SSC = (SSC + 1) % 2;
    delay(500);
    if ( SSC == 1 ) { node.begin(1, Serial1); device = "D1"; } else { node.begin(2, Serial1); device = "D2"; }
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

  JSON_Entry["device"] = "Cotbus-EA-2";

  JSON_Entry["UTS"] = Epoch_Time;
  JSON_Entry["EAD"] = device;
  
  JSON_Entry["OSF"] = TF;
  JSON_Entry["KWH"] = KWH;
  JSON_Entry["ASP"] = ASP;
  
  JSON_Entry["V1"] = V_L1;
  JSON_Entry["V2"] = V_L2;
  JSON_Entry["V3"] = V_L3;
  
  JSON_Entry["C1"] = A_L1;
  JSON_Entry["C2"] = A_L2;
  JSON_Entry["C3"] = A_L3;
  
  JSON_Entry["TP1"] = TP_L1;
  JSON_Entry["TP2"] = TP_L2;
  JSON_Entry["TP3"] = TP_L3;
  
  JSON_Entry["AP1"] = AP_L1;
  JSON_Entry["AP2"] = AP_L2;
  JSON_Entry["AP3"] = AP_L3;
  
  JSON_Entry["RP1"] = RP_L1;
  JSON_Entry["RP2"] = RP_L2;
  JSON_Entry["RP3"] = RP_L3;
  
  JSON_Entry["PF1"] = PF_L1;
  JSON_Entry["PF2"] = PF_L2;
  JSON_Entry["PF3"] = PF_L3;

  JSON_Entry["TTP"] = TTP;
  JSON_Entry["TAP"] = TAP;
  JSON_Entry["TRP"] = TRP;
  JSON_Entry["TPF"] = TPF;

  JSON_Entry["L12"] = LV_12;
  JSON_Entry["L23"] = LV_23;
  JSON_Entry["L31"] = LV_31;

  char JSONmessageBuffer[900];
  JSON_Entry.prettyPrintTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.print(JSONmessageBuffer);

  if (WiFi.status() == WL_CONNECTED) {                              // Check WiFi connection status 
    
    HTTPClient http;
    http.begin(customVarr.cstr);                                    // Specify destination for HTTP request
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
      Connect_To_WiFi(customVarr.ssid, customVarr.pass);
      Serial.println("Re-Sending JSON Data Packet");
      Json_Packet_Sender();
    }
    
    http.end();  //Free resources
    
  } else {
    
    Serial.println("Error in WiFi connection");
    Serial.println("Re-Connecting to WiFi");
    Connect_To_WiFi(customVarr.ssid, customVarr.pass);
    Serial.println("Re-Sending JSON Data Packet");
    Json_Packet_Sender();
    
  }
}

// ==================================================================================================================================================================================

void Read_EA_Param(){
  
  Serial.println(" ________________________________________________________ Voltage ________________________________________________________ ");
    
  V_L1 = getData(VDR_L1, 0.1);   Serial.print("L1 Voltage : ");    Serial.println(V_L1);    delay(10);
  V_L2 = getData(VDR_L2, 0.1);   Serial.print("L2 Voltage : ");    Serial.println(V_L2);    delay(10);
  V_L3 = getData(VDR_L3, 0.1);   Serial.print("L3 Voltage : ");    Serial.println(V_L3);    delay(10);

  Serial.println(" ________________________________________________________ Current ________________________________________________________ ");
  
  A_L1 = getData(ADR_L1, 0.02);   Serial.print("L1 Current : ");    Serial.println(A_L1);    delay(10);
  A_L2 = getData(ADR_L2, 0.02);   Serial.print("L2 Current : ");    Serial.println(A_L2);    delay(10);
  A_L3 = getData(ADR_L3, 0.02);   Serial.print("L3 Current : ");    Serial.println(A_L3);    delay(10);

  Serial.println(" ________________________________________________________ True Power ________________________________________________________ ");
  
  TP_L1 = getData(TPDR_L1, 2.0);   Serial.print("L1 True Power : ");    Serial.println(TP_L1);    delay(10);
  TP_L2 = getData(TPDR_L2, 2.0);   Serial.print("L2 True Power : ");    Serial.println(TP_L2);    delay(10);
  TP_L3 = getData(TPDR_L3, 2.0);   Serial.print("L3 True Power : ");    Serial.println(TP_L3);    delay(10);

  Serial.println(" ________________________________________________________ Apparent Power ________________________________________________________ ");
  
  AP_L1 = getData(APDR_L1, 2.0);   Serial.print("L1 Apparent Power : ");    Serial.println(AP_L1);    delay(10);
  AP_L2 = getData(APDR_L2, 2.0);   Serial.print("L2 Apparent Power : ");    Serial.println(AP_L2);    delay(10);
  AP_L3 = getData(APDR_L3, 2.0);   Serial.print("L3 Apparent Power : ");    Serial.println(AP_L3);    delay(10);

  Serial.println(" ________________________________________________________ Reactive Power ________________________________________________________ ");
  if(sq(AP_L1)>sq(TP_L1)){ RP_L1 = sqrt( sq(AP_L1) - sq(TP_L1) );   Serial.print("L1 Reactive Power : ");    Serial.println(RP_L1);    delay(10);} else{RP_L1=0.0;}
  if(sq(AP_L2)>sq(TP_L2)){ RP_L2 = sqrt( sq(AP_L2) - sq(TP_L2) );   Serial.print("L2 Reactive Power : ");    Serial.println(RP_L2);    delay(10);} else{RP_L2=0.0;}
  if(sq(AP_L3)>sq(TP_L3)){ RP_L3 = sqrt( sq(AP_L3) - sq(TP_L3) );   Serial.print("L3 Reactive Power : ");    Serial.println(RP_L3);    delay(10);} else{RP_L3=0.0;}
 
  Serial.println(" ________________________________________________________ Power Factor ________________________________________________________ ");
  
  if (AP_L1 > 0) { PF_L1 = TP_L1 / AP_L1; } else { PF_L1 = 0.0; }
  if (AP_L2 > 0) { PF_L2 = TP_L2 / AP_L2; } else { PF_L2 = 0.0; }
  if (AP_L3 > 0) { PF_L3 = TP_L3 / AP_L3; } else { PF_L1 = 0.0; }
  
  Serial.print("L1 Power Factor : ");   Serial.println(PF_L1);    delay(10);
  Serial.print("L2 Power Factor : ");   Serial.println(PF_L2);    delay(10);
  Serial.print("L3 Power Factor : ");   Serial.println(PF_L3);    delay(10);
 
  Serial.println(" ________________________________________________________ Total 3 Phase Aggregated Values ________________________________________________________ ");

  TF = getData(TFDR, 0.1);           Serial.print("Total Frequency ");                 Serial.println(TF);    delay(10);
  
  TTP = getData(TTPDR, 2.0);         Serial.print("Total 3 Phase True Power ");        Serial.println(TTP);   delay(10);
  
  TAP = getData(TAPDR, 2.0);         Serial.print("Total 3 Phase Apparent Power ");    Serial.println(TAP);   delay(20);

  if(sq(TAP)>sq(TTP)){ TRP = sqrt( sq(TAP) - sq(TTP) );   Serial.print("Total 3 Phase Reactive Power ");    Serial.println(TRP);    delay(10);} else{TRP=0.0;}

  KWH = getData(KWHDR, 0.01);        
  Serial.print("Total KWH ");    
  Serial.println(KWH);   
  delay(100);

  ASP = ASP + KWH; 
  Serial.print("Aggregated System Power ");    
  Serial.println(ASP);   
  delay(100);
  
  if (TAP > 0) { TPF = TTP / TAP; } else { TPF = 0.0; }

  Serial.print("Total 3 Phase Power Factor ");    Serial.println(TPF);    delay(10);

  Serial.println(" ________________________________________________________ Line to Line / Phase Voltages  ________________________________________________________ ");
  
  LV_12 = getData(PVDR_L12, 0.1);   Serial.print("L1-L2 Phase Voltage ");   Serial.println(LV_12);   delay(10);

  LV_23 = getData(PVDR_L23, 0.1);   Serial.print("L2-L3 Phase Voltage ");   Serial.println(LV_23);   delay(10);

  LV_31 = getData(PVDR_L31, 0.1);   Serial.print("L3-L1 Phase Voltage ");   Serial.println(LV_31);   delay(10);

  Serial.println(" ________________________________________________________ Unix Timestamp  ________________________________________________________ ");

  Epoch_Time = Get_Epoch_Time();    Serial.print("Epoch Unix Timestamp : ");   Serial.println(Epoch_Time);    delay(10);
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
    /* Restart ESP32 if not connected for 5 minutes */
    if ( millis() - Time_Checker > 300000 ) {
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

  while ( !Ping_Success ) {
    Serial.println("Failed to Ping www.google.com");
    delay(500);
    bool Ping_Success = Ping.ping("www.google.com", 3);
    /* Restart ESP32 if not successful for 5 minutes */
    if ( millis() - Time_Checker > 300000 ) {
      ESP.restart();
    }
  }

  if ( Ping_Success ) {
    Serial.println();
    Serial.println("Ping successful to www.google.com");
    Serial.println();
  }
}
