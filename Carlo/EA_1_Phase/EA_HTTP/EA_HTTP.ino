#include <WiFi.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <ModbusMaster.h>

#define MAX485_DE 14    // Driver Enable
#define MAX485_RE 32    // Receiver Enable

ModbusMaster node;

// JSON Packet Sending time Counter
unsigned long timer = 10000;

const char* ssid = "Sam";
const char* pass = "samamaxx";

// ==================================================================================================================================================================================

// Phase 1 Credentials

// Significant Bit Variables ( Voltage, Current, True Power, Apparent Power, Frequency ) .........................................
uint16_t V1_LSB, V1_MSB, C1_LSB, C1_MSB, TP1_LSB, TP1_MSB, AP1_LSB, AP1_MSB, F1_LSB, F1_MSB;

// Data Registers value holder .........................
uint8_t V1_Data_Reg, C1_Data_Reg, TP1_Data_Reg, AP1_Data_Reg, F1_Data_Reg;

// Local Holding Variables ......................
uint16_t V1, C1, P1, AP1, F1;

// Global Holding Variables ......................
float L1V, L1C, L1TP, L1AP, L1RP, L1PF, L1F;

// ==================================================================================================================================================================================

void setup()
{
  pinMode(MAX485_RE, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);

  digitalWrite(MAX485_RE, 0);
  digitalWrite(MAX485_DE, 0);

  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 16, 17);  // ( Baud rate = 9600, Configuration = [8 Data bits,No parity,1 Stop bit], Rx pin, Tx pin)

  node.begin(1, Serial1);                   // Modbus slave ID 1

  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  Serial.println("Starting connecting WiFi.");
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// ==================================================================================================================================================================================

void loop() {
  Serial.println("***************** LINE 1 PARAMETERS *********************");
  delay(100);
  get_Voltage();
  delay(100);
  get_Current();
  delay(100);
  get_True_Power();
  delay(100);
  get_Power_Derivatives();
  delay(100);
  get_Frequency();
  delay(100);
  Serial.println("*********************************************************");
  delay(100);

  // JSON Packet Sent after every 300 sec ...................................................
  if ( millis() >= timer )
  {
    timer = millis() + 10000UL;
    json_packet_sender();
  }
}

// ==================================================================================================================================================================================

void preTransmission()
{
  digitalWrite(MAX485_RE, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE, 0);
  digitalWrite(MAX485_DE, 0);
}

// ==================================================================================================================================================================================

void get_Voltage() {

  V1_Data_Reg = node.readHoldingRegisters(0x0280, 2);           // This loop is to get the output of 4 bytes (Float datatype)

  if ( V1_Data_Reg == node.ku8MBSuccess )                       // [readHoldingRegisters(Address of the register, Number of registers want to be read from this address)]
  {
    V1_LSB = node.getResponseBuffer(0x00);
    V1_MSB = node.getResponseBuffer(0x01);

    uint16_t V_Data[2] = {V1_LSB, V1_MSB};
    memcpy(&V1, V_Data, 4);

    Serial.print("L1 Phase Voltage: ");
    L1V = V1 * 0.1;
    Serial.print(L1V);
    Serial.println(" V");
  }
}

// ==================================================================================================================================================================================

void get_Current() {

  C1_Data_Reg = node.readHoldingRegisters(0x0282, 2);    // This loop is to get the output of 4 bytes (Float datatype)

  if ( C1_Data_Reg == node.ku8MBSuccess )
  {
    C1_LSB = node.getResponseBuffer(0x00);
    C1_MSB = node.getResponseBuffer(0x01);

    uint16_t C_Data[2] = {C1_LSB, C1_MSB};
    memcpy(&C1, C_Data, 4);

    Serial.print("L1 Phase Current: ");
    L1C = C1 * 0.001 * 7;
    Serial.print(L1C);
    Serial.println(" A");
  }
}

// ==================================================================================================================================================================================

void get_True_Power() {

  TP1_Data_Reg = node.readHoldingRegisters(0x0284, 2);      // This loop is to get the output of 4 bytes (Float datatype)

  if (TP1_Data_Reg == node.ku8MBSuccess)
  {
    TP1_LSB = node.getResponseBuffer(0x00);
    TP1_MSB = node.getResponseBuffer(0x01);

    uint16_t P_Data[2] = {TP1_LSB, TP1_MSB};
    memcpy(&P1, P_Data, 4);

    Serial.print("L1 Phase Power: ");
    L1TP = P1 * 0.7;
    Serial.print(L1TP);
    Serial.println(" W");
  }
}

// ==================================================================================================================================================================================

void get_Power_Derivatives() {

  AP1_Data_Reg = node.readHoldingRegisters(0x02A0, 2);         // This loop is to get the output of 4 bytes (Float datatype)

  if (AP1_Data_Reg == node.ku8MBSuccess)
  {
    AP1_LSB = node.getResponseBuffer(0x00);
    AP1_MSB = node.getResponseBuffer(0x01);

    uint16_t AP_Data[2] = {AP1_LSB, AP1_MSB};
    memcpy(&AP1, AP_Data, 4);

    // Apparent Power
    Serial.print("Apparent Power: ");
    L1AP = AP1 * 0.7;
    Serial.print(L1AP);
    Serial.println(" VA");

    // Reactive Power
    Serial.print("Reactive Power: ");
    L1RP = sqrt(sq(L1AP) - sq(L1TP));
    Serial.print(L1RP);
    Serial.println(" VAR");

    // Power Factor
    if (L1AP > 0) {
      Serial.print("Power Factor: ");
      L1PF = L1TP / L1AP;
      Serial.print(L1PF);
      Serial.println(" PF");
    }
  }
}

// ==================================================================================================================================================================================

void get_Frequency() {

  F1_Data_Reg = node.readHoldingRegisters(0x02B8, 2);        // This loop is to get the output of 4 bytes (Float datatype)
  if (F1_Data_Reg == node.ku8MBSuccess)                      // [readHoldingRegisters(Address of the register, Number of registers want to be read from this address)]
  {
    F1_LSB = node.getResponseBuffer(0x00);
    F1_MSB = node.getResponseBuffer(0x01);

    uint16_t F_Data[2] = {F1_LSB, F1_MSB};
    memcpy(&F1, F_Data, 4);

    Serial.print("Phase A Frequency: ");
    L1F = F1 * 0.1;
    Serial.print(L1F);
    Serial.println("  Hz");
  }
}

// ==================================================================================================================================================================================

void json_packet_sender() {

  StaticJsonBuffer<300> JSON_Packet;
  JsonObject& JSON_Entry = JSON_Packet.createObject();

  JSON_Entry["device"] = "EA1";
  JSON_Entry["F1"] = L1F;
  JSON_Entry["V1"] = L1V;
  JSON_Entry["C1"] = L1C;
  JSON_Entry["TP1"] = L1TP;
  JSON_Entry["AP1"] = L1AP;
  JSON_Entry["RP1"] = L1RP;
  JSON_Entry["PF1"] = L1PF;

  char JSONmessageBuffer[300];
  JSON_Entry.prettyPrintTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.print(JSONmessageBuffer);

  if (WiFi.status() == WL_CONNECTED) { //Check WiFi connection status

    HTTPClient http;
    http.begin("https://api.datacake.co/integrations/api/1a679601-42aa-44db-a966-a4fb7f91e3f0/");  //Specify destination for HTTP request
    http.addHeader("Content-Type", "application/json");             // Specify content-type header
    int httpResponseCode = http.POST(JSONmessageBuffer);            // Send the actual POST request

    if (httpResponseCode > 0) {
      String response = http.getString();   // Get the response to the request
      Serial.println(httpResponseCode);     // Print return code
      Serial.println(response);             // Print request answer
    } else {
      Serial.print("Error on sending POST: ");
      Serial.println(httpResponseCode);
    }
    http.end();  //Free resources
  } else {
    Serial.println("Error in WiFi connection");
  }
}
