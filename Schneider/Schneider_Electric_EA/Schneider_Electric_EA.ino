#include <ModbusMaster.h>
#include <esp_task_wdt.h>

// =====================================================================================================================================================================================

#define MAX485_DE 19    // Driver Enable
#define MAX485_RE 18    // Receiver Enable

// =====================================================================================================================================================================================

ModbusMaster node;

// =====================================================================================================================================================================================

#define WDT_TIMEOUT 300      // 5 Minute

// =====================================================================================================================================================================================

unsigned long timer = 60000;      // JSON Packet Sending time Counter 

// EA Data Registers  ===============================================================================================================================================================

#define VDR_L1 0x2830           // L1 Voltage Data Register
#define VDR_L2 0x2832           // L2 Voltage Data Register
#define VDR_L3 0x2832           // L3 Voltage Data Register

#define ADR_L1 0x3000           // L1 Current Data Register
#define ADR_L2 0x3002           // L2 Current Data Register
#define ADR_L3 0x3004           // L3 Current Data Register

#define TPDR_L1 0x3054          // L1 True Power Data Register
#define TPDR_L2 0x3056          // L2 True Power Data Register
#define TPDR_L3 0x3058          // L3 True Power Data Register

#define APDR_L1 0x3070          // L1 Apparent Power Data Register
#define APDR_L2 0x3072          // L2 Apparent Power Data Register
#define APDR_L3 0x3074          // L3 Apparent Power Data Register

#define PVDR_L12 0x3020         // L1 - L2 Phase Voltage Data Register
#define PVDR_L23 0x3022         // L2 - L3 Phase Voltage Data Register
#define PVDR_L31 0x3024         // L3 - L1 Phase Voltage Data Register

#define TTPDR 0x3060            // Total True Power Data Register
#define TAPDR 0x3076            // Total Apparent Power Data Register

#define TFDR 0x3110             // Total Frequency Data Register
#define KWHDR 0x2700            // Kilo Watt Hour Data Register

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
  uint8_t dataRegisterNode = node.readHoldingRegisters( ( dataRegister - 0x1 ), 2);
  uint16_t buff[2];

  if ( dataRegisterNode == node.ku8MBSuccess ) {

    for (uint8_t j = 0; j < 2; j++) {
      buff[j] = node.getResponseBuffer(j);
    }

    uint32_t value = ( buff[0] << 16 ) + ( buff[1] );
//    float data = HexTofloat(value);
    result = (*(float*)&value);
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

  pinMode(MAX485_RE, OUTPUT);
  pinMode(MAX485_DE, OUTPUT); 
  
  digitalWrite(MAX485_RE, 0);
  digitalWrite(MAX485_DE, 0);

  Serial1.begin(9600, SERIAL_8N1, 16, 17);  // ( Baud rate = 9600, Configuration = [8 Data bits,No parity,1 Stop bit], Rx pin, Tx pin)

  node.begin(2, Serial1);                   // Modbus slave ID 1

  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
  
}

// ====================================================================================================================================================================================

void loop() { 
  Read_EA_Param();
  delay(10000);
  esp_task_wdt_reset();
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
  if(sq(AP_L1)>sq(TP_L1)){ RP_L1 = sqrt( sq(AP_L1) - sq(TP_L1) );   Serial.print("L1 Reactive Power : ");    Serial.println(RP_L1);    delay(10);} else {RP_L1=0.0;}
  if(sq(AP_L2)>sq(TP_L2)){ RP_L2 = sqrt( sq(AP_L2) - sq(TP_L2) );   Serial.print("L2 Reactive Power : ");    Serial.println(RP_L2);    delay(10);} else {RP_L2=0.0;}
  if(sq(AP_L3)>sq(TP_L3)){ RP_L3 = sqrt( sq(AP_L3) - sq(TP_L3) );   Serial.print("L3 Reactive Power : ");    Serial.println(RP_L3);    delay(10);} else {RP_L3=0.0;}
 
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
  
  TAP = getData(TAPDR, 2.0);         Serial.print("Total 3 Phase Apparent Power ");    Serial.println(TAP);   delay(10);

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

}
