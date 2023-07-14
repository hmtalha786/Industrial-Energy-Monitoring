#include <ModbusMaster.h>

// =====================================================================================================================================================================================

#define PIN_5V_EN 16

#define RS485_EN_PIN 17 // 17 /RE
#define RS485_TX_PIN 22 // 21
#define RS485_RX_PIN 21 // 22
#define RS485_SE_PIN 19 // 22 /SHDN

// =====================================================================================================================================================================================

ModbusMaster node;

// =====================================================================================================================================================================================

unsigned long timer = 60000;      // JSON Packet Sending time Counter 

// EA Data Registers  ===============================================================================================================================================================

#define VDR_L1 0xFA2           // L1 Voltage Data Register
#define ADR_L1 0xFBA           // L1 Current Data Register
#define TPDR_L1 0xFCA          // L1 True Power Data Register
#define APDR_L1 0xFE8          // L1 Apparent Power Data Register
#define TAEDR_L1 0x1015        // L1 Active Energy Data Register

#define VDR_L2 0xFA3           // L2 Voltage Data Register
#define ADR_L2 0xFBB           // L2 Current Data Register
#define TPDR_L2 0xFCB          // L2 True Power Data Register
#define APDR_L2 0xFE9          // L2 Apparent Power Data Register
#define TAEDR_L2 0x1017        // L2 Active Energy Data Register

#define VDR_L3 0xFA4           // L3 Voltage Data Register
#define ADR_L3 0xFBC           // L3 Current Data Register
#define TPDR_L3 0xFCC          // L3 True Power Data Register
#define APDR_L3 0xFEA          // L3 Apparent Power Data Register
#define TAEDR_L3 0x1019        // L3 Active Energy Data Register

#define PVDR_L12 0xFAE         // L1 - L2 Phase Voltage Data Register
#define PVDR_L23 0xFAF         // L2 - L3 Phase Voltage Data Register
#define PVDR_L31 0xFB0         // L3 - L1 Phase Voltage Data Register

#define TFDR 0x1009            // L1 Frequency Data Register

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

// Line to Line Voltages ==============================================================================================================================================================

float TTP;      // Total True Power
float TAP;      // Total Apparent Power
float TRP;      // Total Reactive Power
float TPF;      // Total Power Factor
float TF;       // Total Frequency
float CSP;      // Total Calculated Power
float ASP;      // Total Accumulated Power
float TAE;      // Total Active Energy

// Line to Line Voltages ==============================================================================================================================================================

float LV_12;    // L1-L2 Voltages
float LV_23;    // L2-L3 Voltages
float LV_31;    // L3-L1 Voltages

// Total Active Energies ==============================================================================================================================================================

float TTE1;     // Total Active Energy L1
float TTE2;     // Total Active Energy L2
float TTE3;     // Total Active Energy L3

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
  double result;
  uint8_t dataRegisterNode = node.readHoldingRegisters(dataRegister, 2);
  if ( dataRegisterNode == node.ku8MBSuccess )                       // [readHoldingRegisters(Address of the register, Number of registers want to be read from this address)]
  {
    uint16_t data_LSB = node.getResponseBuffer(0x00);
    uint16_t data_MSB = node.getResponseBuffer(0x01);
    
    uint32_t joinedData = (data_LSB<<16) | (data_MSB);    
    result = joinedData;
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
  delay(100);

  pinMode(PIN_5V_EN, OUTPUT);
  pinMode(RS485_EN_PIN, OUTPUT);
  pinMode(RS485_SE_PIN, OUTPUT);

  digitalWrite(PIN_5V_EN, HIGH);
  digitalWrite(RS485_EN_PIN, HIGH);
  digitalWrite(RS485_SE_PIN, HIGH);

  Serial1.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

  delay(50);
  node.begin(1, Serial1);
  Serial.println("testing");

}

// ====================================================================================================================================================================================

void loop() {
  if ( millis() >= timer ) { 
    Serial.println("");
    Serial.println(" ++++++++++++++++++ Getting Tense Analyzer Data ++++++++++++++++++ ");  
    timer = millis() + 60000UL;  
    Read_EA_Param();
    Serial.println(""); 
  }
}

// ==================================================================================================================================================================================

void Read_EA_Param() {
  
  Serial.println(""); //------------------------------------------------------------------------------------------------------------------

  Serial.print("L1 Voltage : "); 
  V_L1 = getData(VDR_L1, 0.1);      
  Serial.println(V_L1);    
  delay(50);

  Serial.print("L2 Voltage : ");
  V_L2 = getData(VDR_L2, 0.1);       
  Serial.println(V_L2);    
  delay(50);

  Serial.print("L3 Voltage : ");
  V_L3 = getData(VDR_L3, 0.1);       
  Serial.println(V_L3);    
  delay(50);

  Serial.println(""); //------------------------------------------------------------------------------------------------------------------

  Serial.print("L1 Current : ");
  A_L1 = getData(ADR_L1, 0.8);       
  Serial.println(A_L1);    
  delay(50);

  Serial.print("L2 Current : ");
  A_L2 = getData(ADR_L2, 0.8);       
  Serial.println(A_L2);    
  delay(50);

  Serial.print("L3 Current : ");
  A_L3 = getData(ADR_L3, 0.8);       
  Serial.println(A_L3);    
  delay(50);

  Serial.println(""); //------------------------------------------------------------------------------------------------------------------

  Serial.print("L1 True Power : ");
  TP_L1 = getData(TPDR_L1, 80);       
  Serial.println(TP_L1);    
  delay(50);

  Serial.print("L2 True Power : ");
  TP_L2 = getData(TPDR_L2, 80);       
  Serial.println(TP_L2);    
  delay(50);

  Serial.print("L3 True Power : ");  
  TP_L3 = getData(TPDR_L3, 80);     
  Serial.println(TP_L3);    
  delay(50);
  
  Serial.println(""); //------------------------------------------------------------------------------------------------------------------
  
  Serial.print("L1 Apparent Power : "); 
  AP_L1 = getData(APDR_L1, 80);      
  Serial.println(AP_L1);    
  delay(50);

  Serial.print("L2 Apparent Power : ");  
  AP_L2 = getData(APDR_L2, 80);     
  Serial.println(AP_L2);    
  delay(50);

  Serial.print("L3 Apparent Power : "); 
  AP_L3 = getData(APDR_L3, 80);      
  Serial.println(AP_L3);    
  delay(50);

  Serial.println(""); //------------------------------------------------------------------------------------------------------------------

  Serial.print("L1 Reactive Power : ");
  if (sq(AP_L1) > sq(TP_L1)) { RP_L1 = sqrt( sq(AP_L1) - sq(TP_L1) ); } else { RP_L1 = 0.0; }
  Serial.println(RP_L1);   
  delay(50);

  Serial.print("L2 Reactive Power : ");  
  if (sq(AP_L2) > sq(TP_L2)) { RP_L2 = sqrt( sq(AP_L2) - sq(TP_L2) ); } else { RP_L2 = 0.0; }
  Serial.println(RP_L2);   
  delay(50);

  Serial.print("L3 Reactive Power : ");   
  if (sq(AP_L3) > sq(TP_L3)) { RP_L3 = sqrt( sq(AP_L3) - sq(TP_L3) ); } else { RP_L3 = 0.0; }
  Serial.println(RP_L3);   
  delay(50);

  Serial.println(""); //------------------------------------------------------------------------------------------------------------------
  
  Serial.print("3-Phase Active Power : "); 
  TTP = TP_L1 + TP_L2 + TP_L3;       
  Serial.println(TTP);   
  delay(50);

  Serial.print("3-Phase Apparent Power : ");
  TAP = AP_L1 + AP_L2 + AP_L3;        
  Serial.println(TAP);   
  delay(50);
  
  Serial.print("3-Phase Reactive Power : ");
  if ( sq( TAP ) > sq( TTP ) ) { TRP = sqrt( sq( TAP ) - sq( TTP ) ); } else { TRP = 0.0; }
  Serial.println(TRP);  
  delay(50); 

  Serial.println(""); //------------------------------------------------------------------------------------------------------------------
  
  Serial.print("3-Phase Calculated Active Power : ");
  CSP = ( TTP / 60 );    
  Serial.println(CSP);  
  delay(50);

  Serial.print("3-Phase Aggregated Active Power : ");
  ASP = ASP + CSP;     
  Serial.println(ASP);  
  delay(50);

  Serial.println(""); //------------------------------------------------------------------------------------------------------------------
  
  Serial.print("L1 Power Factor : ");   
  if (AP_L1 > 0) { PF_L1 = TP_L1 / AP_L1; } else { PF_L1 = 0.0; }
  Serial.println(PF_L1);    
  delay(50);

  Serial.print("L2 Power Factor : ");   
  if (AP_L2 > 0) { PF_L2 = TP_L2 / AP_L2; } else { PF_L2 = 0.0; }
  Serial.println(PF_L2);    
  delay(50);

  Serial.print("L3 Power Factor : ");
  if (AP_L3 > 0) { PF_L3 = TP_L3 / AP_L3; } else { PF_L1 = 0.0; }
  Serial.println(PF_L3);    
  delay(50);
  
  Serial.println(""); //------------------------------------------------------------------------------------------------------------------
  
  Serial.print("3-Phase Power Factor : ");
  if (TAP > 0) { TPF = TTP / TAP; } else { TPF = 0.0; }
  Serial.println(TPF);    
  delay(50);

  Serial.println(""); //------------------------------------------------------------------------------------------------------------------
  
  Serial.print("L1-L2 Phase Voltage : ");
  LV_12 = getData(PVDR_L12, 0.1);      
  Serial.println(LV_12);   
  delay(50);

  Serial.print("L2-L3 Phase Voltage : ");
  LV_23 = getData(PVDR_L23, 0.1);      
  Serial.println(LV_23);   
  delay(50);

  Serial.print("L3-L1 Phase Voltage : "); 
  LV_31 = getData(PVDR_L31, 0.1);     
  Serial.println(LV_31);   
  delay(50);

  Serial.println(""); //------------------------------------------------------------------------------------------------------------------
  
  Serial.print("Overall Frequency : ");
  TF = getData(TFDR, 0.1);       
  Serial.println(TF);    
  delay(50);
  
  Serial.println(""); //------------------------------------------------------------------------------------------------------------------
  
  Serial.print("L1 Active Energy : ");
  TTE1 = getData_fourbytes(TAEDR_L1, 1);                 
  Serial.println(TTE1);   
  delay(50);

  Serial.print("L2 Active Energy : ");
  TTE2 = getData_fourbytes(TAEDR_L2, 1);                 
  Serial.println(TTE2);   
  delay(50);

  Serial.print("L3 Active Energy : "); 
  TTE3 = getData_fourbytes(TAEDR_L3, 1);                
  Serial.println(TTE3);   
  delay(50);

  Serial.println(""); //------------------------------------------------------------------------------------------------------------------
  
  Serial.print("3-Phase True Energy : "); 
  TAE = TTE1 + TTE2 + TTE3;       
  Serial.println(TAE);   
  delay(50);
}
