#include <HardwareSerial.h>
#include <PZEM004Tv30.h>

// ----------------------------------------------------------------------------------------------------------------------------------------------------------

 HardwareSerial MySerial0(0);
 HardwareSerial MySerial1(1);
 HardwareSerial MySerial2(2);

// L1 Phase Parameters ==============================================================================================================================================================

float V_L1;   // L1 Voltage
float A_L1;   // L1 Current
float TP_L1;  // L1 True Power
float AP_L1;  // L1 Apparent Power   
float RP_L1;  // L1 Reactive Power
float TE_L1;  // L1 Total Energy
float TF_L1;  // L1 Frequency
float PF_L1;  // L1 Power Factor

// L2 Phase Parameters ==============================================================================================================================================================

float V_L2;   // L2 Voltage
float A_L2;   // L2 Current
float TP_L2;  // L2 True Power
float AP_L2;  // L2 Apparent Power
float RP_L2;  // L2 Reactive Power
float TE_L2;  // L2 Total Energy
float TF_L2;  // L2 Frequency
float PF_L2;  // L2 Power Factor

// L3 Phase Parameters ===============================================================================================================================================================

float V_L3;   // L3 Voltage
float A_L3;   // L3 Current
float TP_L3;  // L3 True Power
float AP_L3;  // L3 Apparent Power
float RP_L3;  // L3 Reactive Power
float TE_L3;  // L3 Total Energy
float TF_L3;  // L3 Frequency
float PF_L3;  // L3 Power Factor

// ----------------------------------------------------------------------------------------------------------------------------------------------------------

void setup() {
   pinMode(13,OUTPUT);
   MySerial0.begin(9600, SERIAL_8N1, 14, 15);
   MySerial1.begin(9600, SERIAL_8N1, 16, 17);
   MySerial2.begin(9600, SERIAL_8N1, 18, 19);
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------

void loop() {
  EA1(); 
  delay(1500);
  EA2();
  delay(1500);
  EA3();
  delay(1500);
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------

void EA1() {

  MySerial0.begin(9600, SERIAL_8N1, 14, 15);
  PZEM004Tv30 pzem( &MySerial0, 14, 15 );

  V_L1 = pzem.voltage();

  A_L1 = pzem.current();

  PF_L1 = pzem.pf();

  TE_L1 = pzem.energy();

  TF_L1 = pzem.frequency();

  TP_L1 = pzem.power();

  AP_L1 = TP_L1 / PF_L1;

  RP_L1 = sqrt(sq(AP_L1)-sq(TP_L1));

  digitalWrite(13,HIGH);
  delay(500);
  digitalWrite(13,LOW);
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------

void EA2() {

  MySerial1.begin(9600, SERIAL_8N1, 16, 17);
  PZEM004Tv30 pzem( &MySerial1, 16, 17 );

  V_L2 = pzem.voltage();

  A_L2 = pzem.current();

  PF_L2 = pzem.pf();

  TE_L2 = pzem.energy();

  TF_L2 = pzem.frequency();

  TP_L2 = pzem.power();

  AP_L2 = TP_L2 / PF_L2;

  RP_L2 = sqrt(sq(AP_L2)-sq(TP_L2));

  digitalWrite(13,HIGH);
  delay(500);
  digitalWrite(13,LOW);
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------

void EA3() {

  MySerial2.begin(9600, SERIAL_8N1, 18, 19);  
  PZEM004Tv30 pzem( &MySerial2, 18, 19 );

  V_L3 = pzem.voltage();

  A_L3 = pzem.current();

  PF_L3 = pzem.pf();

  TE_L3 = pzem.energy();

  TF_L3 = pzem.frequency();

  TP_L3 = pzem.power();

  AP_L3 = TP_L3 / PF_L3;

  RP_L3 = sqrt(sq(AP_L3)-sq(TP_L3));

  digitalWrite(13,HIGH);
  delay(500);
  digitalWrite(13,LOW);
}
