#include <PZEM004Tv30.h>
#include <Wire.h>

// ----------------------------------------------------------------------------------------------------------------------------------------------------------

//PZEM004Tv30 pzem(8, 9);  // Software Serial pin 8 (RX) & 9 (TX)

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

// 3 Phase Aggregated Parameters ======================================================================================================================================================

// float TTP;      // Total True Power
// float TAP;      // Total Apparent Power
// float TRP;      // Total Reactive Power
// float TPF;      // Total Power Factor
// float TF;       // Total Frequency
// float KWH;      // Total Kwh

// Line to Line Voltages ==============================================================================================================================================================

// float LV_12;    // L1-L2 Voltages
// float LV_23;    // L2-L3 Voltages
// float LV_31;    // L3-L1 Voltages

// ----------------------------------------------------------------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);
  pinMode(12, INPUT);
  pinMode(13, INPUT);

}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------

void loop() {
  Serial.println("Getting Parameters");
  EA1(); 
  delay(500);
  EA2();
  delay(500);
  EA3();
  delay(500);
  print_data();
  delay(500);
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------

void EA1() {

  // PZEM004Tv30 pzem(2, 3);
  PZEM004Tv30 pzem(D3, D4);

  V_L1 = pzem.voltage();

  A_L1 = pzem.current();

  PF_L1 = pzem.pf();

  TE_L1 = pzem.energy();

  TF_L1 = pzem.frequency();

  TP_L1 = pzem.power();

  AP_L1 = TP_L1 / PF_L1;

  RP_L1 = sqrt(sq(AP_L1)-sq(TP_L1));

}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------

void EA2() {

  // PZEM004Tv30 pzem(4, 5);
  PZEM004Tv30 pzem(14, 15);

  V_L2 = pzem.voltage();

  A_L2 = pzem.current();

  PF_L2 = pzem.pf();

  TE_L2 = pzem.energy();

  TF_L2 = pzem.frequency();

  TP_L2 = pzem.power();

  AP_L2 = TP_L2 / PF_L2;

  RP_L2 = sqrt(sq(AP_L2)-sq(TP_L2));

}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------

void EA3() {

  // PZEM004Tv30 pzem(6, 7);

  PZEM004Tv30 pzem(18, 19);

  V_L3 = pzem.voltage();

  A_L3 = pzem.current();

  PF_L3 = pzem.pf();

  TE_L3 = pzem.energy();

  TF_L3 = pzem.frequency();

  TP_L3 = pzem.power();

  AP_L3 = TP_L3 / PF_L3;

  RP_L3 = sqrt(sq(AP_L3)-sq(TP_L3));

}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------

void print_data() {

  Serial.print("V1 : ");      Serial.print(V_L1);      Serial.println(" V");
  Serial.print("V2 : ");      Serial.print(V_L2);      Serial.println(" V");
  Serial.print("V3 : ");      Serial.print(V_L3);      Serial.println(" V");

  Serial.print("C1 : ");      Serial.print(A_L1);      Serial.println(" A");
  Serial.print("C2 : ");      Serial.print(A_L2);      Serial.println(" A");
  Serial.print("C3 : ");      Serial.print(A_L3);      Serial.println(" A");

  Serial.print("TP1 : ");     Serial.print(TP_L1);     Serial.println(" W");
  Serial.print("TP2 : ");     Serial.print(TP_L2);     Serial.println(" W");
  Serial.print("TP3 : ");     Serial.print(TP_L3);     Serial.println(" W");

  Serial.print("AP1 : ");     Serial.print(AP_L1);     Serial.println(" W");
  Serial.print("AP2 : ");     Serial.print(AP_L2);     Serial.println(" W");
  Serial.print("AP3 : ");     Serial.print(AP_L3);     Serial.println(" W");

  Serial.print("RP1 : ");     Serial.print(RP_L1);     Serial.println(" W");
  Serial.print("RP2 : ");     Serial.print(RP_L2);     Serial.println(" W");
  Serial.print("RP3 : ");     Serial.print(RP_L3);     Serial.println(" W");

  Serial.print("TE1 : ");     Serial.print(TE_L1);     Serial.println(" KWH");
  Serial.print("TE2 : ");     Serial.print(TE_L2);     Serial.println(" KWH");
  Serial.print("TE3 : ");     Serial.print(TE_L3);     Serial.println(" KWH");

  Serial.print("TF1 : ");     Serial.print(TF_L1);     Serial.println(" Hz");
  Serial.print("TF2 : ");     Serial.print(TF_L2);     Serial.println(" Hz");
  Serial.print("TF3 : ");     Serial.print(TF_L3);     Serial.println(" Hz");

  Serial.print("PF1 : ");     Serial.print(PF_L1);     Serial.println(" PF");
  Serial.print("PF2 : ");     Serial.print(PF_L2);     Serial.println(" PF");
  Serial.print("PF3 : ");     Serial.print(PF_L3);     Serial.println(" PF");

}