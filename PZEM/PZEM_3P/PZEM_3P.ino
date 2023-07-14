#include <PZEM004Tv30.h>
#include <Wire.h>

//PZEM004Tv30 pzem(8, 9);  // Software Serial pin 8 (RX) & 9 (TX)

// ----------------------------------------------------------------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(9600);
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------

void loop() {

  EA1();
  EA2();
  EA3();
  
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------

void EA1() {

  PZEM004Tv30 pzem(2, 3);

  float voltage = pzem.voltage();
  float current = pzem.current();
  float power = pzem.power();
  float energy = pzem.energy();
  float frequency = pzem.frequency();
  float pf = pzem.pf();

  Serial.println("Parameters of Energy Analyzer 1");

  get_data(voltage, current, power, energy, frequency, pf);

  delay(2000);
  Serial.println();
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------

void EA2() {

  PZEM004Tv30 pzem(4, 5);

  float voltage = pzem.voltage();
  float current = pzem.current();
  float power = pzem.power();
  float energy = pzem.energy();
  float frequency = pzem.frequency();
  float pf = pzem.pf();

  Serial.println("Parameters of Energy Analyzer 2");

  get_data(voltage, current, power, energy, frequency, pf);

  delay(2000);
  Serial.println();
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------

void EA3() {

  PZEM004Tv30 pzem(6, 7);

  float voltage = pzem.voltage();
  float current = pzem.current();
  float power = pzem.power();
  float energy = pzem.energy();
  float frequency = pzem.frequency();
  float pf = pzem.pf();

  Serial.println("Parameters of Energy Analyzer 3");

  get_data(voltage, current, power, energy, frequency, pf);

  delay(2000);
  Serial.println();
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------

void get_data(float voltage, float current, float power, float energy, float frequency, float pf) {

  // float voltage = pzem.voltage();
  if (voltage != NAN) {
    Serial.print("Voltage : ");
    Serial.print(voltage);
    Serial.println("V");
  } else {
    Serial.println("Error reading voltage");
  }

  // float current = pzem.current();
  if (current != NAN) {
    Serial.print("Current: ");
    Serial.print(current);
    Serial.println("A");
  } else {
    Serial.println("Error reading current");
  }

  // float power = pzem.power();
  if (current != NAN) {
    Serial.print("Power: ");
    Serial.print(power);
    Serial.println("W");
  } else {
    Serial.println("Error reading power");
  }

  // float energy = pzem.energy();
  if (current != NAN) {
    Serial.print("Energy: ");
    Serial.print(energy, 3);
    Serial.println("kWh");

  } else {
    Serial.println("Error reading energy");
  }

  // float frequency = pzem.frequency();
  if (current != NAN) {
    Serial.print("Frequency: ");
    Serial.print(frequency, 1);
    Serial.println("Hz");
  } else {
    Serial.println("Error reading frequency");
  }

  // float pf = pzem.pf();
  if (current != NAN) {
    Serial.print("PF: ");
    Serial.println(pf);
  } else {
    Serial.println("Error reading power factor");
  }

}