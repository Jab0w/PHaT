
// Global variables

int PCommand=0;
bool Error = 0;
int count = 0;
float calibrationFactor = 0.00488*1.769; // Calibration*VoltageDivider
float voltage;
float pinvoltage;
#include <Wire.h>
#include <Adafruit_MAX31865.h>
#include <SPI.h>
#include "PinChangeInterrupt.h"
#include <SD.h>
#define Trigger_PIN  7

#define sd 1
#define RTD 1
// Use software SPI: CS, DI, DO, CLK

#ifdef RTD
Adafruit_MAX31865 max = Adafruit_MAX31865(3);
float RREF = 4300.0;
#endif

#ifdef sd
File TestFile;
#endif

void setup() {
  // put your setup code here, to run once:

// Setup for RTD
  init_io();
  Serial.begin(9600);
  Serial.println("PHaT - Start");

#ifdef sd
  Serial.println("SD Card Printing");
#endif
  ///// Setup for Pressure Sensor ///////
/*
  SPI.begin();
  SPI_PT_WRITE(PCommand=0xAE);
  short Output;
  Output = SPI_PT_WRITE(a=0xA7);
  Serial.println(Output);
  Output = SPI_PT_WRITE(a=0x00);
  Serial.println(Output);
  Output = SPI_PT_WRITE(a=0x00);
  Serial.println(Output);
  digitalWrite(5, HIGH);
  SPI.endTransaction();
*/
 //////////// Setup for SD Card ////////////////
#ifdef sd
  const int CSSD = 4;
  //digitalWrite(4, LOW);
  //Serial.print("Initializing SD card...");

  if (!SD.begin(CSSD)) {
    //Serial.println("initialization failed!");
    Error = 1;
  }
  //Serial.println("initialization done.");
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  TestFile = SD.open("test2.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (TestFile) {
    //Serial.print("Writing to test.txt...");
    //TestFile.println("testing 1, 2, 3.");
    // close the file:
    TestFile.close();
    //Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    //Serial.println("error opening test.txt");
    Error = 1;
      // re-open the file for reading:
  }
  //TestFile = SD.open("test.txt");
  //if (TestFile) {
  //  Serial.println("test.txt:");

    // read from the file until there's nothing else in it:
    //while (TestFile.available()) {
    //  Serial.write(TestFile.read());
    //}
    // close the file:
  //  TestFile.close();
  //} else {
    // if the file didn't open, print an error:
  //  Serial.println("error opening test.txt");
  //  Error = 1;
  //}
#endif
  if (Error){
    digitalWrite(A3, HIGH);
    delay(1000);
  } else {
    digitalWrite(A3, HIGH);
    delay(80);
    digitalWrite(A3, LOW);
    delay(80);
    digitalWrite(A3, HIGH);
    delay(80);
    digitalWrite(A3, LOW);
  }

}

void loop() {
  #ifdef sd
    TestFile = SD.open("test.txt", FILE_WRITE);
  #endif
  // loop for RH            // If battery lower than 6.5V the frequency starts to drop below 12KHz
                                       // For all other battery power the frequency is stable at 12KHz
  
  digitalWrite(A0, HIGH);
  Serial.println("Saving Count");
  //TestFile.print("Count, ");
  //TestFile.println(count);
  digitalWrite(A0, LOW);
  count = 0;
  delay(1000);

  // Check Battery Voltage

  voltage = analogRead(A7) * calibrationFactor;     // Voltage can be read down to a battery power of 6.5 V
  pinvoltage = analogRead(A7) * 0.00488;
  digitalWrite(A1, HIGH);
  //Serial.println("Saving Voltage");
  Serial.print(F("Battery[V], "));
  Serial.println(voltage);

  //TestFile.print(F("Pin[V], "));
  //TestFile.println(pinvoltage);
  digitalWrite(A1, LOW);

  // Loop for Pressure
/*
  short Output;
  Output = SPI_PT_WRITE(a=0x02);
  digitalWrite(5, HIGH);
  SPI.endTransaction();
  Serial.println(Output);
*/
/////////// Loop for RTD sensor //////////////
#ifdef RTD
  float rtd = max.readRTD();
  Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;

  //Serial.print("Ratio = "); Serial.println(ratio,8);
  //Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  digitalWrite(A2, HIGH);
  //Serial.println("Saving Temperature");
  //Serial.print("Temp[C], ");
  //Serial.println(max.temperature(1000, RREF)); //More accurate way to do this
  digitalWrite(A2, LOW);

  // Check and print any faults
  uint8_t fault = max.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    max.clearFault();
  }
#endif

  ////////////
#ifdef sd
  TestFile.close();
#endif
}