//// Global variables

//Macro Variables
byte TxByte = 0;
byte RxByte;
String FileTitle = "PHaTData";
String FileFormat = ".txt";
String FileName;
int Error[16];
bool Flag[16];
unsigned int ErrorSum;
// Variable for RTD sensor
float Ext_Temperature = 0;
// Variable for Pressure Sensor
int PCommand=0;
byte P_Data1;
byte P_Data2;
byte P_Data3;
unsigned short n_prom[8];
unsigned int n_rem;
unsigned long UCTemperature;
unsigned long UCPressure;
int SPIRate = 100000;
//
// Variables for 555 timer/RH sensor
float B5 = 0.0014;    // 1/degree C
float B2 = 0.1325;    // %RH/degree C
float B3 = -0.0317;   // unitless
float B4 = -3.0876;   // %RH
int count = 0;
int count_cap = 0;
int count_delay = 500;     // number of miliseconds to count 555 timer interupts
float f_ref = 3.6E3;     // The number of interupts detected in 0.5 seconds at 30% RH
float c_ref = 650;     // The capacitance of RH sensor at 30% RH in pF
float RH_const = f_ref*c_ref;
float c_humid = 0.0;
float RH_value = 30.0;
float Change_RH = 0;
//
// Varables for Voltage Reader
float calibrationFactor = 0.00488*1.769; // Calibration*VoltageDivider
float voltage;
float pinvoltage;
//
#include "Wire.h"
#include <Adafruit_MAX31865.h>
#include "SPI.h"
#include "PinChangeInterrupt.h"
#include "SD.h"
#define Trigger_PIN  7
// Independent Parts, a 0 will remove that part from the code
#define sd 1           // should always be 1 or code won't work
#define RTD 1
#define PS 1
#define RH 1
#define V 1
#define Time 1

//i2c internal functions
#define external_i2c 1
//#define ahe_debug 1 //debug for i2c internal data

#ifdef RTD
Adafruit_MAX31865 max = Adafruit_MAX31865(3);
float RREF = 4300.0;
float RNOMINAL = 1000.0;
#endif

#ifdef sd
File DataFile;    // File name the code will know to write to
File APRSFile;
#endif

//ahe - global i2c variables
char i2c_data[20];
int i2c_valid;
int i2c_cnt = 0;

char i2c_file_data[70];

// GPS data handelling 
// ahe
char GPS_str[50];  //<time>,<lon>,<lat>
char *gps_data = GPS_str;
int gps_valid_cmd = 0;
int gps_valid_data = 0;



void setup() {



  init_io();
  Serial.begin(57600); //changed to match BT debugging
  Serial.println("PHaT - Start");


  //ahe added function calls for i2c start
  #ifdef external_i2c
      Wire.begin(8);                // join i2c bus with address #8
      Wire.onReceive(receiveEvent); // register event
  #endif

  #ifdef RTD
    max.begin(MAX31865_3WIRE);
    Serial.println("PHaT - 1");
  #endif

    ///// Setup for Pressure Sensor ///////
  #ifdef PS
    SPI.begin();
    SPI.beginTransaction(SPISettings(SPIRate, MSBFIRST, SPI_MODE0));
    digitalWrite(5, LOW);
    delayMicroseconds(50);
    SPI.transfer(0x1E);
    delay(3);
    digitalWrite(5, HIGH);
    delay(1);

    Serial.println("PHaT - 2");

    for(int i=0;i<8;i++){
      unsigned short a = 0xA0+i*2;
      digitalWrite(5, LOW);
      delayMicroseconds(50);
      SPI.transfer(a);
      n_prom[i] = SPI.transfer16(0x00);
      delayMicroseconds(50);
      digitalWrite(5, HIGH);
      SPI.endTransaction();
      delay(1);

    }

    n_rem = crc4(n_prom);
    if (n_rem != 15){
      Error[0] = 1;
      Serial.println(n_rem);
    }
  #endif

  Serial.println("PHaT - 3");
  Serial.println(Error[0]);
  //////////// Setup for SD Card ////////////////
  #ifdef sd
    const int CSSD = 4;
    digitalWrite(4, LOW);
    //Serial.println("Initializing SD card...");

    if (!SD.begin(CSSD)) {
      Error[1] = 1;
      Serial.println(Error[1]);
    }
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    int m = 0;
    FileName = FileTitle + FileFormat;
    //while (SD.exists(FileName) == true){
    //  m=m+1;
    // FileName = FileTitle + FileFormat;
    //}
    //FileName = "PHaT_Data_"+m;
    Serial.println(FileName);
    DataFile = SD.open(FileName, FILE_WRITE);    // Actual name of the file

    // if the file opened okay, write to it:
    if (DataFile) {
      DataFile.println("External Temp[C], Count, Humidity[%RH], Pressure[Pa], Internal Temp[C], Time[s], Batt Voltage[V], Pin Voltage[V], Flags");
      // close the file:
      Serial.println("PHaT - 4");
      DataFile.close();
    } else {
      // if the file didn't open, print an error:
      Error[2] = 1;
      Serial.println(Error[2]);
        // re-open the file for reading:
    }
    DataFile = SD.open(FileName);
    if (DataFile) {
      Serial.println("PHaT - 5");
      // close the file:
      DataFile.close();
    } else {
      // if the file didn't open, print an error:
      Error[3] = 1;
    }

    ErrorSum = SumArray(Error);
    Serial.println(ErrorSum);
    if (ErrorSum > 0){
      Serial.println("PHaT - 7");
      digitalWrite(A3, HIGH);
      delay(1000);
      digitalWrite(A3, LOW);
    } else {
      Serial.println("PHaT - 8");
      digitalWrite(A3, HIGH);
      delay(80);
      digitalWrite(A3, LOW);
      delay(80);
      digitalWrite(A3, HIGH);
      delay(80);
      digitalWrite(A3, LOW);
    }
  #endif

}

void loop() {
  #ifdef sd
    DataFile = SD.open(FileName, FILE_WRITE);
  #endif

  /////////// Loop for RTD sensor //////////////
  #ifdef RTD
    //Serial.println("RTD Start");
    float rtd = max.readRTD();
    float ratio = rtd;
    ratio /= 32768;
    Ext_Temperature = max.temperature(1000, RREF);

    //Serial.println(count);

    digitalWrite(A2, HIGH);
    DataFile.print(Ext_Temperature); //More accurate way to do this
    digitalWrite(A2, LOW);

  // loop for RH            // If battery lower than 6.5V the frequency starts to drop below 12KHz
                                       // For all other battery power the frequency is stable at 12KHz
  #ifdef RH
    //Serial.println("RH Start");
    count=0;
    digitalWrite(A0, HIGH);
    Serial.print("");  // This is needed before the delay for some reason. Not sure why, but if it's not there count is zero
    delay(count_delay);
    count_cap = count;
    digitalWrite(A0, LOW);
    //Serial.println(count_cap);
    //Serial.println(RH_const);
    //delay(1000);
    c_humid = RH_const/count_cap;
    RH_value = 30 + 0.909091 * (c_ref - c_humid);
    Change_RH = (B5*RH_value+B2)*Ext_Temperature + (B3*RH_value+B4);
    RH_value = RH_value + Change_RH;
    //Serial.println(c_humid);
    //Serial.println(RH_value);
    DataFile.print(", ");
    DataFile.print(count_cap);
    DataFile.print(", ");
    DataFile.print(RH_value); 

  #endif

  // Loop for Pressure Sensor
  #ifdef PS
    //Serial.println("PS Start");
    for(int i=0;i<2;i++){
      unsigned short a = 0x48+i*10;
      digitalWrite(5, LOW);
      delayMicroseconds(50);
      SPI.transfer(a);
      delay(9);
      digitalWrite(5, HIGH);
      delay(1);
      digitalWrite(5, LOW);
      delayMicroseconds(50);
//      for(int j=0;j<3;j++){
//        P_Data[i][j] = SPI.transfer(0x00);
//      }
      P_Data1 = SPI.transfer(0x00);
      P_Data2 = SPI.transfer(0x00);
      P_Data3 = SPI.transfer(0x00);
      delayMicroseconds(50);
      digitalWrite(5, HIGH);
      SPI.endTransaction();
      delay(1);
    }
    //Serial.print("1 : ");
    //Serial.println(P_Data1);
    //Serial.print("2 : ");
    //Serial.println(P_Data2);
    //Serial.print("3 : ");
    //Serial.println(P_Data3);
    DataFile.print(", ");
    DataFile.print("N/A");
    DataFile.print(", ");
    DataFile.print("N/A");
  #endif

  // Loop for Time of Sample
  #ifdef Time
    //Serial.println("Time Start");
    DataFile.print(", ");
    DataFile.print("N/A");
  #endif

    // Check and print any faults
    uint8_t fault = max.readFault();
    if (fault) {
      if (fault & MAX31865_FAULT_HIGHTHRESH) {
        Flag[0] = 1; // Serial.println("RTD High Threshold"); 
      }
      if (fault & MAX31865_FAULT_LOWTHRESH) {
        Flag[1] = 1; //Serial.println("RTD Low Threshold"); 
      }
      if (fault & MAX31865_FAULT_REFINLOW) {
        Flag[2] = 1; //Serial.println("REFIN- > 0.85 x Bias"); 
      }
      if (fault & MAX31865_FAULT_REFINHIGH) {
        Flag[3] = 1; //Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
      }
      if (fault & MAX31865_FAULT_RTDINLOW) {
        Flag[4] = 1; //Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
      }
      if (fault & MAX31865_FAULT_OVUV) {
        Flag[5] = 1; //Serial.println("Under/Over voltage"); 
      }
      max.clearFault();
    }
  #endif

  // Check Battery Voltage
  #ifdef V
    //Serial.println("Voltage Start");
    voltage = analogRead(A7) * calibrationFactor;     // Voltage can be read down to a battery power of 6.5 V
    pinvoltage = analogRead(A7) * 0.00488;
    digitalWrite(A1, HIGH);
    DataFile.print(", ");
    DataFile.print(voltage);
    DataFile.print(", ");
    DataFile.print(pinvoltage);
    digitalWrite(A1, LOW);
    if (voltage<6.5){
      Flag[6] = 1;
      digitalWrite(A3, HIGH);
      delay(80);
      digitalWrite(A3, LOW);
    }
  #endif
  //Serial.println(count);
  //Serial.println("Flag Start");
  DataFile.print(", ");
  for (int i=0 ; i<=15; i++){
    DataFile.print(Flag[i]);
  }

  //ahe
  //add GPS data

  if(gps_valid_data==1){
    DataFile.print(GPS_str);
    gps_valid_data = 0;
  }else{
    DataFile.print(",,");
  }

  //New line to end data
    DataFile.println("");
  ////////////
  #ifdef sd
    DataFile.close();
  #endif

  //ahe - i2c
  if(i2c_valid==1 & gps_valid_cmd==1){
      Serial.print("i2c data:");
      sprintf(i2c_file_data,"%s,%s",i2c_data,GPS_str);
      Serial.println(i2c_file_data);

      //dump the data to a file - add function calls to do that

      
      gps_valid_cmd = 0;
      i2c_valid = 0;
  }
}