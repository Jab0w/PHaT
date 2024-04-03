

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    //gps_data[serial_indx++] = inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
     inputString += inChar;
    serial_indx++;

    if ((inChar == '\n') | serial_indx > 40){
      stringComplete = true;
      serial_indx = 0;
      //gps_data[serial_indx++] = '\0';
      //serial_indx = 0;
      //Serial.flush();
    }
  }
}

/////////////////// UNUSED CODE - AHE /////////////////////


// from APRS code to extract data
void update_GPS_alt(char *p) {

  //data is sent with 2 decimal places
  // reformat the data to be used by the APRS library
  //"$GPGGA,191757.00,3938.28486,N,07957.13511,W,1,03,2.71,274.5,M,-33.9,M,,*6F";
  // v3
  // $GNGGA,165006.000,2241.9107,N,12017.2383,E,1,14,0.79,22.6,M,18.5,M,,*42

  // Documentation: https://openrtk.readthedocs.io/en/latest/communication_port/nmea.html
  //$GNGGA<0>,000520.095<1>,<2>,<3>,<4>,<5>,0<6>,0<7>,<8>,<9>,M<10>,<11>,M<12>,<13>,*5D<14>

  //char *p = strtok(test_data, ",");  //code - <0>
  p = strtok(NULL, ",");             //time - <1>


  p = strtok(NULL, ",");             //lat - <2>
  sprintf(Lat, "%s", p);

  // bug fix in v2
  if (Lat[4] == '.') {
    Lat[7] = '\0';
  } else {
    Lat[8] = '\0';
  }

  
  p = strtok(NULL, ",");  // lat_char - <3>
  sprintf(Lat, "%s%s\0", Lat, p);

  //p = strtok(NULL, ",");             //lng
  //p = strtok(NULL, ",");             //dir

  p = strtok(NULL, ",");  //lng - <4>
  sprintf(Lon, "%s", p);

  if (Lon[4] == '.') {
    Lon[7] = '\0';
  } else {
    Lon[8] = '\0';
  }

  p = strtok(NULL, ",");  //dir - <5>
  sprintf(Lon, "%s%s\0", Lon, p);

  p = strtok(NULL, ",");  //state - <6>
  //Serial.print("-");
  //Serial.println(p);

  if (p[0]>='1' & p[0]<='4') {
    ;
  }
  else{
    //Serial.println("[!] Invalid data");
    msg_valid = 0;
    return;
  }
  
  //aprs comment
  sprintf(alt,"v4.3,");
  
  p = strtok(NULL, ",");    //sta-no - <7>
  strcat(alt,p);
  strcat(alt,",");
  p = strtok(NULL, ",");    //horizontal 
  strcat(alt,p);
  strcat(alt,",alt=");
  p = strtok(NULL, ",");    //alti
  strcat(alt,p);
  p = strtok(NULL, ",");    //alti-unit
  strcat(alt,p);
  

  msg_valid = 1;

  
/*
  Serial.print(Lat);
  Serial.print(",");
  Serial.println(Lon);
*/
}