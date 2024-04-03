

// I2C recieve event



void receiveEvent(int howMany)
{
  int x = Wire.read();    // receive byte as an integer
  #ifdef ahe_debug
    Serial.print("r:");
    Serial.println(x);
  #endif

  if(x==12){
    i2c_cnt = 0;
    i2c_valid = 1;
    i2c_data[i2c_cnt] = '\0';
    #ifdef ahe_debug
      Serial.println("i2c_nl");
    #endif
  }  
  if (i2c_cnt==20){
    i2c_valid = 1;
    i2c_cnt = 0;
    #ifdef ahe_debug
      Serial.println("i2c_cmd_over");
    #endif
  }
  
  i2c_data[i2c_cnt] = x+64;
  i2c_cnt++;

}



