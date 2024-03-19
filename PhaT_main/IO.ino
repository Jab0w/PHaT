void init_io() {

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  //pinMode(5, OUTPUT);

  //555 trigger pin
  count = 0;
  pinMode(Trigger_PIN, INPUT);
  attachPCINT(digitalPinToPCINT(Trigger_PIN), timer_function, RISING);
  
  //Buzzer
  pinMode(A3, OUTPUT);

  //Pressure

  //RH
}

int SPI_PT_WRITE(int PCommand){

  short Output;
  SPI.beginTransaction(SPISettings(12000, MSBFIRST, SPI_MODE0));
  digitalWrite(5, LOW);
  Output = SPI.transfer(PCommand);
  //digitalWrite(5, HIGH);
  //SPI.endTransaction();

  return Output;
}
void timer_function(){
    //Serial.println(">");
    count++;
}

unsigned char crc4(unsigned int n_prom[]) { 
  int cnt;      // simple counter
  unsigned int n_rem;     // crc reminder
  unsigned int crc_read;    // original value of the crc
  unsigned char
  n_bit;  n_rem = 0x00;
  crc_read=n_prom[7];     //save read CRC 
  n_prom[7]=(0xFF00 & (n_prom[7]));   //CRC byte is replaced by 0
  for (cnt = 0; cnt < 16; cnt++)      // operation is performed on bytes
  { // choose LSB or MSB
    if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
    else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
    for (n_bit = 8; n_bit > 0; n_bit--)
    {               
      if (n_rem & (0x8000))
      {    
        n_rem = (n_rem << 1) ^ 0x3000; 
      }
      else  
      { 
        n_rem = (n_rem << 1); 
      }         
    }      
  }  
n_rem=  (0x000F & (n_rem >> 12)); // // final 4-bit reminder is CRC code  
n_prom[7]=crc_read;   // restore the crc_read to its original place      
return (n_rem ^ 0x00); 
}

unsigned int SumArray(int Error[]) {
  int sum = 0;
  for (int i=0; i<=15; i++){
    Serial.print(Error[i]);
    sum = sum + Error[i];
  }
  return sum; 
}

void I2C_RxHandler(int numBytes)
{
  RxByte = Wire.read();
  if (RxByte=28) {
    DataFile = SD.open("PHaTData.txt");
    while(Wire.available()) {  // Read Any Received Data
      RxByte = Wire.read();
    }
  } else if (RxByte=61) {
    APRSFile = SD.open("APRSData.txt");
    while(Wire.available()) {  // Read Any Received Data
      RxByte = Wire.read();
    }
  } else{
  }
}

void I2C_TxHandler(void)
{
  Wire.write(TxByte);
}