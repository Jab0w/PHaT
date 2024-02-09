
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