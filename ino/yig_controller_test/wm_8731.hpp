class Wm8731{
  public:
    int m_ssPin;
    Wm8731(int ssPin){
      m_ssPin=ssPin;
      pinMode(m_ssPin, OUTPUT);
      digitalWrite(m_ssPin, LOW);
      SPI.begin();
      SPI.setBitOrder(MSBFIRST);
      SPI.setClockDivider(SPI_CLOCK_DIV2);
      SPI.setDataMode(SPI_MODE0);
      Wire.begin(); //initialize wire library
      init();   
    }

    void init(){
      //Serial.println("OLA!");
      this->softReset(); //Reset Wm8731
      //delay(100);
      //Initialize WM8731 registers

      writeRegister(0x6, 0); //Disable power down
      writeRegister(0x0, 23); //left line in
      writeRegister(0x1, 23); //right line in
      writeRegister(0x2, 121);
      writeRegister(0x3, 121);
      writeRegister(0x4, 0b11010); //Disable mic boost and mute
      writeRegister(0x5, 0);
      writeRegister(0x7, 3);
      writeRegister(0x8, 0b00001110);
      writeRegister(0x8, 0b00000001);
      
    }

    void softReset(){
      writeRegister(0xF, 0x0);
    }
    
    void writeRegister(uint8_t addr, uint16_t data){
      uint16_t w=addr<<9|data&0x1FF;
      digitalWrite(m_ssPin, LOW);
      Wire.beginTransmission(0x1a);
      Wire.write((uint8_t)(w>>8));
      Wire.write((uint8_t)(w&0xFF));
      Wire.endTransmission();
    }

    void writeLeft(int data){
      digitalWrite(m_ssPin, HIGH);
      writeData(data);
    }

    void writeRight(int data){
      digitalWrite(m_ssPin, LOW);
      writeData(data);
    }

    void writeData(int data){
      //PORTB |= (1<<PORTB2);  // toggle ss pin // PORTB2 IS PIN 10
      asm volatile ("out %0, %B1" : : "I" (_SFR_IO_ADDR(SPDR)), "r" (data) );
      while(!(SPSR & (1<<SPIF))){ // wait for data transfer to complete
      }
      
      asm volatile ("out %0, %A1" : : "I" (_SFR_IO_ADDR(SPDR)), "r" (data) );
      asm volatile ("in r3, %0" : : "I" (_SFR_IO_ADDR(SPDR)) );
      while(!(SPSR & (1<<SPIF))){ // wait for data transfer to complete
      }
    }
   
};
