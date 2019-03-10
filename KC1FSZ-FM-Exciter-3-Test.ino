// Control code for FM Exciter 3.0
// Bruce MacKinnn KC1FSZ
// 25 February 2019
//
#include <SPI.h>
#include <Wire.h>

// The Etherkit library
//#include <si5351.h>

#define PIN_AD9834_SCLK 2
#define PIN_AD9834_SDATA 3
#define PIN_AD9834_FSYNC 4
#define PIN_AD9834_RESET 5
#define PIN_AD9834_SIGN_BIT_OUT 6
#define PIN_LED13 13 
#define MASK_LSB12 0b0000111111111111
#define MASK_LSB14 0b0011111111111111
#define MASK_BIT15 0b1000000000000000

//Si5351 si5351;

// Desired center frequency
unsigned long vfo = 0;

void clkStrobeAD9834() {
  digitalWrite(PIN_AD9834_SCLK,0);
  digitalWrite(PIN_AD9834_SCLK,1);
}

void writeBitAD9834(bool bit) {
    digitalWrite(PIN_AD9834_SDATA,(bit) ? 1 : 0);
    clkStrobeAD9834();
}

// This function writes a complete 16-bit word into the AD9834,
// MSB first!
void writeAD9834(unsigned int w) {
  // Clock the data bits into the shift register
  for (unsigned int i = 0; i < 16; i++) {
    // VERY IMPORTANT: THIS NEEDS TO BE BIT-WISE AND!
    writeBitAD9834((w & MASK_BIT15) != 0);
    // Rotate to the left one to expose the next least significant bit
    w = w << 1;
  }
}

void setAD9834FSYNC(bool b) {
    digitalWrite(PIN_AD9834_FSYNC,(b) ? 1 : 0);  
}

void writeAD9834PHASE(int reg,unsigned long p) {
  
  unsigned long out = 0;
  unsigned long registerAddress = 0;

  // Lower FSYNC to start the write (similar to CS)
  setAD9834FSYNC(false);

  // Write the address 
  if (reg == 0) {
    registerAddress = 0b1100;
  } else {
    registerAddress = 0b1110;
  }
  out = (registerAddress << 12);
  // Write the data
  out |= (p & MASK_LSB12);
  
  writeAD9834(out);

  // Raise FSYNC to end the write
  setAD9834FSYNC(true);  
}

void writeAD9834FREQ28(int reg,unsigned long f) {

  unsigned long out = 0;
  unsigned long registerAddress = 0;

  setAD9834FSYNC(false);

  // Write the address 
  if (reg == 0) {
    registerAddress = 0b01;
  } else {
    registerAddress = 0b10;
  }
  out = (registerAddress << 14);

  // Write the 14 LSBs of the register
  out |= (f & MASK_LSB14);
  writeAD9834(out);

  // Write the address again 
  out = (registerAddress << 14);
  
  // Write the 14 MSBs of the data
  out |= ((f >> 14) & MASK_LSB14);
  writeAD9834(out);

  setAD9834FSYNC(true);
}

void writeAD9834Control(int resetFlag) {
  
  unsigned long out = 0;
  unsigned long a = 0;

  a = 0b00; // [Control register address, 2 bits]
  out |= (a << 14);
  a = 0b01; // [B28=1, frequency loaded as two consecutive writes]]
  out |= (a << 13);
  a = 0b0; // [HLB not needed]
  out |= (a << 12);
  a = 0b0; // [FSEL=FREQ0]
  out |= (a << 11);
  a = 0b0; // [PSEL=PHASE0]
  out |= (a << 10);
  a = 0b0; // [PINSW=SW]
  out |= (a << 9);
  a = resetFlag & 0b1; // [RESET]
  out |= (a << 8);
  a = 0b0; // [SLEEP1=MCLK enabled]
  out |= (a << 7);
  a = 0b0; // [SLEEP12=DAC enabled]
  out |= (a << 6);
  a = 0b1; // [OPBITIN=ENABLED]
  out |= (a << 5);
  a = 0b0; // [SIGN/PIB=MSB of DAC]
  out |= (a << 4);
  a = 0b0; // [DIV2=output/2]
  out |= (a << 3);
  a = 0b0; // [Reserved=0]
  out |= (a << 2);
  a = 0b0; // [MODE=SIN ROM Used]
  out |= (a << 1);
  a = 0b0; // [Reserved=0]
  out |= a;
  
  setAD9834FSYNC(false);
  writeAD9834(out);
  setAD9834FSYNC(true);
}

void setFreq(int reg,unsigned long freqHz) {
  // Do the math to compute the register value
  double master = 75000000L;
  double range = 268435456;
  double freqReg = (double)freqHz * (range / master);
  Serial.print("Setting FREQ0: ");
  Serial.print(freqReg);
  writeAD9834FREQ28(reg,(unsigned long)freqReg);
}

void flash() {
  digitalWrite(PIN_LED13,1);  
  delay(250);
  digitalWrite(PIN_LED13,0);  
  delay(250);  
}

long lastMaintStamp = millis();
long maintInterval = 5000;

void setup() {

  pinMode(PIN_LED13,OUTPUT);
  pinMode(PIN_AD9834_SCLK,OUTPUT);
  pinMode(PIN_AD9834_SDATA,OUTPUT);
  pinMode(PIN_AD9834_FSYNC,OUTPUT);
  pinMode(PIN_AD9834_RESET,OUTPUT);
  pinMode(PIN_AD9834_SIGN_BIT_OUT,INPUT);

  digitalWrite(PIN_LED13,0);
  digitalWrite(PIN_AD9834_SCLK,1);
  digitalWrite(PIN_AD9834_SDATA,0);
  digitalWrite(PIN_AD9834_FSYNC,1);
  digitalWrite(PIN_AD9834_RESET,0);

  // Hello world check
  flash();
  flash();
  
  Serial.begin(9600);
  Serial.println("KC1FSZ FM Exciter Controller 3");
  delay(1000);

  // Si5351 initialization
  //si5351.init(SI5351_CRYSTAL_LOAD_8PF,0,0);
  // Boost up drive strength
  //si5351.drive_strength(SI5351_CLK0,SI5351_DRIVE_8MA);

  // AD9834 self-test
  //delay(1000);

  // Strobe reset pin
  digitalWrite(PIN_AD9834_RESET,1);
  digitalWrite(PIN_AD9834_RESET,0);

  // Set RESET off
  writeAD9834Control(0);
  writeAD9834FREQ28(0,0);
  //writeAD9834PHASE(0,0);
  // Setup a test frequency (very low)
  //writeAD9834FREQ28(0,0xfffff);  
  // Set RESET off, start things going
  //writeAD9834Control(0);
  setFreq(0,10700000);
}

int count = 0;

void loop() {
  /*
  // Periodic maintenance like displays, etc.
  long ms = millis();
  if (ms < lastMaintStamp || (ms - lastMaintStamp) > maintInterval) {  
    lastMaintStamp = ms;  
    // Alernate the frequency between two values
    if (count % 2 == 0) {
      writeAD9834FREQ(0,1);  
    } else {
      writeAD9834FREQ(0,2);  
    }
    count++;
  }
  */
}

