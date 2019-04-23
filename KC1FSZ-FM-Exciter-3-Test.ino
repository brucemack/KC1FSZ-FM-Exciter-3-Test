// Control code for FM Exciter 3.0
// Bruce MacKinnn KC1FSZ
// 25 February 2019
//
#include <SPI.h>
#include <Wire.h>

// The Etherkit library
#include <si5351.h>

#define PIN_AD9834_SCLK 2
#define PIN_AD9834_SDATA 3
#define PIN_AD9834_FSYNC 4
#define PIN_AD9834_RESET 5
#define PIN_AD9834_SIGN_BIT_OUT 6
#define PIN_PTT_IN 7

#define PIN_LED13 13 
#define MASK_LSB12 0b0000111111111111
#define MASK_LSB14 0b0011111111111111
#define MASK_BIT15 0b1000000000000000

Si5351 si5351;

// Desired center frequency
//const unsigned long centerF = 10700000L;
const unsigned long centerF = 19000000L;
// FM deviation (Hz)
unsigned long devF = 5000; 
// Baseband sampling frequency 
const unsigned long sampleFreq = 10000L;
// The number of microseconds between baseband samples
const unsigned long sampleIntervalUs = 1000000L / sampleFreq;
// Baseband PL tone
unsigned long plFreq = 123L;
// The 1Hz phase accumulator for the PL tone.  This will count from 0 to sampleFreq
// at a speed determined by the PL freq. 
unsigned long plPhaseAccumulator = 0; 
// Baseband CW tone
unsigned long cwFreq = 1000L;
// The 1Hz phase accumulator for the CW tone.  This will count from 0 to sampleFreq
// at a speed determined by the CW freq. 
unsigned long cwPhaseAccumulator = 0; 
// Current state of the PTT control
bool pttState = false;

// PL mix (from 0.0 to 1.0)
/* From https://forums.qrz.com/index.php?threads/another-question-about-pl-tones.300214/

With modern, solid-state, decoders the CTCSS level can run at well under 5 percent of the total deviation, 
that is, +/- .25 kHz (250 Hz) deviation with maximum audio deviation of +/- 5 kHz. I don't like to run the 
level above +/- .5 kHz (500 Hz) deviation especially with the higher frequency tones. Once you get above 
around 175 Hz for the tone many decoders will not filter out the tone that well and the signal will have 
a "hum" on it.

Even the old Motorola TU-333 "Vibrasponder" reeds worked fine with +/- 500 Hz deviation and I have seen a 
lot of solid-state decoders work at as low a deviation as +/- 100 Hz. Even back as far as 1970, when I owned 
the Motorola reconditioned equipment center for the south-central United States, we generally set the Private 
Line (this was "true" Private Line because that is the Motorola trademark for CTCSS) at +/- 500 Hz. In over 
9 years we had absolutely no complaints about too little tone.
 */
float plMix = 0.10;

unsigned long maintIntervalMs = 1000;
unsigned long maintLastMs = 0;
unsigned long sampleStatsL0,sampleStatsL1,sampleStatsH0,sampleStatsH1;

const float master = 75000000L;
const float range = 268435456; // 2^28

unsigned long lastSampleStamp = 0;
unsigned long lastSampleCycle = 0;
unsigned long lastSampleLength = 0;

float lastSample = 0;
// Used for the pre-empasis filter
float filterCoefficient0 = -0.53;

bool toneEnabled = false;
bool preEmphasisFilter = false;
bool maintEnabled = false;

IntervalTimer timer0;

// This is the number of phase buckets we manage.  More buckets means 
// a smoother transition through the baseband signals.
const unsigned int phaseRange = 16 * 16;
// The LUT only has to cover 90 degrees of the phase range because 
// we have quadrant translation.
const unsigned int lutSize = phaseRange / 4;
// This is the LUT (filled during setup())
float sinLut[lutSize];

unsigned int counter = 0;

// ----- I/O functions --------------------------------------------------

void setupAD9834Pins() {  
  pinMode(PIN_AD9834_SCLK,OUTPUT);          // 2
  pinMode(PIN_AD9834_SDATA,OUTPUT);         // 3
  pinMode(PIN_AD9834_FSYNC,OUTPUT);         // 4 
  pinMode(PIN_AD9834_RESET,OUTPUT);         // 5
  pinMode(PIN_AD9834_SIGN_BIT_OUT,INPUT);   // 6
  digitalWrite(PIN_AD9834_SCLK,1);  // 2
  digitalWrite(PIN_AD9834_SDATA,0); // 3
  digitalWrite(PIN_AD9834_FSYNC,1); // 4
  digitalWrite(PIN_AD9834_RESET,0); // 5
}

// PIN 2 = PTD0
void clkStrobeAD9834() {
  digitalWriteFast(2,0);
  digitalWriteFast(2,1);
}

// PIN 5 = PTD7
void resetStrobeAD9834() {
  digitalWriteFast(5,1);
  digitalWriteFast(5,0);
}

void writeBitAD9834(bool bit) {
  if (bit) {
    digitalWriteFast(3,1);
  } else {
    digitalWriteFast(3,0);
  }
  clkStrobeAD9834();
}

void setAD9834FSYNC(bool bit) {
  if (bit) {
    digitalWriteFast(4,1);
  } else {
    digitalWriteFast(4,0);
  }
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

void writeAD9834Control(int resetBit,int fselBit,int pselBit) {
  
  unsigned long out = 0;
  unsigned long a = 0;

  a = 0b00; // [Control register address, 2 bits]
  out |= (a << 14);
  a = 0b01; // [B28=1, frequency loaded as two consecutive writes]]
  out |= (a << 13);
  a = 0b0; // [HLB not needed]
  out |= (a << 12);
  //a = 0b0; // [FSEL=FREQ0]
  a = fselBit & 1;
  out |= (a << 11);
  //a = 0b0; // [PSEL=PHASE0]
  a = pselBit & 1;
  out |= (a << 10);
  a = 0b0; // [PINSW=SW]
  out |= (a << 9);
  a = resetBit & 1; // [RESET]
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
  float freqReg = (float)freqHz * (range / master);  
  writeAD9834FREQ28(reg,(unsigned long)freqReg);
}

void flash() {
  digitalWrite(PIN_LED13,1);  
  delay(250);
  digitalWrite(PIN_LED13,0);  
  delay(250);  
}

// This function takes an integer phase and returns the sin() value.  Quadrant
// translation is used to save memory.
//
// ph - Integer phase from 0 to range-1
// lut - Pointer to a look-up-table that has range / 4 entries from 0 to pi/2 radians 
//   or 0 to 90 degrees.
// range - Total size of the phase range.  Should be a mutiple of 4!
//
float sinWithQuadrant(unsigned int ph,float* lut,unsigned int range) {
  // Figure out which quandrant we're in and adjust accordingly
  unsigned int quadrant = ph / lutSize;
  // There are some special cases here
  if (ph == lutSize) {
    return 1.0;
  } else if (ph == lutSize * 3) {
    return -1;
  } else if (quadrant == 0) {
    return lut[ph];
  } else if (quadrant == 1) {
    return lut[range / 2 - ph];
  } else if (quadrant == 2) {
    return -lut[ph - range / 2];
  } else {
    return -lut[range - ph];
  }
}

void doSample() {

  // Timing diagnostics
  long now = micros();
  lastSampleCycle = (now - lastSampleStamp);
  lastSampleStamp = now;
  
  int rawSample = analogRead(A1);
  
  // Capture some stats
  if (rawSample == 0) {
    sampleStatsL0++;
  } else if (rawSample == 4095) {
    sampleStatsH0++;
  } else if (rawSample > 2048 + 100) {
    sampleStatsH1++;
  } else if (rawSample < 2048 - 100) {
    sampleStatsL1++;
  }
  // Scale the sample into a -1.0 -> 1.0 space
  float audioAmp = 2.0 * ((float)rawSample / 4096.0) - 1.0;  
  if (preEmphasisFilter) {
    audioAmp = filterCoefficient0 * lastSample + audioAmp;
  }
  lastSample = audioAmp;
  
  // Accumulate phase and wrap the counters as needed
  plPhaseAccumulator += plFreq;
  if (plPhaseAccumulator >= sampleFreq) {
    plPhaseAccumulator -= sampleFreq;
  }
  // Scale the phase accumulators to the size of the LUT phase range.  Note
  // that we mutiply before dividing to avoid precision issues
  unsigned long plPhaseScaled = (plPhaseAccumulator * phaseRange) / sampleFreq;
  // Do the trig to get the amplitudes (-1 to +1)
  float plAmp = sinWithQuadrant(plPhaseScaled,sinLut,phaseRange);

  // Mix the two signals and modulate the max deviation.  Do FM!
  float mixedAudio;
  if (toneEnabled) {
    // Accumulate phase and wrap the counters as needed
    cwPhaseAccumulator += cwFreq;
    if (cwPhaseAccumulator >= sampleFreq) {
      cwPhaseAccumulator -= sampleFreq;
    }
    // Scale the phase accumulators to the size of the LUT phase range.  Note
    // that we mutiply before dividing to avoid precision issues
    unsigned long cwPhaseScaled = (cwPhaseAccumulator * phaseRange) / sampleFreq;
    // Do the trig to get the amplitudes (-1 to +1)
    float cwAmp = sinWithQuadrant(cwPhaseScaled,sinLut,phaseRange);
    mixedAudio =  (plMix * plAmp) + cwAmp;
  } else {
    mixedAudio = (plMix * plAmp) + audioAmp;
  }
   
  float deltaF = mixedAudio * (float)devF;
  long targetF = (long)centerF + (long)deltaF;

  // We alternate back and forth between FREQ0 and FREQ1 per the 
  // AD9834 documentation on page 21.
  if (++counter % 2 == 0) {
    setFreq(0,(unsigned long)targetF);
    // Switch registers
    writeAD9834Control(0,0,0);
  } else {
    setFreq(1,(unsigned long)targetF);
    // Switch registers
    writeAD9834Control(0,1,0);    
  }

  // Timing diagnostic counters
  lastSampleLength = micros() - now;
}

void doMaint() {
  Serial.print(lastSampleCycle);
  Serial.print(" ");
  Serial.print(lastSampleLength);
  Serial.print(" ");
  Serial.print(lastSample);
  Serial.print(" ");
  Serial.print(sampleStatsL0);
  Serial.print(" ");
  Serial.print(sampleStatsL1);
  Serial.print(" ");
  Serial.print(sampleStatsH1);
  Serial.print(" ");
  Serial.print(sampleStatsH0);
  Serial.println();
  sampleStatsL0 = 0;
  sampleStatsL1 = 0;
  sampleStatsH1 = 0;
  sampleStatsH0 = 0;
}

void pollMaint() {
  unsigned long m = millis();
  if (m < maintLastMs || m > (maintLastMs + maintIntervalMs)) {
    maintLastMs = m;
    doMaint();
  }
}

void printSi5351Status() {
  // Get status
  si5351.update_status();
  delay(250);
  si5351.update_status();
  delay(250);
  Serial.print("Current status - SYS_INIT: ");
  Serial.print(si5351.dev_status.SYS_INIT);
  Serial.print("  LOL_A: ");
  Serial.print(si5351.dev_status.LOL_A);
  Serial.print("  LOL_B: ");
  Serial.print(si5351.dev_status.LOL_B);
  Serial.print("  LOS: ");
  Serial.print(si5351.dev_status.LOS);
  Serial.print("  REVID: ");
  Serial.println(si5351.dev_status.REVID);
}

void setup() {
  
  pinMode(PIN_LED13,OUTPUT);
  digitalWrite(PIN_LED13,0);
  pinMode(PIN_PTT_IN,INPUT_PULLUP);

  setupAD9834Pins();

  // Hello world check
  flash();
  flash();
  
  Serial.begin(9600);
  Serial.println("KC1FSZ FM Exciter Controller 3");
  delay(100);

  // Si5351 initialization
  si5351.init(SI5351_CRYSTAL_LOAD_0PF,0,0);
  // 147,668,000 was observed when targeting 147,630,000
  si5351.set_correction(278000,SI5351_PLL_INPUT_XO);

  printSi5351Status();

  // Reset the DDS
  resetStrobeAD9834();

  // Set RESET off
  writeAD9834Control(0,0,0);
  
  setFreq(0,centerF);

  timer0.begin(doSample,sampleIntervalUs);

  analogReadResolution(12);

  // Build the look-up table.  This will only cover the first 90 
  // degrees of the sin() function.
  for (unsigned int i = 0; i < lutSize; i++) {
    float rad = ((float)i / (float)phaseRange) * 2.0 * 3.1415926;
    sinLut[i] = sin(rad);
  }  

  // Start the LO  
  unsigned long outF = 147630000;
  unsigned long long f = outF - centerF;
  
  Serial.print("LO freq ");
  Serial.println((unsigned long)f);

  // MOVED FROM CLK2 to CLK0!
  si5351.output_enable(SI5351_CLK0,0);
  si5351.output_enable(SI5351_CLK1,0);  
  si5351.output_enable(SI5351_CLK2,0);
  
  si5351.set_freq((unsigned long long)f * 100ULL,SI5351_CLK0);
  si5351.drive_strength(SI5351_CLK0,SI5351_DRIVE_8MA);
  si5351.output_enable(SI5351_CLK0,0);
}

void loop() {

  // Look at commands 
  if (Serial.available() > 0) {
    // Read the incoming byte:
    char ib = Serial.read();
    if (ib == '-') {
      devF -= 100;
    } else if (ib == '=') {
      devF += 100;
    } else if (ib == '[') {
      plMix -= 0.1;
    } else if (ib == ']') {
      plMix += 1.0;
    } else if (ib == 'm') {
      maintEnabled = !maintEnabled;
    } else if (ib == 'p') {
      preEmphasisFilter = !preEmphasisFilter;
    }
    
    if (plMix < 0) {
      plMix = 0.0;
    } else if (plMix > 1.0) {
      plMix = 1.0;
    } 
    if (devF < 0) {
      devF = 0;
    }
    
    Serial.print("devF=");
    Serial.print(devF);
    Serial.print(", plMix=");
    Serial.print(plMix);
    Serial.print(", preEmph=");
    Serial.print(preEmphasisFilter);
    Serial.println();
  }

  if (maintEnabled)
    pollMaint();

  // Deal with the push-to-talk pin.
  //
  int pttInput = digitalReadFast(PIN_PTT_IN);
  if (pttInput == 0 && pttState == false) {
    si5351.output_enable(SI5351_CLK0,1);
    digitalWriteFast(13,1);
    pttState = true;
  } else if (pttInput == 1 && pttState == true) {
    si5351.output_enable(SI5351_CLK0,0);
    digitalWrite(13,0);
    pttState = false;
  }
}
