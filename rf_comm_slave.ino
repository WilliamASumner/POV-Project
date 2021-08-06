#include <SPI.h>
//#include <DigitalIO.h> // CHANGE RF24/RF24_config.h line 27: #define SOFTSPI line 28-31: define SPI pins
#include <Tlc5948.h>
#include <nRF24L01.h>
#include <RF24.h>
#ifdef ARDUINO_TEENSY40 // teensy 4.0
const int IRQ_PIN = 6;
const int CE_PIN = 8;
const int CSN_PIN = 9;
const int NOTEPIN1 = 3;
const int NOTEPIN2 = 4;
const int NOTEPIN3 = 5;
const int MAG_OUT = 14;
//const int SOFT_SPI_MISO_PIN = 12;
//const int SOFT_SPI_MOSI_PIN = 11;
//const int SOFT_SPI_SCK_PIN = 13;
//const int SOFT_SPI_MODE = 0;
#else
#error "Unimplemented"
const int CE_PIN = 9;
const int CSN_PIN = 10;
const int NOTEPIN1 = 3;
const int NOTEPIN2 = 5;
const int NOTEPIN3 = 6;
const int MAG_OUT = 3;
#endif

// ******* Note Processing *******

inline void noteSetup() {
  pinMode(NOTEPIN1,OUTPUT);
  pinMode(NOTEPIN2,OUTPUT);
  pinMode(NOTEPIN3,OUTPUT);

  analogWriteFrequency(3,8000000); // set default frequencies ( PWM_REG -> 120Hz, ES_PWM -> 15Khz)
  analogWriteFrequency(4,8000000);
  analogWriteFrequency(5,8000000);

  analogWrite(3,127);
  analogWrite(4,127);
  analogWrite(5,127);
}

inline void playFreq(int notePin, float noteFreq) {
  analogWriteFrequency(notePin,noteFreq * 512);
  analogWrite(notePin,127); // 50% waveform
}

inline void stopFreq(int notePin) {
  //analogWrite(notePin,0); // do nothing for testing
}

// *******  LED Display Setup  *******
const int NUM_TLCS = 3;
Tlc5948 tlc;
const uint8_t rowSize = 16;
const uint8_t numColors = 3;
const uint8_t ringSize = 1; // 60; // aka columns
// This is a big variable... check if we need this
uint16_t displayBuffer[ringSize][rowSize][3] = {0}; 

// Display refresh rate notes:
// Ideally want 60FPS for it to look smooth, probably not gonna happen
// According to Quora (still need to test): house fan 1300 RPM -> 21 RPS -> 20 FPS 
// 20 FPS means 0.05 seconds per frame(aka rotation) / 60 columns -> 0.000833 s per column
// according to benchmark (for only 10 LEDs not 16, and just writing, not ISR overhead) -> 75us -> 0.000075s
// this is 11x faster so we should be ok

//IntervalTimer displayTimer; // how we display at the 'right' time for each column

volatile bool writeDisplayData = false;
volatile unsigned int displayColumn = 0;

void updateDisplayIsr() {
  displayColumn = (displayColumn + 1) % ringSize; 
  writeDisplayData = true; 
}

uint16_t emptyBuffer[48] = { 0 };
uint16_t fullBuffer[48] = { 0xff00,0x0000,0xff00,0xff00,0x0000,0xff00,0xff00,0x0000,0xff00,
                            0xff00,0x0000,0xff00,0xff00,0x0000,0xff00,0xff00,0x0000,0xff00,
                            0xff00,0x0000,0xff00,0xff00,0x0000,0xff00,0xff00,0x0000,0xff00,
                            0xff00,0x0000,0xff00,0xff00,0x0000,0xff00,0xff00,0x0000,0xff00,
                            0xff00,0x0000,0xff00,0xff00,0x0000,0xff00,0xff00,0x0000,0xff00,
                            0xff00,0x0000,0xff00 };
inline void ledSetup() {
  //SPI.begin();
  tlc.begin(false);

  //tlc.writeGsBufferSPI16(emptyBuffer,3,3); // clear out the gs data (likely random)
  tlc.writeGsBuffer((uint8_t *)emptyBuffer,sizeof(emptyBuffer));
  tlc.setDcData(Channels::out1,0xff); // dot correction
  tlc.setBcData(0x7f); // global brightness 
  
  Fctrls fSave = tlc.getFctrlBits();
  fSave &= ~(Fctrls::dsprpt_mask);
  fSave |= Fctrls::dsprpt_mode_1; // set autodisplay repeat

  fSave &= ~(Fctrls::espwm_mask);
  fSave |= Fctrls::espwm_mode_1; // set ES PWM mode on, basically breaks up
                                   // long ON/OFF periods into 128 smaller segments
                                   // with even distribution
  tlc.setFctrlBits(fSave);
  tlc.writeControlBuffer(NUM_TLCS);

  unsigned int timerDurationUs = 2000000 / ringSize; // 5Hz / 60 cols = 3333 us / col
                                                      //\ This will get updated by hall-effect
//  displayTimer.begin(updateDisplayIsr,timerDurationUs); // start timer with a guess-timate of column time
}

// ******* Hall-Effect (Magnet Sensor) Setup *******
// Estimated rotation of about 5 rotations per s ~> 0.2 s or 200 us / rotation
// TODO write RF24-comm for reading RPM
volatile bool magnetChanged = false;
volatile uint32_t prevTime = 0;
void hallIsr() {
  // Create a timer with a duration that will give 60 even segments using the time
  // Because a rotation likely won't go over 1s and micros() overflows at 1hr, we should be ok
  unsigned int newDuration = (micros() - prevTime) / ringSize; // convert elapsed time to col time
//  displayTimer.update(newDuration); // new displayTimer duration
  displayColumn = 0; // reset to beginning of display
  //magnetChanged = true;
  prevTime = micros(); // reset elapsed time
}

inline void magSetup() {
  pinMode(MAG_OUT,INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(MAG_OUT),hallIsr,RISING); // TODO check if there's a better way to do this? VVV
  // A couple options: change VREEF for RISING (check manual)
  //                   switch to polling with ADC (would that really be better though?) 
  //                       \ Add Kalman filter to ADC data and trigger on threshold 
}

// ******* Keyboard Processing *******

/* Keys are defined as:
       Col0    Col1
      [ 0 ]   [ 1 ]  // Row 0  \___ Note Group 1
      [ 2 ]   [ 3 ]  // Row 1  /
      -------------
      [ 4 ]   [ 5 ]  ... \_________ Note Group 2
      [ 6 ]   [ 7 ]  ... / 
      -------------
      [ 8 ]   [ 9 ]  ...      \____ Note Group 3
      [ 10 ]  [ 11 ] // Row 5 /
      
*/

// Current frequencies are just two strings an octave apart
const float keysToFreq[12] = {164.81,329.63,174.61,349.23,
                              185.00,369.99,196.00,392.00,
                              207.65,415.30,220.00,440.00};
const uint8_t keysToPin[12] = {NOTEPIN1,NOTEPIN1,NOTEPIN1,NOTEPIN1,
                               NOTEPIN2,NOTEPIN2,NOTEPIN2,NOTEPIN2,
                               NOTEPIN3,NOTEPIN3,NOTEPIN3,NOTEPIN3};
const uint8_t keysToGroup[12] = {0,0,0,0,1,1,1,1,2,2,2,2};
bool groupPlaying[3] = {false, false, false};
uint16_t prevKeys = 0x0; // holder for previous value of keys

void processKeys(uint16_t keys) {
  if (prevKeys == keys) {
    return; // nothing to do
  }

  uint16_t diff = prevKeys ^ keys;
  uint16_t keysTemp = keys;

  for (int i = 0; i < 12; i++) {
    if (diff & 0x1) { // if there's a difference
      uint8_t pinNum = keysToPin[i];
      if ((keysTemp & 0x1) && !groupPlaying[keysToGroup[i]]) { // if a note is on and no other note in group playing, turn on note
        playFreq(pinNum,keysToFreq[i]);
      } else if (!(keysTemp & 0x1)) { // turn off note
        stopFreq(pinNum);
      }
    }
    keysTemp >>= 1;
    diff >>= 1;
  }
  prevKeys = keys; 
}

// Ring-Buffer and Functions for storing keypresses
// [keypress1] [keypress2] [keypress3]** [keypress4]
// Basically adds 
const int keyBufferSize = 16; // holds 16 key presses/releases
volatile int insertIndex = 0;
volatile int readIndex = 0; // TODO fix this
uint16_t keyBuffer[keyBufferSize] = { 0 };

inline void addKey(uint16_t key) {
  keyBuffer[insertIndex] = key;
  insertIndex = (insertIndex + 1 ) % keyBufferSize;
}


// ******* Radio Setup *******

RF24 radio(CE_PIN, CSN_PIN,SPI_SPEED);
const uint64_t pipe = 0xB0B15ADEADBEEFAA;
uint16_t keyData = 0x0;

inline void radioSetup() {
  radio.begin();
  radio.openReadingPipe(1, pipe);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate( RF24_250KBPS );
  radio.startListening();

  delay(500);
  if (radio.isChipConnected()) {
    Serial.println("Radio is connected");
    radio.printPrettyDetails();
  } else {
    Serial.println("Error, radio not available");
  }
}




// *******  Animation  *******

inline void animationSetup() {
  for (int i = 0; i < ringSize; i++) {
    for (int j = 0; j < rowSize; j++) {
      displayBuffer[i][j][0] = 0x0;
      displayBuffer[i][j][1] = 0x0;
      displayBuffer[i][j][2] = 0x0;
    }
  }
}

inline void animationUpdate() {
  static uint16_t brightnessVal = 0x0;
  for (int i = 0; i < ringSize; i++) {
    uint16_t prevKeysTemp = prevKeys;
     for (int j = 0; j < rowSize; j++) {
      
      displayBuffer[i][j][0] = brightnessVal * (prevKeysTemp & 0x1);
      displayBuffer[i][j][1] = brightnessVal * (prevKeysTemp & 0x1);
      displayBuffer[i][j][2] = brightnessVal * (prevKeysTemp & 0x1);
      prevKeysTemp >>= 1;
    }
  }
  brightnessVal += 0xff;
}


void setup() {
  Serial.begin(9600);
//  delay(1000);
  noteSetup();
  radioSetup();
  ledSetup();
  delay(500);

  //tlc.fillGsBuffer(32 * NUM_TLCS,0x08); // test blink full brightness
  //tlc.writeGsBufferSPI16(fullBuffer,3,3); // clear out the gs data (likely random)
  tlc.writeGsBuffer((uint8_t*)fullBuffer,sizeof(fullBuffer)); // clear out the gs data (likely random)
  delay(1000);
  tlc.writeGsBuffer((uint8_t*)emptyBuffer,sizeof(emptyBuffer)); // clear out the gs data (likely random)
  //magSetup(); 
  animationSetup();
  Serial.println("finished setup");
}

void loop() {
//  if (writeDisplayData) { // if time to display data
//    Serial.println("Writing display data");
//    tlc.writeGsBuffer((uint8_t*)displayBuffer[displayColumn],sizeof(displayBuffer[0]));
//    writeDisplayData = false;
//  } else 
  if ( radio.available()) { // check user input
    Serial.println("Received data!");
    radio.read(&keyData, sizeof(keyData));
    processKeys(keyData);
    Serial.print("Keydata: ");
    Serial.println(keyData);
    if (keyData & 0x1) {
      Serial.println("blinking!");
      tlc.writeGsBuffer((uint8_t*)fullBuffer,sizeof(fullBuffer)); // clear out the gs data (likely random)
      delay(1000);
      tlc.writeGsBuffer((uint8_t*)emptyBuffer,sizeof(emptyBuffer)); // clear out the gs data (likely random)
    }
  } else {
    //animationUpdate();
  }
//  
//  else if (true) { // new frame on magnetChanged
//    Serial.println("Updating animation");
//    animationUpdate();
//    //magnetChanged = false;
//  }
}
