#include "Arduino.h"
#include "Dshot.h"

const FREQUENCY frequency = F500;

// Set inverted to true if you want bi-directional DShot
#define inverted true

// Enable Extended Dshot Telemetry
bool enableEdt = false;

// DSHOT Output pin
const uint8_t m1 = 22;
const uint8_t m2 = 23;
const uint8_t m3 = 24;
const uint8_t m4 = 25;
const uint8_t m5 = 26;
const uint8_t m6 = 27;
const uint8_t m7 = 28;
const uint8_t m8 = 29;
const uint8_t m9 = 30;

// DSHOT Output port mask
const uint8_t m1mask = B10000000;
const uint8_t m2mask = B01000000;
const uint8_t m3mask = B00100000;
const uint8_t m4mask = B00010000;
const uint8_t m5mask = B00001000;
const uint8_t m6mask = B00000100;
const uint8_t m7mask = B00000010;
const uint8_t m8mask = B00000001;
const uint8_t m9mask = B10000000;

const uint16_t NOmask = B00000000;

#define debug true
#define laptop false

// decare variables used by Mary for laptop reader
int inByte = 0;         // incoming serial byte
String temp = "";
bool nextByte = true;

// Port definitions for Port D


/* Initialization */
uint32_t dshotResponse = 0;
uint32_t dshotResponseLast = 0;
uint16_t mappedLast = 0;

// Buffer for counting duration between falling and rising edges
const uint8_t buffSize = 20;
uint16_t counter[buffSize];

// Statistics for success rate
uint16_t receivedPackets = 0;
uint16_t successPackets = 0;

Dshot dshot = new Dshot(inverted);
volatile uint16_t frame = dshot.buildFrame(0, 0);
volatile uint16_t request = dshot.buildFrame(0, 1);

volatile uint8_t edtTemperature = 0;
volatile uint8_t edtVoltage = 0;
volatile uint8_t edtCurrent = 0;
volatile uint8_t edtDebug1 = 0;
volatile uint8_t edtDebug2 = 0;
volatile uint8_t edtDebug3 = 0;
volatile uint8_t edtState = 0;

uint32_t lastPeriodTime = 0;

#define DELAY_CYCLES(n) __builtin_avr_delay_cycles(n)

void sendDshot300Frame(uint16_t notelem, uint16_t telem, uint8_t motor);
void sendDshot300Bit(uint8_t bit);
void sendInvertedDshot300Bit(uint8_t bit);
void sendBit(uint8_t A, uint8_t C); // (byte m1, byte m2, byte m3, byte m4, byte m5, byte m6, byte m7, byte m8, byte m9);

void readUpdate();
void printResponse();

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// KISS TELEM STUFF ////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t ticker = 0;
uint16_t dshotValue = 0;

uint8_t mreq = 9;
uint8_t Amask = NOmask;
uint8_t Cmask = NOmask;

uint8_t test = 0;

#define telemPin 19 //13

static int16_t ESC_telemetrie[5]; // Temperature, Voltage, Current, used mAh, eRpM
static uint16_t requestTelemetrie = 0;
static uint16_t regularThrottleSignal = 1000;

static uint8_t receivedBytes = 0;

static uint8_t SerialBuf[10];

void KISSsetup() {
  pinMode(telemPin, INPUT_PULLUP);
  Serial1.begin(115200); // open Serial1 for ESC communication
}

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed){
	uint8_t crc_u, i;
	crc_u = crc;
	crc_u ^= crc_seed;
	for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
	return (crc_u);
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen){
	uint8_t crc = 0, i;
	for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
	return (crc);
}

void receiveTelemtrie(){
  if (Serial1.available()) {
    uint8_t inByte = Serial1.read();
    //Serial.write(inByte);
    
    if(receivedBytes < 10){ // collect bytes
      //while(Serial1.available()){
      SerialBuf[receivedBytes] = inByte;
      receivedBytes++;
      //}
      if(receivedBytes == 10){ // transmission complete
        
        uint8_t crc8 = get_crc8(SerialBuf, 9); // get the 8 bit CRC
        
        if(crc8 != SerialBuf[9]) return; // transmission failure 
        
        // compute the received values
        ESC_telemetrie[0] = SerialBuf[0]; // temperature
        ESC_telemetrie[1] = (SerialBuf[1]<<8)|SerialBuf[2]; // voltage
        ESC_telemetrie[2] = (SerialBuf[3]<<8)|SerialBuf[4]; // Current
        ESC_telemetrie[3] = (SerialBuf[5]<<8)|SerialBuf[6]; // used mA/h
        ESC_telemetrie[4] = (SerialBuf[7]<<8)|SerialBuf[8]; // eRpM *100

        //Serial.println("GOT DATA");
        if(debug){
          printTelem();
        }
        static uint8_t SerialBuf[10];
        receivedBytes = 0;
      }
    } else {
      // Error handline
    }
  }
}

void printTelem() {
  if (ESC_telemetrie != 0) {
    Serial.println("Telemetrie Recieved");
    Serial.print("Temperature (C): ");
    Serial.println(ESC_telemetrie[0]); 
    Serial.print("Voltage: (V) /100: ");
    Serial.println(ESC_telemetrie[1]);   
    Serial.print("Current (A) /100: ");
    Serial.println(ESC_telemetrie[2]); 
    Serial.print("used mA/h: ");
    Serial.println(ESC_telemetrie[3]);   
    Serial.print("eRpM *100: ");
    Serial.println(ESC_telemetrie[4]);  
    Serial.println(" ");
  }
}

void establishContact() {
  while (Serial.available() <= 0) {
    Serial.print('a');   // send a capital A
    delay(300);
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// end TELEM STUFF ////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void sendDshot300Frame(uint16_t notelem, uint16_t telem, uint8_t motor) {
  //uint16_t temp = frame;
  uint8_t offset = 0;
  //byte telemetry = 1; 
  // Serial.println(temp, BIN);

  //uint16_t telem = request;
  //uint16_t notelem = temp;

  uint16_t M1 = notelem;
  uint16_t M2 = notelem;
  uint16_t M3 = notelem;
  uint16_t M4 = notelem;
  uint16_t M5 = notelem;
  uint16_t M6 = notelem;
  uint16_t M7 = notelem;
  uint16_t M8 = notelem;
  uint16_t M9 = notelem;

  switch(motor) {
    case 1:
      Serial.println("M1 Telem");
      M1 = telem;
      break;
    case 2:
      Serial.println("M2 Telem");
      M2 = telem;
      break;
    case 3:
      Serial.println("M3 Telem");
      M3 = telem;
      break;
    case 4:
      Serial.println("M4 Telem");
      M4 = telem;
      break;  
    case 5:
      Serial.println("M5 Telem");
      M5 = telem;
      break;
    case 6:
      Serial.println("M6 Telem");
      M6 = telem;
      break;
    case 7:
      Serial.println("M7 Telem");
      M7 = telem;
      break;
    case 8:
      Serial.println("M8 Telem");
      M8 = telem;
      break;
    case 9:
      Serial.println("M9 Telem");
      M9 = telem;
      break;
  }

  unsigned int A[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  unsigned int C[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  do {
    //A[offset] = ~(B00000000 | ((M1 & 0x8000) >> 8) | ((M2 & 0x8000) >> 9) | ((M3 & 0x8000) >> 10) | ((M4 & 0x8000) >> 11) | ((M5 & 0x8000) >> 12) | ((M6 & 0x8000) >> 13) | ((M7 & 0x8000) >> 14) | ((M8 & 0x8000) >> 15));
    A[offset] = B00000000 | (M1 & 0x8000) >> 15;
    A[offset] |= (M2 & 0x8000) >> 14;
    A[offset] |= (M3 & 0x8000) >> 13;
    A[offset] |= (M4 & 0x8000) >> 12;
    A[offset] |= (M5 & 0x8000) >> 11;
    A[offset] |= (M6 & 0x8000) >> 10;
    A[offset] |= (M7 & 0x8000) >> 9;
    A[offset] |= (M8 & 0x8000) >> 8;
    A[offset] = ~A[offset];

    C[offset] = ~(B01111111 | ((M9 & 0x8000) >> 8));

    //sendBit(A, C);
    //sendBit((M1 & 0x8000) >> 15, (M2 & 0x8000) >> 15, (M3 & 0x8000) >> 15, (M4 & 0x8000) >> 15, (M5 & 0x8000) >> 15, (M6 & 0x8000) >> 15, (M7 & 0x8000) >> 15, (M8 & 0x8000) >> 15, (M9 & 0x8000) >> 15);
    M1 <<= 1;
    M2 <<= 1;
    M3 <<= 1;
    M4 <<= 1;
    M5 <<= 1;
    M6 <<= 1;
    M7 <<= 1;
    M8 <<= 1;
    M9 <<= 1;

  } while(++offset < 0x10);

  offset = 0;

  do {
//    sendBit((M1 & 0x8000) >> 15, (M2 & 0x8000) >> 15, (M3 & 0x8000) >> 15, (M4 & 0x8000) >> 15, (M5 & 0x8000) >> 15, (M6 & 0x8000) >> 15, (M7 & 0x8000) >> 15, (M8 & 0x8000) >> 15, (M9 & 0x8000) >> 15);
    sendBit(A[offset], C[offset]);
  } while(++offset < 0x10);
}

/**
 * digitalWrite takes about 3.4us to execute, that's why we switch ports directly.
 * Switching ports directly will allow a transition in 0.19us or 190ns.
 *
 * In an optimal case, without any lag for sending a "1" we would switch high, stay high for 2500 ns (40 ticks) and then switch back to low.
 * Since a transition takes some time too, we need to adjust the waiting period accordingly. Ther resulting values have been set using an
 * oscilloscope to validate the delay cycles.
 *
 * Duration for a single byte should be 1/300kHz = 3333.33ns = 3.3us or 53.3 ticks
 *
 * for uno: 16MHz = 0.0625 µs(p) -> 3.3us / 0.0625 µs(p) = 52.8
 * for mega:  
 *
 * The delays after switching back to low are to account for the overhead of going through the loop ins sendBitsDshot*
 */
void sendInvertedDshot300Bit(uint8_t bit) {
  if(bit) {
    PORTA = B00000000;
    PORTC = B00000000;
    DELAY_CYCLES(37);
    PORTA = B11111111;
    PORTC = B10000000;
    DELAY_CYCLES(7);
  } else {
    PORTA = B00000000;
    PORTC = B00000000;
    DELAY_CYCLES(16);
    PORTA = B11111111;
    PORTC = B10000000;
    DELAY_CYCLES(25);
  }
}

void sendBit(uint8_t A, uint8_t C){//(byte m1, byte m2, byte m3, byte m4, byte m5, byte m6, byte m7, byte m8, byte m9) {
  PORTA = B00000000;
  PORTC = B00000000;
  DELAY_CYCLES(16); // 16 //10 works when B11111111 in next line
  PORTA = A; //~(B00000000 | (m1 << 7) | (m2 << 6) | (m3 << 5) | (m4 << 4) | (m5 << 3) | (m6 << 2) | (m7 << 1) | (m8));
  PORTC = C; //~(B01111111 | (m9 << 7));
  DELAY_CYCLES(21); //21
  PORTA = B11111111;
  PORTC = B10000000;
  DELAY_CYCLES(7);
}

void setupTimer() {
  cli();

  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  // 500 Hz (16000000/((124 + 1) * 256))
  OCR2A = 124;
  TCCR2B |= 0b00000110; // Prescaler 256

  TCCR2A |= 0b00001010; // CTC mode - count to OCR2A
  TIMSK2 = 0b00000010; // Enable INT on compare match A

  sei();
}

ISR(TIMER2_COMPA_vect) {
  ticker++;
  //mreq++;
  if(ticker == 499){ //499 
    sendDshot300Frame(frame, request, mreq);
    ticker = 0;
    mreq++;
    if(mreq > 9){
      mreq = 0;
    }
  }
  else{
    sendDshot300Frame(frame, request, 0);
  }
}

void dshotSetup() {
  Serial.begin(9600);
  while(!Serial);

  if(laptop){
    establishContact(); 
  }
364365366367368369370371372373374375376377378379380381382383384385386387388389390391392393394395396397
  // Set the default signal Level B12345678
  PORTA = B11111111;
  PORTC = B10000000;

  #if debug
    Serial.println("Input throttle value to be sent to ESC");
    Serial.println("Valid throttle values are 47 - 2047");
    Serial.println("Lines are only printed if the value changed");

    Serial.print("Frames are sent repeatadly in the chosen update frequency: ");
    Serial.print(frequency);



  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(m3, OUTPUT);
  pinMode(m4, OUTPUT);
  pinMode(m5, OUTPUT);
  pinMode(m6, OUTPUT);
  pinMode(m7, OUTPUT);
  pinMode(m8, OUTPUT);
  pinMode(m9, OUTPUT);

  // Set the default signal Level B12345678
  PORTA = B11111111;
  PORTC = B10000000;

  #if debug
    Serial.println("Input throttle value to be sent to ESC");
    Serial.println("Valid throttle values are 47 - 2047");
    Serial.println("Lines are only printed if the value changed");
    Serial.print("Frames are sent repeatadly in the chosen update frequency: ");
    Serial.print(frequency);
    Serial.println("Hz");
  #endif

  setupTimer(); // loop start
}

void consolloop() {
  if(Serial.available() > 0) {
    dshotValue = Serial.parseInt(SKIP_NONE);
    Serial.read();

    if(dshotValue > 2047) {
      dshotValue = 2047;
    }
    frame = dshot.buildFrame(dshotValue, 0);
    request = dshot.buildFrame(dshotValue, 1);

    if(debug){
      Serial.print("> Frame: ");
      Serial.print(frame, BIN);
      Serial.print(", Request: ");
      Serial.print(request, BIN);
      Serial.print(", Value: ");
      Serial.println(dshotValue);
    }
  }
}

void laptoploop() {
  // if we get a valid byte, read analog ins:

  if (Serial.available() > 0) {
    // get incoming byte:
    inByte = Serial.read();

    if (inByte == 'a') {
      nextByte = false;

      if(dshotValue > 2047) {
        dshotValue = 2047;
      }
      frame = dshot.buildFrame(dshotValue, 0);
      request = dshot.buildFrame(dshotValue, 1);

      temp = "";
      Serial.print(dshotValue); // its sending '9' then '5' not 95 *facepalm*
      Serial.print(' '); // sending a space to indicate new number
      Serial.print(ESC_telemetrie[0]);
      Serial.print(' ');
      Serial.print(ESC_telemetrie[1]); // float( /100)
      Serial.print(' ');
      Serial.print(ESC_telemetrie[4]);
      Serial.print(' ');
      Serial.print('a'); // telling computer its done sending
    } else if (inByte == 't' || nextByte == true && inByte != 10) {
      //Serial.print("b");
      if (inByte != 't' && inByte != NULL) {
        //Serial.print("Reading throttle");
        temp += (inByte-48);
        //Serial.print(temp);
        dshotValue = temp.toInt();
      }
      nextByte = true;
      //Serial.println("NextByte true");
      Serial.print('a');
    } else {
      Serial.print('w');
      //Serial.print(inByte);
      nextByte = false;
      temp = "";
    }

  } else {
    Serial.print('a');
  }
}

void setup() {
  dshotSetup();
  KISSsetup();
}

void loop() {
  if(laptop){
    laptoploop();
  } else { 
    consolloop();
  }
  receiveTelemtrie();
}
