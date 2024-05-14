#include "Arduino.h"
#include "Dshot.h"

const FREQUENCY frequency = F500;

// Set inverted to true if you want bi-directional DShot
#define inverted true

// Enable Extended Dshot Telemetry
bool enableEdt = false;

// DSHOT Output pin
const uint8_t pinDshot = 8;

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

bool newResponse = false;
bool hasEsc = false;



// Duration LUT - considerably faster than division
const uint8_t duration[] = {
  0,
  0,
  0,
  0,
  1, // 4
  1, // 5 <
  1, // 6
  2, // 7
  2,
  2,
  2, // 10 <
  2,
  2, // 12
  3, // 13
  3,
  3, // 15 <
  3,
  3, // 17

  // There should not be more than 3 bits with the same state after each other
  4, // 18
  4,
  4, // 20 <
  4,
  4, // 22
};

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

void sendDshot300Frame(uint16_t temp);
void sendDshot300Bit(uint8_t bit);
void sendInvertedDshot300Bit(uint8_t bit);

void readUpdate();
void printResponse();

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// KISS TELEM STUFF ////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t ticker = 0;
uint16_t dshotValue = 0;


#define telemPin 19//13

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

void sendDshot300Frame(uint16_t temp) {
  //uint16_t temp = frame;
  uint8_t offset = 0;
  // Serial.println(temp, BIN);
  do {
    sendInvertedDshot300Bit((temp & 0x8000) >> 15);
    temp <<= 1;
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
    PORTH = B00000000;
    //DELAY_CYCLES(40);
    DELAY_CYCLES(37);
    PORTH = B00100000; //B00100000;
    //DELAY_CYCLES(13);
    DELAY_CYCLES(7);
  } else {
    PORTH = B00000000;
    //DELAY_CYCLES(20);
    DELAY_CYCLES(16);
    PORTH = B00100000; //B00100000;
    //DELAY_CYCLES(33);
    DELAY_CYCLES(25);
  }
}

void setupTimer() {
  cli();

  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  switch(frequency) {
    case F500: {
      // 500 Hz (16000000/((124 + 1) * 256))
      OCR2A = 124;
      TCCR2B |= 0b00000110; // Prescaler 256
    } break;

    case F1k: {
      // 1000 Hz (16000000/((124 + 1) * 128))
      OCR2A = 124;
      TCCR2B |= 0b00000101; // Prescaler 128
    } break;

    case F2k: {
      // 2000 Hz (16000000/((124 + 1) * 64))
      OCR2A = 124;
      TCCR2B |= 0b00000100; // Prescaler 64
    } break;

    case F4k: {
      // 4000 Hz (16000000/( (124 + 1) * 32))
      OCR2A = 124;
      TCCR2B |= 0b00000011; // Prescaler 32
    } break;

    default: {
      // 8000 Hz (16000000/( (249 + 1) * 8))
      OCR2A = 249;
      TCCR2B |= 0b00000010; // Prescaler 8
    } break;
  }

  TCCR2A |= 0b00001010; // CTC mode - count to OCR2A
  TIMSK2 = 0b00000010; // Enable INT on compare match A

  sei();
}

ISR(TIMER2_COMPA_vect) {
  ticker++;
  if(ticker == 499){ 
    sendDshot300Frame(request);
    ticker = 0;
  }
  else{
    sendDshot300Frame(frame);
  }
}

void dshotSetup() {
  Serial.begin(9600);
  while(!Serial);

  if(laptop){
    establishContact(); 
  }

  pinMode(pinDshot, OUTPUT);

  // Set the default signal Level
  PORTH = B00100000;

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
      Serial.print(" Value: ");
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
