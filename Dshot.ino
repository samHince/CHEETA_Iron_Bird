#include "Arduino.h"
#include "Dshot.h"
#include "C2.h"

/**
 * Update frequencies from 2kHz onwards tend to cause issues in regards
 * to processing the DShot response and will result in actual 3kHz instead.
 *
 * At this point the serial port will not be able to print anything anymore since
 * interrupts are "queing" up, rendering the main loop basically non-functional.
 *
 * 8kHz can ONLY be achieved when sending (uninverted) Dshot und not processing
 * responses.
 *
 * For real world use you should thus not go over 1kHz, better stay at 500Hz, this
 * will give you some headroom in order to serial print some more data.
 *
 * The limit of sending at 3kHz shows that the time difference is the actual time
 * that is needed to process the DShot response - so around 400us - 400kns or 6400
 * clock cycles.
 *
 * Even if response processing can be sped up, at the higher frequencies we would
 * still struggle to serial print the results.
 */
const FREQUENCY frequency = F500;

// Set inverted to true if you want bi-directional DShot
#define inverted true

// Enable Extended Dshot Telemetry
bool enableEdt = false;

// DSHOT Output pin
const uint8_t pinDshot = 8;

/**
 * If debug mode is enabled, more information is printed to the serial console:
 * - Percentage of packages successfully received (CRC checksums match)
 * - Information on startup
 *
 * With debug disabled output will look like so:
 * --: 65408
 * OK: 65408
 *
 * With debug enabled output will look like so:
 * OK: 13696us 96.52%
 * --: 65408us 96.43%
 * OK: 13696us 96.43%
 * OK: 22400us 96.44%
 */
#define debug false
#define laptop true
int inByte = 0;         // incoming serial byte
String temp = "";
bool nextByte = true;

// If this pin is pulled low, then the device starts in C2 interface mode
#define C2_ENABLE_PIN 13

/*
// When using Port B, make sure to remap the C2_ENABLE_PIN to a different port
// Port definitions for Port B
#define C2_PORT PORTB
#define C2_DDR DDRB
#define C2_PIN PINB

// Pin 0-7 for the given port
#define C2D_PIN  4 // D12
#define C2CK_PIN 3 // D11
*/

// Port definitions for Port D
#define C2_PORT PORTD
#define C2_DDR DDRD
#define C2_PIN PIND

/* Pin 0-7 for the given port */
#define C2D_PIN  2 // D2
#define C2CK_PIN 3 // D3

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

bool c2Mode = false;
C2 *c2;

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

volatile uint8_t edtTemperature = 0;
volatile uint8_t edtVoltage = 0;
volatile uint8_t edtCurrent = 0;
volatile uint8_t edtDebug1 = 0;
volatile uint8_t edtDebug2 = 0;
volatile uint8_t edtDebug3 = 0;
volatile uint8_t edtState = 0;

uint32_t lastPeriodTime = 0;

#define DELAY_CYCLES(n) __builtin_avr_delay_cycles(n)

void sendDshot300Frame();
void sendDshot300Bit(uint8_t bit);
void sendInvertedDshot300Bit(uint8_t bit);
void processTelemetryResponse();
void readUpdate();
void printResponse();

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// KISS TELEM STUFF ////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t ticker = 0;
uint16_t dshotValue = 0;


#define telemPin 13

static int16_t ESC_telemetrie[5]; // Temperature, Voltage, Current, used mAh, eRpM
static uint16_t requestTelemetrie = 0;
static uint16_t regularThrottleSignal = 1000;

static uint8_t receivedBytes = 0;

static uint8_t SerialBuf[10];

void KISSsetup() {
  //// init timer 1 
  //pinMode(9,OUTPUT);
  //TCNT1  = 0;  TCCR1C = 0; TIMSK1 = 0;
  //TCCR1A = B10101010;     // PWM_WaveformMode=14 -> Fast_PWM, TOP=ICRn, PWM_OutputMode=non-inverting
  //TCCR1B = B00011001;     // Prescaler=clk/1 / Imp=125.. 250us @11Bit oder Imp=1000.. 2000us @14Bit
  //ICR1 = 0xFFFF;          // set TOP TIMER1 to max
  
  //DDRB &= ~(1 << 6); // pin 7 to input
  //PORTB |= (1 << 6); // enable pullups
  //EIMSK |= (1 << INT6); // enable interuppt
  //EICRB |= (1 << ISC60);
  pinMode(telemPin, INPUT_PULLUP);
  // inturrupt? https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
  
  
  //Serial.begin(9600); // open seria0 for serial monitor
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
        //printTelem();
      }
    } else {
      //Serial.println("RECIEVE ERROR");
    }
  }
}

void KISSloop () {
   static uint32_t loopTime = 0;
   static uint8_t tlemetrieCounter = 0;
   if(micros()-loopTime > 2000){ // 2000Hz looptime
      loopTime = micros();
      receiveTelemtrie(); // look for incoming telemetrie
      if(++tlemetrieCounter == 2000){ // get telemetrie with 25Hz //was 20
        tlemetrieCounter = 0;
        receivedBytes = 0; // reset bytes counter
        // request telemetrie with a 30µs signal
      //  OCR1A = 30<<4;
      }else{
      //  OCR1A = regularThrottleSignal<<1;  
      }
      
      //print the telemetry
      //if(tlemetrieCounter == 10){ //10 //(tlemetrieCounter == 10)
      //   Serial.println("Requested Telemetrie");
      //   Serial.print("Temperature (C): ");
      //   Serial.println(ESC_telemetrie[0]); 
      //   Serial.print("Voltage: (V) /100: ");
      //   Serial.println(ESC_telemetrie[1]);   
      //   Serial.print("Current (A) /100: ");
      //   Serial.println(ESC_telemetrie[2]); 
      //   Serial.print("used mA/h: ");
      //   Serial.println(ESC_telemetrie[3]);   
      //   Serial.print("eRpM *100: ");
      //   Serial.println(ESC_telemetrie[4]);  
      //   Serial.println(" ");
      //   Serial.println(" ");  
      //}else{
        //fire oneshot only when not sending the serial datas .. arduino serial library is too slow
        //TCNT1 = 0xFFFF; OCR1A=0; 
      //}
  }
}

void printTelem() {
  // read from port 1, send to port 0:

  //if (Serial1.available()) {
  //  uint8_t inByte = Serial1.read();
  //  Serial.write(char(inByte));
  //}




  //receiveTelemtrie(); // look for incoming telemetrie
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
    Serial.println(" ");  
  }
}

void testloop() {
  // read from port 1, send to port 0:

  if (Serial1.available()) {
    uint8_t inByte = Serial1.read();
    Serial.write(inByte);
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

void processTelemetryResponse() {
  // Set to Input in order to process the response - this will be at 3.3V level
  pinMode(pinDshot, INPUT_PULLUP);

  // Delay around 26us
  DELAY_CYCLES(410);

  register uint8_t ices1High = 0b01000000;
  register uint16_t prevVal = 0;
  register uint8_t tifr;
  register uint16_t *pCapDat;

  TCCR1A = 0b00000001; // Toggle OC1A on compare match
  TCCR1B = 0b00000010; // trigger on falling edge, prescaler 8, filter off

  // Limit to 70us - that should be enough to fetch the whole response
  // at 2MHz - scale factor 8 - 150 ticks seems to be a sweetspot.
  OCR1A = 150;
  TCNT1 = 0x00;

  TIFR1 = (1 << ICF1) | (1 << OCF1A) | (1 << TOV1); // clear all timer flags
  for(pCapDat = counter; pCapDat <= &counter[buffSize - 1];) {
    // wait for edge or overflow (output compare match)
    while(!(tifr = (TIFR1 & ((1 << ICF1) | (1 << OCF1A))))) {}

    uint16_t val = ICR1;

    // Break if counter overflows
    if(tifr & (1 << OCF1A)) {
      // Ignore overflow at the beginning of capture
      if(pCapDat != counter) {
        break;
      }
    }

    TCCR1B ^= ices1High; // toggle the trigger edge
    TIFR1 = (1 << ICF1) | (1 << OCF1A); // clear input capture and output compare flag bit

    *pCapDat = val - prevVal;

    prevVal = val;
    pCapDat++;
  }

  pinMode(pinDshot, OUTPUT);

  // Set all 21 possible bits to one and flip the once that should be zero
  dshotResponse = 0x001FFFFF;
  unsigned long bitValue = 0x00;
  uint8_t bitCount = 0;
  for(uint8_t i = 1; i < buffSize; i += 1) {
    // We are done once the first intereval has a 0 value.
    if(counter[i] == 0) {
      break;
    }

    bitValue ^= 0x01; // Toggle bit value - always start with 0
    counter[i] = duration[counter[i]];
    for(uint8_t j = 0; j < counter[i]; j += 1) {
      dshotResponse ^= (bitValue << (20 - bitCount++));
    }
  }

  // Decode GCR 21 -> 20 bit (since the 21st bit is definetly a 0)
  dshotResponse ^= (dshotResponse >> 1);
}

/**
 * Frames are sent MSB first.
 *
 * Unfortunately we can't  rotate through carry on an ATMega.
 * Thus we fetch MSB, left shift the result and then right shift the frame.
 *
 * IMPROVEMENT: Since this part is actually time critical, any improvement that can be
 *              made is a good improvment. Thus when the frame is initially generated
 *              it might make sense to arange it in a way that is benefitial for
 *              transmission.
 */
void sendDshot300Frame() {
  uint16_t temp = frame;
  uint8_t offset = 0;
  do {
    #if inverted
      sendInvertedDshot300Bit((temp & 0x8000) >> 15);
    #else
      sendDshot300Bit((temp & 0x8000) >> 15);
    #endif
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

void sendDshot300Bit(uint8_t bit) {
  if(bit) {
    PORTH = B00100000; //B00100000;PORTH = B00100000;
    //DELAY_CYCLES(40);
    DELAY_CYCLES(37);
    PORTH = B00000000;
    //DELAY_CYCLES(13);
    DELAY_CYCLES(7);
  } else {
    PORTH = B00100000; //B00100000;PORTH = B00100000;
    //DELAY_CYCLES(20);
    DELAY_CYCLES(16);
    PORTH = B00000000;
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
  sendDshot300Frame();

  ticker++;
  if(ticker == 1000){ //65535
    //Serial.println("data");
    
    frame = dshot.buildFrame(dshotValue, 1);
    //receiveTelemtrie();
    //frame = dshot.buildFrame(dshotValue, 0);

  }
  if(ticker == 1001){ //65535
    receiveTelemtrie();
    //Serial.println("data off");
    ticker = 0;
    //frame = dshot.buildFrame(dshotValue, 1);
    //receiveTelemtrie();
    frame = dshot.buildFrame(dshotValue, 0);

  }

  //############
  //testLoop();
  //KISSloop();
  //receiveTelemtrie();
  //############

  #if inverted
    processTelemetryResponse();
    newResponse = true;
  #endif
}

void dshotSetup() {
  Serial.begin(9600);
  while(!Serial);

  if(laptop){
    establishContact(); 
  }

  pinMode(pinDshot, OUTPUT);

  // Set the default signal Level
  #if inverted
    PORTH = B00100000;
  #else
    PORTH = B00000000;
  #endif

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

void readUpdate() {
  if(Serial.available() > 0) {
    dshotValue = Serial.parseInt(SKIP_NONE);
    Serial.read();

    if(dshotValue > 2047) {
      dshotValue = 2047;
    }
    frame = dshot.buildFrame(dshotValue, 0);

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

      //dshotValue = Serial.parseInt(SKIP_NONE);
      //Serial.read();

      if(dshotValue > 2047) {
        dshotValue = 2047;
      }
      frame = dshot.buildFrame(dshotValue, 0);

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

void dshotLoop() {
  if(laptop){
    laptoploop();
  } else { 
    readUpdate();
  }
}

void setup() {
  dshotSetup();
  KISSsetup();
}

void loop() {
  dshotLoop();
  receiveTelemtrie();
}
