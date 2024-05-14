#include "Arduino.h"
#include "Dshot.h"
#include "Wire.h"

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
const FREQUENCY frequency = F500; // F500 = 62720

// DSHOT Output pin
const uint8_t pinDshot = 26; //7 Pin 13, B7

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
#define debug true

#define telemPin 2

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

#define inverted false
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
void readUpdate();
void printResponse();

////// FROM KISS CODE /////
static int16_t ESC_telemetrie[5]; // Temperature, Voltage, Current, used mAh, eRpM
static uint16_t requestTelemetrie = 0;
static uint16_t regularThrottleSignal = 1000;

static uint8_t receivedBytes = 0;

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

void receiveTelemtrie(){
  static uint8_t SerialBuf[10];
  
  if(receivedBytes < 9){ // collect bytes
    while(Serial1.available()){
      SerialBuf[receivedBytes] = Serial1.read();
      receivedBytes++;
    }
    if(receivedBytes == 10){ // transmission complete
      
      uint8_t crc8 = get_crc8(SerialBuf, 9); // get the 8 bit CRC
      
      if(crc8 != SerialBuf[9]) return; // transmission failure 
      
      // compute the received values
      ESC_telemetrie[0] = SerialBuf[0]; // temperature
      ESC_telemetrie[1] = (SerialBuf[1]<<8)|SerialBuf[2]; // voltage
      ESC_telemetrie[2] = (SerialBuf[3]<<8)|SerialBuf[4]; // Current
      ESC_telemetrie[3] = (SerialBuf[5]<<8)|SerialBuf[6]; // used mA/h
      ESC_telemetrie[4] = (SerialBuf[7]<<8)|SerialBuf[8]; // eRpM *100
    }
  }
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

void KISSloop() {
   static uint32_t loopTime = 0;
   static uint8_t tlemetrieCounter = 0;
   if(micros()-loopTime > 2000){ // 2000Hz looptime
      loopTime = micros();
      receiveTelemtrie(); // look for incoming telemetrie
      if(++tlemetrieCounter == 2000){ // get telemetrie with 25Hz //was 20
        tlemetrieCounter = 0;
        receivedBytes = 0; // reset bytes counter
        // request telemetrie with a 30µs signal
        OCR1A = 30<<4;
      }else{
        OCR1A = regularThrottleSignal<<1;  
      }
      
      //print the telemetry
      if(tlemetrieCounter == 10){ //10 //(tlemetrieCounter == 10)
         Serial.println("Requested Telemetrie");
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
      }else{
        //fire oneshot only when not sending the serial datas .. arduino serial library is too slow
        TCNT1 = 0xFFFF; OCR1A=0; 
      }
  }
}

////// end functions from KISS /////

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
    sendDshot300Bit((temp & 0x8000) >> 15);
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
 * The delays after switching back to low are to account for the overhead of going through the loop ins sendBitsDshot*
 */

void sendDshot300Bit(uint8_t bit) {
  if(bit) {
    PORTB = B11111111, //B00000001;
    //DELAY_CYCLES(40);
    DELAY_CYCLES(37);
    PORTB = B00000000;
    //DELAY_CYCLES(13);
    DELAY_CYCLES(7);
  } else {
    PORTB = B00000010; //B00000001;
    //DELAY_CYCLES(20);
    DELAY_CYCLES(16);
    PORTB = B11111111;
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

  #if inverted
    processTelemetryResponse();
    newResponse = true;
  #endif
}

void dshotSetup() {
  Serial.begin(9600);
  while(!Serial);

  pinMode(pinDshot, OUTPUT);

  PORTB = B00000000;

  //#if debug
  Serial.println("Input throttle value to be sent to ESC");
  Serial.println("Valid throttle values are 47 - 2047");
  Serial.println("Lines are only printed if the value changed");
  Serial.print("Frames are sent repeatadly in the chosen update frequency: ");
  Serial.print(frequency);
  Serial.println("Hz");

  //  if(enableEdt) {
  //    Serial.println();
  //    Serial.println("Send 13 to enable extended DShot telemetry");
  //    Serial.println("CAUTION: EDT is disabled once disarmed (sending 0 after non 0 throttle value)");
  //  }
  //#endif

  setupTimer();
}

void readUpdate() {
  // Serial read might not always trigger properly here since the timer might interrupt
  // Disabling the interrupts is not an option since Serial uses interrupts too.
  if(Serial.available() > 0) {
    uint16_t dshotValue = Serial.parseInt(SKIP_NONE);
    Serial.read();

    if(dshotValue > 2047) {
      dshotValue = 2047;
    }
    frame = dshot.buildFrame(dshotValue, 1);

    Serial.print("> Frame: ");
    Serial.print(frame, BIN);
    Serial.print(" Value: ");
    Serial.println(dshotValue);

    sendDshot300Frame();
    //Serial.println(sizeof(frame, BIN));
  }
}

void printResponse() {
  if(newResponse) {
    newResponse  = false;

    uint16_t mapped = dshot.mapTo16Bit(dshotResponse);
    uint8_t crc = mapped & 0x0F;
    uint16_t value = mapped >> 4;
    uint8_t crcExpected = dshot.calculateCrc(value);

    //Serial.println(mapped, BIN);
    Serial.print(value, BIN);
    Serial.print(" ");
    Serial.println(crc, BIN);

    // Wait for a first valid response
    if(!hasEsc) {
      if(crc == crcExpected) {
        hasEsc = true;
      }

      return;
    }

    // Calculate success rate - percentage of packeges on which CRC matched the value
    receivedPackets++;
    if(crc == crcExpected) {
      successPackets++;
    }

    // Reset packet count if overflows
    if(!receivedPackets) {
      successPackets = 0;
    }

    if((dshotResponse != dshotResponseLast) || !debug) {
      dshotResponseLast = dshotResponse;

      // DShot Frame: EEEMMMMMMMMM
      uint32_t periodBase = value & 0b0000000111111111;
      uint8_t periodShift = value >> 9 & 0b00000111;
      uint32_t periodTime =  periodBase << periodShift;

      uint8_t packageType = value >> 8 & 0b00001111;

      /**if(enableEdt) {
        
         * Extended DShot Frame: PPPEMMMMMMMM
         *
         * In extended Dshot the first bit after after
         * the exponent indicated if it is telemetry
         *
         * A 0 bit indicates telemetry, in this case
         * the exponent maps to a certain type of telemetry.
         *
         * A telemetry package only happens every n packages
         * and it does not include ERPM data, it is assumed
         * that the previous ERPM value is still valid.
         
        if((packageType & 0x01) == 0) {
          switch(packageType) {
            case 0x02: edtTemperature = periodBase; break;
            case 0x04: edtVoltage = periodBase; break;
            case 0x06: edtCurrent = periodBase; break;
            case 0x08: edtDebug1 = periodBase; break;
            case 0x0A: edtDebug2 = periodBase; break;
            case 0x0C: edtDebug3 = periodBase; break;
            case 0x0E: edtState = periodBase; break;
          }

          periodTime = lastPeriodTime;
        } else {
          lastPeriodTime = periodTime;
        }
      }
      */

      if(crc == crcExpected) {
        Serial.print("OK: ");
      } else {
        Serial.print("--: ");
      }

      #if debug
        float successPercent = (successPackets * 1.0 / receivedPackets * 1.0) * 100;
      #endif

      Serial.print(periodTime);
      #if debug
        Serial.print("us ");
        Serial.print(round(successPercent));
        Serial.print("%");

        /** if(enableEdt) {
          Serial.print(" EDT: ");
          Serial.print(edtTemperature);
          Serial.print("°C");

          Serial.print(" | ");
          Serial.print(edtVoltage * 0.25);
          Serial.print("V");

          Serial.print(" | ");
          Serial.print(edtCurrent);
          Serial.print("A");

          Serial.print(" | D1: ");
          Serial.print(edtDebug1);

          Serial.print(" | D2: ");
          Serial.print(edtDebug2);

          Serial.print(" | D3: ");
          Serial.print(edtDebug3);

          Serial.print(" | S: ");
          Serial.print(edtState);
        } */
      #endif
      Serial.println();
    }
  }
}

void dshotLoop() {
  readUpdate();
  //printResponse();
}

void setup() {
  dshotSetup();
  //KISSsetup();
}

void loop() {
  dshotLoop();
  //KISSloop();

}