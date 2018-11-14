// -------------------------------------------------------------
// ,CANtest for Teensy 3.6 dual CAN bus
// by Collin Kidder, Based on CANTest by Pawelsky (based on CANtest by teachop)
//
// Both buses are left at default 250k speed and the second bus sends frames to the first
// to do this properly you should have the two buses linked together. This sketch
// also assumes that you need to set enable pins active. Comment out if not using
// enable pins or set them to your correct pins.
//
// This sketch tests both buses as well as interrupt driven Rx and Tx. There are only
// two Tx buffers by default so sending 5 at a time forces the interrupt driven system
// to buffer the final three and send them via interrupts. All the while all Rx frames
// are internally saved to a software buffer by the interrupt handler.
//
#include <FlexCAN.h>
#ifndef __MK66FX1M0__
#error "Teensy 3.6 with dual CAN bus is required to run this example"
#endif

int BuiltInLed = 13;
bool toggle = 1;

static CAN_message_t msg;
static uint8_t hex[17] = "0123456789abcdef";

// -------------------------------------------------------------
static void hexDump(uint8_t dumpLen, uint8_t *bytePtr)
{

  uint8_t working;
  while ( dumpLen-- ) {
    working = *bytePtr++;
    Serial.write( hex[ working >> 4 ] );
    Serial.write( hex[ working & 15 ] );
  }
  Serial.write('\r');
  Serial.write('\n');
}


// -------------------------------------------------------------
void setup(void)
{

  delay(1000);
  Serial.println(F("Hello Teensy 3.6 dual CAN Test."));
  //Define deafult filter
  static struct CAN_filter_t defaultMask;
  defaultMask.id = 0;

  Can0.begin(250000, defaultMask, 1, 1 );
  Can1.begin(250000, defaultMask, 0, 0 );

  //if using enable pins on a transceiver they need to be set on
  pinMode(13, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(35, OUTPUT);

  digitalWrite(13, HIGH);
  digitalWrite(28, LOW);
  digitalWrite(35, LOW);


  msg.ext = 0;
  msg.id = 0x100;
  msg.len = 8;
}


// -------------------------------------------------------------
void loop(void)
{

  CAN_message_t inMsg; // leser transmitter CAN-ID
  while (Can0.available()) // Når Peak / PCAN sender (space)
  {
    for (int i=0; i<8; i++) {
      Serial.print(inMsg.buf[i]); Serial.print(" ");
    }
    Can0.read(inMsg);   // inMsg == CAN-ID til transmitter (==Peak)
    Serial.println(inMsg.id, HEX); // konverterer CAN-ID fra dec til hex
    //Serial.println("CAN bus 0: Hei "); hexDump(8, inMsg.buf);
    Can0.write(inMsg); // Can0 mottar
    

    if (inMsg.id == 0x21) { // TX ID-21 toggler LED
      Serial.println("te");
      digitalWrite(BuiltInLed, toggle);
      toggle = !toggle;
    }
    else if (inMsg.id == 0x22) {
       bool sisteBit = inMsg.buf[7]; // siste siffer i data til TX ID-22 styrer LED
      digitalWrite(BuiltInLed, sisteBit);
    }
    else if (inMsg.id == 0x20) {
      Serial.println("ggf");
    }
  }

  msg.buf[0] = 0x53;  // Receiver lagrer tallet i "Data"
  msg.buf[2] = 10;
  Can0.write(msg);  // Can0: count++

  delay(20);
}
