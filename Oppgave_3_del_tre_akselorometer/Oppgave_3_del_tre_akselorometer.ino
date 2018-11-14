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

//-------Til IMU-----------
#include<Wire.h>
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
unsigned long gammelT = 0;
unsigned long nyT = 0;
unsigned long dT = 0;
//-------------------------

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
  msg.id = 0x20;
  msg.len = 8;

  setupIMU();
}

// -------------------------------------------------------------
void loop(void)
{

  if (Can0.available()) // Når Peak / PCAN sender (space)
  {
    CAN_message_t inMsg; // leser transmitter CAN-ID
    Can0.read(inMsg);   // inMsg == CAN-ID til TX
    //Serial.println(inMsg.id, HEX); // konverterer CAN-ID fra dec til hex
    //Serial.println("CAN bus 0: Hei "); hexDump(8, inMsg.buf);
    Can0.write(inMsg); // Can0 mottar og lagrer Data som blir sendt av TX


    if (inMsg.id == 0x21) { // TX ID-21 toggler LED
      digitalWrite(BuiltInLed, toggle);
      toggle = !toggle;
    }
    else if (inMsg.id == 0x22) {
      bool sisteBit = inMsg.buf[7]; // siste siffer i data til TX ID-22 styrer LED
      digitalWrite(BuiltInLed, sisteBit);
    }

  }

  loopIMU();

  nyT = millis();
  dT = nyT - gammelT;
  if (dT >= 100) { // Hvert sekund fordi T = 1/f = 1/1Hz = 1
    gammelT = millis();
    dT = 0;

    AcY += 500;
    AcZ -= 15100;
    AcX = map(AcX, -15000,15000, 0, 255);
    AcY = map(AcY, -15000,15000, 0, 255);
    AcZ = map(AcZ, -15000,15000, 0, 255);
    Serial.print("AcX: ");
    Serial.print(AcX);
    Serial.print("   AcY: ");
    Serial.print(AcY);
    Serial.print("   AcZ: ");
    Serial.println(AcZ);
    
    msg.buf[0] = (AcX);
    msg.buf[1] = (AcY);
    msg.buf[2] = (AcZ);
    Can0.write(msg);
  }

  delay(20);
}


void setupIMU() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);
}

void loopIMU() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
//  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
//  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
//  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}
