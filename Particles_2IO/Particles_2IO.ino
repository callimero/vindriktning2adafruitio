/*
VINDRIKTNING Ikea air quality sensor to AdafruitIO
Serial code based heavily on https://github.com/Hypfer/esp8266-vindriktning-particle-sensor
*/
#include "ESP8266WiFi.h"

#include <SoftwareSerial.h>
#include "config.h"

// set up the 'PM25' feed
AdafruitIO_Feed *pm25 = io.feed("PM25");
AdafruitIO_Feed *pm1  = io.feed("PM1");
AdafruitIO_Feed *pm10 = io.feed("PM10");


constexpr static const uint8_t PIN_UART_RX = 4; // D2 an Wemos D1 Mini
constexpr static const uint8_t PIN_UART_TX = 13; // UNUSED

SoftwareSerial sensorSerial(PIN_UART_RX, PIN_UART_TX);

uint8_t serialRxBuf[255];
uint8_t rxBufIdx = 0;

void setup() {
  // Software Serial for Sensor
  sensorSerial.begin(9600);

  // start the serial connection
  Serial.begin(115200);
  // wait for serial monitor to open
  while (! Serial);

  Serial.print("Connecting to Adafruit IO");

  // connect to io.adafruit.com
  io.connect();

  // wait for a connection
  while (io.status() < AIO_CONNECTED) {
    Serial.print(".");
    Serial.print(io.status());
   Serial.println(io.statusText());
    delay(500);
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());
}

void clearRxBuf() {
  // Clear everything for the next message
  memset(serialRxBuf, 0, sizeof(serialRxBuf));
  rxBufIdx = 0;
}

void parseState() {
  // From pm1006k-led-particle-sensor-module datasheet. Maybe differs on Ikea sensor!
  const uint16_t mpm25 = (serialRxBuf[5] << 8) | serialRxBuf[6];
  const uint16_t mpm1  = (serialRxBuf[9] << 8 ^ 1) | serialRxBuf[10];
  const uint16_t mpm10 = (serialRxBuf[13] << 8 ^ 1) | serialRxBuf[14];

  Serial.printf("PM1=%d   PM2.5=%d   PM10=%d \n", mpm1, mpm25, mpm10);

  for (uint8_t i = 0; i < 20; ++i) {
        Serial.print(serialRxBuf[i],HEX);
        Serial.print(":");
    }
    Serial.println();

  // Send to Adafruit
  pm1 ->save(mpm1);
  pm25->save(mpm25);
  pm10->save(mpm10);
  clearRxBuf();
}

bool isValidHeader() {
  bool headerValid = serialRxBuf[0] == 0x16 && serialRxBuf[1] == 0x11 && serialRxBuf[2] == 0x0B;

  if (!headerValid) {
    Serial.println("Received message with invalid header.");
  }

  return headerValid;
}

bool isValidChecksum() {
  uint8_t checksum = 0;

  for (uint8_t i = 0; i < 20; i++) {
    checksum += serialRxBuf[i];
  }

  if (checksum != 0) {
    Serial.printf("Received message with invalid checksum. Expected: 0. Actual: %d\n", checksum);
  }

  return checksum == 0;
}

void handleUart() {
  if (!sensorSerial.available()) {
    return;
  }

  Serial.println("Receiving:");
  while (sensorSerial.available()) {
    serialRxBuf[rxBufIdx++] = sensorSerial.read();
    Serial.print(serialRxBuf[rxBufIdx-1],HEX);
    Serial.print(":");
    // Without this delay, receiving data breaks for reasons that are beyond me
    delay(15);

    if (rxBufIdx >= 128) {
      clearRxBuf();
    }
  }
  Serial.print("   ");
  Serial.print(rxBufIdx);
  Serial.println(" ...received - Done.");

  if (isValidHeader() && isValidChecksum()) {
    parseState();
  } else {
    clearRxBuf();
  }
}


void loop() {
  io.run();

  delay(5000);
  Serial.println("Handle UART");
  handleUart();

}
