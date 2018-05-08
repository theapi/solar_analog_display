/** 
 *  Listens for the solar data via a UDP multicast.
 *  
 *  UDP broadcast comes from a system running https://github.com/theapi/solar
 *
 *  Uses the library from https://github.com/theapi/solar
 *  And https://github.com/theapi/denbit
 */


#include <ESP8266WiFi.h>
#include <WiFiUdp.h>


#include "Payload.h"
#include "GardenPayload.h"

// Include the Denbit library.
#include <Denbit.h>
// Initialize the denbit.
Denbit denbit;

#define METER_PIN_VCC         15 // D8
#define METER_PIN_CHARGE_MV   13 // D7
#define METER_PIN_CHARGE_MA   12 // D6
#define METER_PIN_TEMPERATURE  5 // D1
#define METER_PIN_LIGHT        4 // D2

#define METER_MV 0.17F // 850 / 5000
#define METER_VCC 0.17F // 850 / 5000
#define METER_MA 7.0F   // 700 / 100
#define METER_DEG 17.6F // 880 / 50 (actually -10 to 40)
#define METER_LIGHT 0.225F // 900 / 4000

theapi::GardenPayload rx_payload = theapi::GardenPayload();

WiFiUDP Udp;
// Multicast declarations
IPAddress ipMulti(239, 0, 0, 57);
unsigned int portMulti = 12345;      // local port to listen on
char incomingPacket[255];  // buffer for incoming packets

const byte DEBUG_LED = 16;

void setup() {
  Serial.begin(115200);
  analogWriteFreq(5000);
  
  // Start the Over The Air programming.
  denbit.OTAsetup();

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Udp.beginMulticast(WiFi.localIP(), ipMulti, portMulti);
  Serial.printf("Now listening to IP %s, UDP port %d\n", ipMulti.toString().c_str(), portMulti);
  
  pinMode(DEBUG_LED, OUTPUT);
  digitalWrite(DEBUG_LED, HIGH);  // LOW = ON
}

/**
 * Generate the pwm for the millivolts reading.
 */
void displayMv(uint16_t val, uint8_t pin) {
  uint16_t pwm = round(METER_MV * (float)val);
  analogWrite(pin, pwm);
}

void displayVcc(uint16_t val, uint8_t pin) {
  uint16_t pwm = round(METER_VCC * (float)val);
  analogWrite(pin, pwm);
}

/**
 * Generate the pwm for the milliamps reading.
 */
void displayMa(uint16_t val, uint8_t pin) {
  uint16_t pwm = round(METER_MA * (float)val);
  analogWrite(pin, pwm);
}

/**
 * Generate the pwm for the temperature reading.
 */
void displayDeg(float deg, uint8_t pin) {
  deg = deg + 10; // The meter starts a -10
  uint16_t pwm = round(METER_DEG * deg);
  analogWrite(pin, pwm);
}

/**
 * Generate the pwm for the light.
 */
void displayLight(uint16_t val, uint8_t pin) {
  uint16_t pwm = round(METER_LIGHT * (float)val);
  //Serial.print(pin); Serial.print(" : "); Serial.print(pwm); Serial.println();
  analogWrite(pin, pwm);
}

void loop() {
  // Check for any Over The Air updates.
  denbit.OTAhandle();

  // Check for udp data. 
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    // receive incoming UDP packets
    Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read(incomingPacket, 255);
    if (len > 0)  {
      incomingPacket[len] = 0;
    }
    //Serial.printf("UDP packet contents: %s\n", incomingPacket);
    // Check for payload signifier;
    if (incomingPacket[0] == '\t') {
      uint8_t payload_buf[rx_payload.size()];
      memcpy(payload_buf, incomingPacket+1, rx_payload.size());
      rx_payload.unserialize(payload_buf);
      if (rx_payload.getMsgType() == theapi::Payload::GARDEN) {
        Serial.print("GARDEN: ");
        Serial.print(rx_payload.getMsgType()); Serial.print(", ");
        Serial.print(rx_payload.getMsgId()); Serial.print(", ");
        Serial.print(rx_payload.getVcc()); Serial.print(", ");
        Serial.print(rx_payload.getChargeMv()); Serial.print(", ");
        Serial.print(rx_payload.getChargeMa()); Serial.print(", ");
        Serial.print(rx_payload.getLight()); Serial.print(", ");
        Serial.print(rx_payload.getCpuTemperature()); Serial.print(", ");
        // Convert the temperature to a float.
        float deg = (float) rx_payload.getTemperature() / 10.0;
        Serial.println(deg);

        displayVcc(rx_payload.getVcc(), METER_PIN_VCC);
        displayMv(rx_payload.getChargeMv(), METER_PIN_CHARGE_MV);
        displayMa(rx_payload.getChargeMa(), METER_PIN_CHARGE_MA);
        displayLight(rx_payload.getLight(), METER_PIN_LIGHT);
        displayDeg(deg, METER_PIN_TEMPERATURE);

        Serial.println();
      }
    }
  }

}



