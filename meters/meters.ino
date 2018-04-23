/** 
 *  Listens for the solar data via a UDP multicast.
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

#define METER_MAX 840
#define METER_3_4 630
#define METER_HALF 420
#define METER_1_4 210

#define METER_MV 0.168F

theapi::GardenPayload rx_payload = theapi::GardenPayload();

WiFiUDP Udp;
// Multicast declarations
IPAddress ipMulti(239, 0, 0, 57);
unsigned int portMulti = 12345;      // local port to listen on
char incomingPacket[255];  // buffer for incoming packets

const byte DEBUG_LED = 16;

void setup() {
  Serial.begin(115200);
  
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
 * Generate the pwm on the pin that's connected to the meter.
 */
void displayReading(uint16_t val, uint8_t pin) {
  uint16_t pwm = round(METER_MV * (float)val);
  Serial.print(pin); Serial.print(" : "); Serial.print(pwm); Serial.println();
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

        displayReading(rx_payload.getVcc(), METER_PIN_VCC);
        displayReading(rx_payload.getChargeMv(), METER_PIN_CHARGE_MV);
        displayReading(rx_payload.getChargeMa(), METER_PIN_CHARGE_MA);
        displayReading(rx_payload.getLight(), METER_PIN_LIGHT);
        displayReading(deg, METER_PIN_TEMPERATURE);

        Serial.println();
      }
    }
  }

}



