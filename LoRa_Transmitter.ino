#include <SoftwareSerial.h>
#include "DHT.h"

#define DHTPIN 4
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);
SoftwareSerial lora(2, 3); // RX, TX for LoRa module

#define RECEIVER_ADDR 1  // ESP32 receiver address


// Helper to send AT commands and print responses
void sendATCommand(String cmd, int waitTime = 500) {
  lora.println(cmd);
  delay(waitTime);
  while (lora.available()) {
    char c = lora.read();
    Serial.write(c);
  }
}


void setup() {
  Serial.begin(115200);
  dht.begin();
  lora.begin(115200); // match ESP32 LoRa baud

  Serial.println("Transmitter starting...");

  // Initialize LoRa module in AT mode
  sendATCommand("AT");           // basic check
  sendATCommand("AT+RESET");     // reset module
  sendATCommand("AT+MODE=1");    // AT mode
  sendATCommand("AT+NETWORK=0"); // network must match receiver
  sendATCommand("AT+ADDRESS=2"); // this transmitter's address
  sendATCommand("AT+BAND=868500000"); // optional: frequency
  sendATCommand("AT+IPR=115200");      // optional: baud

  Serial.println("LoRa AT transmitter ready.");
}



void loop() {
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  if (!isnan(temp) && !isnan(hum)) {
    String payload = String(temp) + "," + String(hum);

    // Send payload to receiver address
    String cmd = "AT+SEND=" + String(RECEIVER_ADDR) + "," + String(payload.length()) + "," + payload;
    lora.println(cmd);

    Serial.println("Sent: " + payload);
  } else {
    Serial.println("Failed to read DHT11");
  }

  delay(5000); // send every 5 seconds
}