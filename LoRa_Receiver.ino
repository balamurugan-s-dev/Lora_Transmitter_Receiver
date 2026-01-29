#include <WiFi.h>
#include <WebSocketsClient_Generic.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <HardwareSerial.h>


// Wifi and Server ip variables
const char* ssid = "Oppo";
const char* password = "bala13570";
const char* serverip = "10.219.2.124";
const uint16_t serverPort = 8080;

HardwareSerial lora(2);
WebSocketsClient webSocket;
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /*reset=*/ U8X8_PIN_NONE);

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length);

// WiFi handler
void WiFiHandler(){
  WiFi.begin(ssid, password);
  Serial.println("Connecting to the WiFi");

  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

}

// LoRa Helper
void sendATCommand(String cmd){
  delay(1000);
  lora.println(cmd);
  delay(500);
  while(lora.available()){
    Serial.write(lora.read());
  }
}


#define RXD2 16
#define TXD2 17
// Main Setup
void setup() {
  Serial.begin(115200);

  lora.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial.println("AT Command Setup for LoRa...");
  sendATCommand("AT");           // basic check
  // sendATCommand("AT+RESET");     // reset module
  // sendATCommand("AT+MODE=0");    // AT mode
  // sendATCommand("AT+NETWORKID=5"); // network must match receiver
  // sendATCommand("AT+ADDRESS=1"); // this transmitter's address
  // sendATCommand("AT+BAND=868500000"); // optional: frequency
  // sendATCommand("AT+MODE=2");
  Serial.println("LoRa AT transmitter ready.");

  u8g2.begin();
  WiFiHandler();

  // Initializing the WebSocket
  webSocket.begin(serverip, serverPort, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(3000);
}


// Send message to WebSocket server
void sendToServer(String msg) {
  if (webSocket.isConnected()) {
    webSocket.sendTXT(msg);
    Serial.println("→ Sent to server: " + msg);
  } else {
    Serial.println("⚠️ WebSocket not connected");
  }
}


String ledstate = "";
void ledStatus(String str){
  ledstate = "LED Status : " +str;
}

void handleLoRaData() {
  static String received = ""; // store incoming characters

  while (lora.available()) {
    char c = lora.read();

    if (c == '\n' || c == '\r') {
      if (received.length() > 0) {
        received.trim(); // remove extra spaces/newlines

        // Expecting format: +RCV=2,11,30.80,61.00,-b2,11
        int firstComma  = received.indexOf(',');
        int secondComma = received.indexOf(',', firstComma + 1);
        int thirdComma  = received.indexOf(',', secondComma + 1);
        int fourthComma = received.indexOf(',', thirdComma + 1);

        if (thirdComma > 0 && fourthComma > 0) {
          String tempStr = received.substring(secondComma + 1, thirdComma);
          String humStr  = received.substring(thirdComma + 1, fourthComma);

          tempStr.trim(); humStr.trim();

          float temp = tempStr.toFloat();
          float hum  = humStr.toFloat();

          Serial.print("Temperature: "); Serial.print(temp);
          Serial.print(" °C, Humidity: "); Serial.println(hum);

          // Display on OLED
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_ncenB08_tr);
          String tempLine = "Temp : " + String(temp, 1) + " C";
          String humLine  = "Humi : " + String(hum, 1) + " %";
          u8g2.drawStr(0, 12, tempLine.c_str());
          u8g2.drawStr(0, 28, humLine.c_str());
          u8g2.drawStr(0, 44, ledstate.c_str());
          u8g2.sendBuffer();

          // Send to WebSocket server
          String payload = String(temp, 2) + "," + String(hum, 2);
          sendToServer(payload);

          lora.println("AT+SEND=2,5,0,ACK");
        }

        received = ""; // clear buffer for next line
      }
    } else {
      received += c; // accumulate characters
    }
  }
}



unsigned long lastPing = 0;
void loop() {
  webSocket.loop();
  handleLoRaData();

  if(millis() - lastPing > 30000){
    webSocket.sendPing();
    lastPing = millis();
  }
}


// WebSocket Event Handler
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length){
  switch(type){
    case WStype_CONNECTED:
      Serial.println("ESP32 is Connected to the Websocket Server");
      break;

    case WStype_TEXT: {
      String command = String((char*)payload);
      command.trim();
      Serial.println("← Command from server: " + command);

      // Forward the command to Nano via LoRa
      // Example: LED_ON or LED_OFF
      String ledPayload = (command == "ON") ? "1,1" : "1,0";
      String ledCmd = "AT+SEND=2," + String(ledPayload.length()) + "," + ledPayload;
      lora.println(ledCmd);
      ledStatus(command);
      break;
    }

    case WStype_DISCONNECTED:
      Serial.println("ESP32 is Disconnected from the Server");
      break;

    default:
      break;
  }
}