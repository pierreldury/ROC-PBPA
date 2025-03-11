/* AIR_PURIFIER_SENSORS
 * Version : 1.0.0
 * Author : PBPA
 * Last Modified by : Pierre Dury
 * Modification details (date format dd/mm/yyyy):
 *  - 10/03/2025 : Initial version
 */

#include <SoftwareSerial.h>
#include <DHT.h>
#include <TimeLib.h>

// ----- CONFIGURATION -----
#define DHTPIN 5           // Digital pin connected to the DHT22 data pin
#define LED_R 7      // RED LED
#define LED_B 3      // BLUE LED PIN
#define LED_V 2      // GREEN LED
#define BUTTON_PIN 6      // BUTTON PIN230
#define DHTTYPE DHT22      // DHT 22 (AM2302)
DHT dht(DHTPIN, DHTTYPE);

// Use SoftwareSerial for the WiFi module (adjust pins as needed)
#define WIFI_RX 8         // Arduino pin connected to WiFi module TX
#define WIFI_TX 9         // Arduino pin connected to WiFi module RX
SoftwareSerial wifiSerial(WIFI_RX, WIFI_TX);

// Devide ID defined by developer
const char* device_id = "AD001";

// Replace with your WiFi network credentials
const char* ssid     = "S24 Ultra de Paul";
const char* password = "adminadmin";

// Raspberry Pi port and Address
const char* serverIP   = "192.168.155.158"; // Update with your Raspberry Pi IP address
const int   serverPort = 5000;
const char* serverPath = "/sensor-data";

// Timing intervals
unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 5000; // Publish every 5 seconds

// Led configurations
bool green_led_activated = false;
unsigned long tempsActivation = 0;
bool buttonState = LOW;
bool lastButtonState = LOW;

// Function to send an AT command
void sendATCommand(const char* cmd, const char* expectedResponse = "OK", unsigned long timeout = 5000) {
  wifiSerial.println(cmd);
  unsigned long startTime = millis();
  String response = "";
  while (millis() - startTime < timeout) {
    if (wifiSerial.available()) {
      char c = wifiSerial.read();
      response += c;
      if (response.endsWith(expectedResponse)) {
        Serial.print("Command OK: ");
        Serial.println(cmd);
        return;
      }
    }
  }
  Serial.print("Command Failed: ");
  Serial.println(cmd);
  Serial.print("Response: ");
  Serial.println(response);
}

// Function to send HTTP POST request to the Raspberry Pi server
bool sendHttpPost(String payload) {
  static unsigned long sendStartTime = 0;
  static bool isSending = false;
  static String currentPayload = "";
  static int step = 0;

  if (!isSending) {
    // Initialize sending process
    isSending = true;
    currentPayload = payload;
    step = 0;
    sendStartTime = millis();
  }

  // Build TCP connection to server IP on server port with a 5 second timeout
  if (step == 0){
    String connectCmd = "AT+CIPSTART=\"TCP\",\"";
    connectCmd += serverIP;
    connectCmd += "\",";
    connectCmd += String(serverPort);
    sendATCommand(connectCmd.c_str(), "OK", 5000);
    step++;
  }

  // Send the HTTP request
  if (step == 1) {
  // Build HTTP POST request
    String httpRequest = "POST " + String(serverPath) + " HTTP/1.1\r\n" +
                      "Host: " + String(serverIP) + "\r\n" +
                      "Content-Type: application/json\r\n" +
                      "Content-Length: " + String(payload.length()) + "\r\n\r\n" +
                      payload;


    Serial.println("HTTP Request:");
    Serial.println(httpRequest);

    // Send the HTTP request
    String httpcmd = "AT+CIPSEND=";
    httpcmd += String(httpRequest.length());
    httpcmd += "\r\n\r\n";
    httpcmd += "> ";
    sendATCommand(httpcmd.c_str());
    wifiSerial.print(httpRequest);
    step++;
  }

  // Optionally, read and print server response
  while (wifiSerial.available()) {
    Serial.write(wifiSerial.read());
  }

  // Check for response or timeout
  if (step == 2) {
    if (wifiSerial.available()) {
      Serial.write(wifiSerial.read());
    }
    if (millis() - sendStartTime >= 3000) {
      // Close the connection
      sendATCommand("AT+CIPCLOSE", "OK", 2000);
      isSending = false;
      return true;
    }
  }

  return false;
}

void setup(){
  Serial.begin(115200);
  wifiSerial.begin(115200);
  dht.begin();

  pinMode(LED_R, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(LED_V, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

  Serial.println("Initializing WiFi module...");
  // Reset the module
  sendATCommand("AT+RST");
  delay(500);
  sendATCommand("ATE0"); // Disable Echo commands
  delay(500);
  sendATCommand("AT+CIPMUX=0"); // Set to Single connection mode
  delay(500);
  // Set WiFi mode to station
  sendATCommand("AT+CWMODE=1");
  
  // Connect to WiFi
  String cmd = "AT+CWJAP=\"" + String(ssid) + "\",\"" + String(password) + "\"";
  sendATCommand(cmd.c_str(), "OK", 15000);
  Serial.println("WiFi connected!");
  delay(500);

  Serial.println("Activating LED...!");
  // Turn on the Red LED to indicate the board is online
  digitalWrite(LED_R, HIGH);
}

void loop() {
  static unsigned long lastBlinkTime = 0;
  static bool ledState = LOW;
  static bool isBlinking = false;
  static unsigned long blinkStartTime = 0;
  static bool isSendingData = false;
  static unsigned long sendStartTime = 0;

  // Read sensor data every publishInterval
  if (millis() - lastPublishTime > publishInterval && !isSendingData) {
    lastPublishTime = millis();

    // Read temperature and humidity from DHT22
    float temperature = dht.readTemperature();
    float humidity    = dht.readHumidity();

    // Check if any reads failed and exit early
    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }

    // Create JSON payload
    String payload = "{\"deviceId\":\"" + String(device_id) + "\", \"temperature\":" + String(temperature) + ", \"humidity\":" + String(humidity) + ", \"timestamp\":" + String(millis()) + "}";

    Serial.print("Publishing: ");
    Serial.println(payload);

    // Start blinking the Blue LED for 5 seconds
    isBlinking = true;
    blinkStartTime = millis();

    // Start sending data
    isSendingData = true;
    sendStartTime = millis();

    // Send the JSON payload via HTTP POST to the Raspberry Pi server
    if (!sendHttpPost(payload)) {
      // Keep calling sendHttpPost until it returns true
      Serial.println("HTTP POST in progress...");
    } else {
      Serial.println("HTTP POST succeeded");
    }

    // Turn off the Blue LED after the request is sent
    digitalWrite(LED_B, LOW);

    // Check temperature and humidity conditions to activate the Green LED
    if (temperature > 28.0 || humidity > 40.0) {
      green_led_activated = true;
    } else {
      green_led_activated = false;
    }

    // Update the Green LED state
    digitalWrite(LED_V, green_led_activated ? HIGH : LOW);
  }


  // Blink the Blue LED for 5 seconds
  if (isBlinking) {
    if (millis() - blinkStartTime < 5000) {
      if (millis() - lastBlinkTime >= 1000) {
        lastBlinkTime = millis();
        ledState = !ledState;
        digitalWrite(LED_B, ledState);
      }
    } else {
      // Stop blinking after 5 seconds
      isBlinking = false;
      digitalWrite(LED_B, LOW);
    }
  }

// Check if data sending is complete
  if (isSendingData && millis() - sendStartTime >= 3000) {
    isSendingData = false;
  }

  // Check button state to toggle the Green LED
  buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == HIGH && lastButtonState == LOW) {
    green_led_activated = !green_led_activated;
    digitalWrite(LED_V, green_led_activated ? HIGH : LOW);
    delay(50); // Debounce delay
  }
  lastButtonState = buttonState;

  // Optionally, print any incoming data from the WiFi module for debugging
  if (wifiSerial.available()) {
    Serial.write(wifiSerial.read());
  }
}