#include <WiFi.h>
#include <ESPAsyncWebSrv.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>  // Include SPIFFS library
#include <ArduinoOTA.h> // OTA updates

// ------------------ Wi-Fi Credentials ------------------
// Uncomment the following line to hardcode your Wi-Fi credentials
#include "credentials.h" 
// -------------------------------------------------------

// Stepper Motor Pin Definitions (TMC2209 Drivers)
#define M0_STEP_PIN 26
#define M0_DIR_PIN 27

#define M1_STEP_PIN 33
#define M1_DIR_PIN 25

#define M2_STEP_PIN 21 //35 ONLY INPUT, NO PULLUP RESISTOR FOR OUTPUT :(
//Replacing GPIO35 with GPIO21, then bridging the pins in hardware
#define M2_DIR_PIN 32


//originally M4
#define M3_STEP_PIN 4
#define M3_DIR_PIN 2
//originally M5
#define M4_STEP_PIN 17
#define M4_DIR_PIN 16

//originally M3, changed because of unfortunate pin selection (output only)
#define M5_STEP_PIN 39 //39 ONLY INPUT, NO PULLUP RESISTOR FOR OUTPUT :(
//Potentially replacing GPIO39 with GPIO1, then bridging the pins in hardware
#define M5_DIR_PIN 34 //34 ONLY INPUT, NO PULLUP RESISTOR FOR OUTPUT :(

// WHardcode i-Fi Credentials
// const char* ssid = "Wifi_SSID";
// const char* password = "Wifi_Password";

// Create an AsyncWebServer object on port 80
AsyncWebServer server(80);

// Stepper motor control variables
bool motorRunning[6] = {false, false, false, false, false, false};
bool motorDirection[6] = {true, true, true, true, true, true}; // true = forward, false = reverse
int stepDelay[6] = {500, 500, 500, 500, 500, 500}; // Initial speed of each motor in microseconds
TaskHandle_t stepperTaskHandle[6] = {NULL, NULL, NULL, NULL, NULL, NULL}; // Handle for each motor task

// Function to step the motor in a separate task for each motor
void stepperTask(void *parameter) {
  int motorIndex = *(int*)parameter;
  int stepPin = motorIndex == 0 ? M0_STEP_PIN : motorIndex == 1 ? M1_STEP_PIN : 
                motorIndex == 2 ? M2_STEP_PIN : motorIndex == 3 ? M3_STEP_PIN : 
                motorIndex == 4 ? M4_STEP_PIN : M5_STEP_PIN;
                
  while (true) {
    if (motorRunning[motorIndex]) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(stepDelay[motorIndex]); // Control speed by adjusting the delay
      digitalWrite(stepPin, LOW);
      delayMicroseconds(stepDelay[motorIndex]);
    } else {
      delay(100); // Small delay when not running
    }
  }
}

void setup() {
  // Set up the TMC2209 control pins
  pinMode(M0_STEP_PIN, OUTPUT); pinMode(M0_DIR_PIN, OUTPUT);
  pinMode(M1_STEP_PIN, OUTPUT); pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M2_STEP_PIN, OUTPUT); pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(M3_STEP_PIN, OUTPUT); pinMode(M3_DIR_PIN, OUTPUT);
  pinMode(M4_STEP_PIN, OUTPUT); pinMode(M4_DIR_PIN, OUTPUT);
  pinMode(M5_STEP_PIN, OUTPUT); pinMode(M5_DIR_PIN, OUTPUT);

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Set up OTA updates
  ArduinoOTA.begin();

  // Create a task for each motor
  for (int i = 0; i < 6; i++) {
    xTaskCreate(stepperTask, "Stepper Task", 1000, (void*)&i, 1, &stepperTaskHandle[i]);
    delay(100); // Delay to avoid race conditions while passing the motor index
  }

  // Serve the web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });

  // Motor control routes for each motor (forward, backward, stop, speed)
  // Replace instances where String concatenation is used with .c_str() to convert to const char*

for (int i = 0; i < 6; i++) {
    // Handle forward movement
    server.on((String("/forward") + i).c_str(), HTTP_GET, [i](AsyncWebServerRequest *request) {
        motorDirection[i] = true; // Set direction to forward
        digitalWrite(i == 0 ? M0_DIR_PIN : i == 1 ? M1_DIR_PIN : i == 2 ? M2_DIR_PIN :
                     i == 3 ? M3_DIR_PIN : i == 4 ? M4_DIR_PIN : M5_DIR_PIN, HIGH);
        motorRunning[i] = true; // Start motor
        request->send(200, "text/plain", "Motor " + String(i) + " moving forward");
    });

    // Handle backward movement
    server.on((String("/backward") + i).c_str(), HTTP_GET, [i](AsyncWebServerRequest *request) {
        motorDirection[i] = false; // Set direction to backward
        digitalWrite(i == 0 ? M0_DIR_PIN : i == 1 ? M1_DIR_PIN : i == 2 ? M2_DIR_PIN :
                     i == 3 ? M3_DIR_PIN : i == 4 ? M4_DIR_PIN : M5_DIR_PIN, LOW);
        motorRunning[i] = true; // Start motor
        request->send(200, "text/plain", "Motor " + String(i) + " moving backward");
    });

    // Handle motor stop
    server.on((String("/stop") + i).c_str(), HTTP_GET, [i](AsyncWebServerRequest *request) {
        motorRunning[i] = false; // Stop motor
        request->send(200, "text/plain", "Motor " + String(i) + " stopped");
    });

    // Handle speed setting
    server.on((String("/setSpeed") + i).c_str(), HTTP_GET, [i](AsyncWebServerRequest *request) {
        if (request->hasParam("value")) {
            String speedValue = request->getParam("value")->value();
            stepDelay[i] = speedValue.toInt(); // Update motor speed
            Serial.println("Motor " + String(i) + " speed updated to: " + String(stepDelay[i]) + " μs");
            request->send(200, "text/plain", "Speed updated to: " + String(stepDelay[i]) + " μs");
        } else {
            request->send(400, "text/plain", "Speed value missing");
        }
    });
}

  // Start the server
  server.begin();
}

void loop() {
  // Handle OTA updates
  ArduinoOTA.handle();
}
