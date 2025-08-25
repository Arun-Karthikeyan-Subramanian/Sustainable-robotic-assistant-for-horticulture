#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <Preferences.h>

// WiFi credentials
const char* ssid = "realme 9 5G Speed Edition";
const char* password = "Maakaali";

// Define pins for DHT11
#define DHTPIN 2
#define DHTTYPE DHT11

// Define pins for ultrasonic sensor
#define TRIG_PIN 18
#define ECHO_PIN 19

// Define pin for 2-pin soil moisture sensor
#define SOIL_MOISTURE_PIN 36

// Define pin for IR sensor (object counter)
#define IR_SENSOR_PIN 34

// ✅ Define pin for NEW IR safety stop sensor
#define IR_STOP_PIN 32

// Define pins for servo motors
#define SERVO1_PIN 5
#define SERVO2_PIN 21

// Define pin for motor (through relay or transistor driver)
#define MOTOR_PIN 23  

// Calibration values for soil moisture sensor
const int AIR_VALUE = 4095;
const int WATER_VALUE = 1500;

// Initialize DHT sensor
DHT dht(DHTPIN, DHTTYPE);

// Create WebServer object on port 80
WebServer server(80);

// Create Servo objects
Servo myServo1;
Servo myServo2;

// Preferences object for saving settings
Preferences preferences;

// Structure to hold all sensor data
struct SensorData {
  float temperature;
  float humidity;
  float distance;
  int soilMoistureRaw;
  int soilMoisturePercent;
  unsigned long objectCount;
  int servo1Angle;
  int servo2Angle;
  bool motorState;
};

// Variables for IR sensor
volatile unsigned long totalObjectCount = 0;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;

// Current servo angles
int currentServo1Angle = 90;
int currentServo2Angle = 90;

// Motor state
volatile bool motorState = false;

// Function declarations
void IRAM_ATTR handleIRDetection();
void IRAM_ATTR handleIRStop();   // ✅ new function
unsigned long getObjectCount();
float readUltrasonicDistance();
int readSoilMoisture();
int calculateSoilMoisturePercent(int rawValue);
SensorData readSensorData();
void handleRoot();
void handleSensorData();
void handleServoControl();
void handleMotorControl();
String generateHTML(const SensorData& data);

void setup() {
  Serial.begin(115200);
  dht.begin();

  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize IR sensors
  pinMode(IR_SENSOR_PIN, INPUT_PULLUP);
  pinMode(IR_STOP_PIN, INPUT_PULLUP);  // ✅ new safety stop sensor

  // Motor pin
  pinMode(MOTOR_PIN, OUTPUT);

  // Load last saved states from flash memory
  preferences.begin("settings", false);
  currentServo1Angle = preferences.getInt("servo1", 90);
  currentServo2Angle = preferences.getInt("servo2", 90);
  motorState = preferences.getBool("motor", false);

  // Initialize servos
  myServo1.setPeriodHertz(50);
  myServo1.attach(SERVO1_PIN, 500, 2400);
  myServo1.write(currentServo1Angle);

  myServo2.setPeriodHertz(50);
  myServo2.attach(SERVO2_PIN, 500, 2400);
  myServo2.write(currentServo2Angle);

  // Initialize motor
  digitalWrite(MOTOR_PIN, motorState ? HIGH : LOW);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), handleIRDetection, FALLING);
  attachInterrupt(digitalPinToInterrupt(IR_STOP_PIN), handleIRStop, FALLING); // ✅ stop motor

  // Soil moisture sensor is analog
  analogReadResolution(12);

  // Connect to WiFi
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Set up server routes
  server.on("/", handleRoot);
  server.on("/data", handleSensorData);
  server.on("/servo", handleServoControl);
  server.on("/motor", handleMotorControl);

  // Start server
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}

void handleRoot() {
  SensorData data = readSensorData();
  String html = generateHTML(data);
  server.send(200, "text/html", html);
}

void handleSensorData() {
  SensorData data = readSensorData();
  String json = "{\"temperature\":" + String(data.temperature) + 
                ",\"humidity\":" + String(data.humidity) + 
                ",\"distance\":" + String(data.distance) + 
                ",\"soilMoisturePercent\":" + String(data.soilMoisturePercent) + 
                ",\"objectCount\":" + String(data.objectCount) + 
                ",\"servo1Angle\":" + String(data.servo1Angle) + 
                ",\"servo2Angle\":" + String(data.servo2Angle) + 
                ",\"motorState\":" + String(motorState ? "true" : "false") + "}";
  server.send(200, "application/json", json);
}

void handleServoControl() {
  if (server.hasArg("servo") && server.hasArg("angle")) {
    int servoID = server.arg("servo").toInt();
    int angle = server.arg("angle").toInt();

    if (angle >= 0 && angle <= 180) {
      if (servoID == 1) {
        currentServo1Angle = angle;
        myServo1.write(currentServo1Angle);
        preferences.putInt("servo1", currentServo1Angle);
        server.send(200, "text/plain", "Servo 1 set to " + String(angle) + "°");
      } else if (servoID == 2) {
        currentServo2Angle = angle;
        myServo2.write(currentServo2Angle);
        preferences.putInt("servo2", currentServo2Angle);
        server.send(200, "text/plain", "Servo 2 set to " + String(angle) + "°");
      } else {
        server.send(400, "text/plain", "Invalid servo ID");
      }
    } else {
      server.send(400, "text/plain", "Invalid angle (0-180)");
    }
  } else {
    server.send(400, "text/plain", "Missing parameters: servo & angle");
  }
}

void handleMotorControl() {
  if (server.hasArg("state")) {
    String state = server.arg("state");
    if (state == "on") {
      motorState = true;
      digitalWrite(MOTOR_PIN, HIGH);
      preferences.putBool("motor", motorState);
      server.send(200, "text/plain", "Motor turned ON");
      Serial.println("Motor turned ON");
    } else if (state == "off") {
      motorState = false;
      digitalWrite(MOTOR_PIN, LOW);
      preferences.putBool("motor", motorState);
      server.send(200, "text/plain", "Motor turned OFF");
      Serial.println("Motor turned OFF");
    } else {
      server.send(400, "text/plain", "Invalid motor state");
    }
  } else {
    server.send(400, "text/plain", "Missing parameter: state");
  }
}

SensorData readSensorData() {
  SensorData result;

  result.humidity = dht.readHumidity();
  result.temperature = dht.readTemperature();

  if (isnan(result.humidity) || isnan(result.temperature)) {
    result.humidity = 0;
    result.temperature = 0;
  }

  result.distance = readUltrasonicDistance();
  result.soilMoistureRaw = readSoilMoisture();
  result.soilMoisturePercent = calculateSoilMoisturePercent(result.soilMoistureRaw);
  result.objectCount = getObjectCount();
  result.servo1Angle = currentServo1Angle;
  result.servo2Angle = currentServo2Angle;
  result.motorState = motorState;

  return result;
}

float readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  float distance = duration * 0.0343 / 2;
  return distance;
}

int readSoilMoisture() {
  return analogRead(SOIL_MOISTURE_PIN);
}

int calculateSoilMoisturePercent(int rawValue) {
  int constrainedValue = constrain(rawValue, WATER_VALUE, AIR_VALUE);
  int percent = map(constrainedValue, AIR_VALUE, WATER_VALUE, 0, 100);
  return constrain(percent, 0, 100);
}

void IRAM_ATTR handleIRDetection() {
  unsigned long currentTime = millis();
  if (currentTime - lastDebounceTime > debounceDelay) {
    totalObjectCount++;
    lastDebounceTime = currentTime;
  }
}

// ✅ ISR: stops motor immediately
void IRAM_ATTR handleIRStop() {
  motorState = false;
  digitalWrite(MOTOR_PIN, LOW);
  preferences.putBool("motor", motorState);
  Serial.println("⚠️ Motor stopped by IR safety sensor");
}

unsigned long getObjectCount() {
  noInterrupts();
  unsigned long count = totalObjectCount;
  interrupts();
  return count;
}

String generateHTML(const SensorData& data) {
  String html = "<!DOCTYPE html><html><head>";
  html += "<title>ESP32 Sensor Data</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; margin: 40px; background-color: #f5f5f5; }";
  html += ".container { max-width: 800px; margin: 0 auto; background-color: white; padding: 20px; border-radius: 10px; box-shadow: 0 0 10px rgba(0,0,0,0.1); }";
  html += "h1 { text-align: center; color: #2c3e50; }";
  html += ".card { border: 1px solid #ddd; border-radius: 8px; padding: 20px; margin: 10px; box-shadow: 0 4px 8px 0 rgba(0,0,0,0.1); }";
  html += ".sensor-value { font-size: 24px; font-weight: bold; color: #2c3e50; }";
  html += ".servo-control, .motor-control { background-color: #f9f9f9; padding: 15px; border-radius: 8px; margin-top: 20px; }";
  html += "input[type='number'] { width: 100px; padding: 10px; font-size: 16px; margin: 10px; }";
  html += "button { background-color: #4CAF50; color: white; border: none; padding: 10px 15px; border-radius: 4px; cursor: pointer; }";
  html += "button:hover { background-color: #45a049; }";
  html += ".status { margin-top: 10px; font-weight: bold; }";
  html += "</style></head><body>";
  html += "<div class='container'>";
  html += "<h1>ESP32 Sensor Data, Servo & Motor Control</h1>";

  html += "<div class='card'><h2>Temperature</h2><p class='sensor-value'>" + String(data.temperature) + " °C</p></div>";
  html += "<div class='card'><h2>Humidity</h2><p class='sensor-value'>" + String(data.humidity) + " %</p></div>";
  html += "<div class='card'><h2>Distance</h2><p class='sensor-value'>" + String(data.distance) + " cm</p></div>";
  html += "<div class='card'><h2>Soil Moisture</h2><p class='sensor-value'>" + String(data.soilMoisturePercent) + " %</p></div>";
  html += "<div class='card'><h2>Object Count</h2><p class='sensor-value'>" + String(data.objectCount) + "</p></div>";

  // Servo 1 Control
  html += "<div class='servo-control'>";
  html += "<h2>Servo 1 Control</h2>";
  html += "<p>Current Angle: <span id='servo1Angle'>" + String(data.servo1Angle) + "</span>°</p>";
  html += "<input type='number' id='servo1Input' min='0' max='180' value='" + String(data.servo1Angle) + "'>";
  html += "<button onclick='setServo(1)'>Set Servo 1</button>";
  html += "<p class='status' id='status1'></p>";
  html += "</div>";

  // Servo 2 Control
  html += "<div class='servo-control'>";
  html += "<h2>Servo 2 Control</h2>";
  html += "<p>Current Angle: <span id='servo2Angle'>" + String(data.servo2Angle) + "</span>°</p>";
  html += "<input type='number' id='servo2Input' min='0' max='180' value='" + String(data.servo2Angle) + "'>";
  html += "<button onclick='setServo(2)'>Set Servo 2</button>";
  html += "<p class='status' id='status2'></p>";
  html += "</div>";

  // Motor Control
  html += "<div class='motor-control'>";
  html += "<h2>Motor Control</h2>";
  html += "<p>Current State: <span id='motorState'>" + String(data.motorState ? "ON" : "OFF") + "</span></p>";
  html += "<button onclick='setMotor(\"on\")'>Start Motor</button>";
  html += "<button onclick='setMotor(\"off\")'>Stop Motor</button>";
  html += "<p class='status' id='motorStatus'></p>";
  html += "</div>";

  // JavaScript
  html += "<script>";
  html += "let firstLoad = true;";
  html += "function setServo(id){";
  html += "  let angle = document.getElementById('servo'+id+'Input').value;";
  html += "  fetch('/servo?servo='+id+'&angle='+angle).then(r=>r.text()).then(txt=>{";
  html += "    document.getElementById('status'+id).textContent = txt;";
  html += "    document.getElementById('servo'+id+'Angle').textContent = angle;";
  html += "    setTimeout(()=>{document.getElementById('status'+id).textContent='';},3000);";
  html += "    firstLoad = true;";
  html += "  });";
  html += "}";
  html += "function setMotor(state){";
  html += "  fetch('/motor?state='+state).then(r=>r.text()).then(txt=>{";
  html += "    document.getElementById('motorStatus').textContent = txt;";
  html += "    document.getElementById('motorState').textContent = (state=='on'?'ON':'OFF');";
  html += "    setTimeout(()=>{document.getElementById('motorStatus').textContent='';},3000);";
  html += "  });";
  html += "}";
  html += "setInterval(()=>{fetch('/data').then(r=>r.json()).then(data=>{";
  html += "document.querySelectorAll('.sensor-value')[0].textContent = data.temperature+' °C';";
  html += "document.querySelectorAll('.sensor-value')[1].textContent = data.humidity+' %';";
  html += "document.querySelectorAll('.sensor-value')[2].textContent = data.distance+' cm';";
  html += "document.querySelectorAll('.sensor-value')[3].textContent = data.soilMoisturePercent+' %';";
  html += "document.querySelectorAll('.sensor-value')[4].textContent = data.objectCount;";
  html += "document.getElementById('servo1Angle').textContent = data.servo1Angle;";
  html += "document.getElementById('servo2Angle').textContent = data.servo2Angle;";
  html += "document.getElementById('motorState').textContent = data.motorState ? 'ON' : 'OFF';";
  html += "if(firstLoad){document.getElementById('servo1Input').value = data.servo1Angle; document.getElementById('servo2Input').value = data.servo2Angle; firstLoad=false;}";
  html += "});},2000);";
  html += "</script>";

  html += "</div></body></html>";

  return html;
}
