#include <LittleFS.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

const char* ssid = "david2";
const char* password = "1234567890";

bool ledState = 0;
const int ledPin = 2;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void notifyClients() {
  ws.textAll(String(ledState));
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo* info = (AwsFrameInfo*)arg;

  if (info && info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    if (len < sizeof(data)) {
      data[len] = 0;
    }

    if (strncmp((char*)data, "toggle", 6) == 0) {
      ledState = !ledState;
      notifyClients();  
    } else if (strncmp((char*)data, "setTunings:", 11) == 0) {
      String message = String((char*)data);
      message.remove(0, 11); // Remove "setTunings:" from the string
      int firstCommaIndex = message.indexOf(',');
      int secondCommaIndex = message.indexOf(',', firstCommaIndex + 1);
      if (firstCommaIndex > 0 && secondCommaIndex > firstCommaIndex) {
        double kp = message.substring(0, firstCommaIndex).toDouble();
        double ki = message.substring(firstCommaIndex + 1, secondCommaIndex).toDouble();
        double kd = message.substring(secondCommaIndex + 1).toDouble();
        
        // Adicionando mensagens de depuração para valores recebidos
        Serial.print("Received PID values: ");
        Serial.print("kp: ");
        Serial.print(kp);
        Serial.print(", ki: ");
        Serial.print(ki);
        Serial.print(", kd: ");
        Serial.println(kd);
        
        setTunings(kp, ki, kd);
      }
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

String processor(const String& var){
  if(var == "STATE"){
    return ledState ? "ON" : "OFF";
  }
  return String();
}

unsigned long lastTime = 0;
double u = 0, y = 0, sp; // u: input, y: output (and sensor measure), sp: setpoint
double iTerm = 0, lastY = 0;
double kp, ki, kd;
unsigned long sampleTime = 200; // 200 ms or 0.2 sec
double uMin, uMax; // control signal saturation
double voltageY;

double VCC = 3.3; // For ESP32 3.3V
int yPin = 34; 
int uPin = 25; // PWM or DAC0 (25) and DAC1 (26)
static const uint16_t RESOLUTION = 4096; // also used for the maximum setpoint
static const uint16_t TIME_TO_DISCHARGE = 8000;

void computeU() {
    unsigned long now = millis();
    int timeChange = (now - lastTime);
    if (timeChange >= sampleTime) {
        double e = sp - y;
        iTerm += (ki * e);
        if (iTerm > uMax) iTerm = uMax;
        else if (iTerm < uMin) iTerm = uMin;
        double dY = (y - lastY);
        u = kp * e + iTerm - kd * dY;
        if (u > uMax) u = uMax;
        else if (u < uMin) u = uMin;
        lastY = y;
        lastTime = now;

        // Adicione prints para depuração
    
        Serial.print("Control signal (u): "); Serial.println(u);
        
    }
}


void setTunings(double kP, double kI, double kD) {
    double sampleTimeInSec = ((double)sampleTime / 1000);
    kp = kP;
    ki = kI * sampleTimeInSec;
    kd = kD / sampleTimeInSec;
}

void setSampleTime(int newSampleTime) {
    if (newSampleTime > 0) {
        double ratio = (double)newSampleTime / (double)sampleTime;
        ki *= ratio;
        kd /= ratio;
        sampleTime = (unsigned long)newSampleTime;
    }
}

void setControlLimits(double min, double max) {
    if (min > max) return;
    uMin = min;
    uMax = max;
    if (u > uMax) u = uMax;
    else if (u < uMin) u = uMin;
    if (iTerm > uMax) iTerm = uMax;
    else if (iTerm < uMin) iTerm = uMin;
}

void setup() {
  Serial.begin(115200);

  if (!LittleFS.begin()) {
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }

  File file = LittleFS.open("/index.html", "r");
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.println("File Content:");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println(WiFi.localIP());

  initWebSocket();
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html", false, processor);
  });
  server.begin();

  sp = 0.5 * RESOLUTION; // considering 0-4095 (50%)
  pinMode(uPin, OUTPUT);
  pinMode(yPin, INPUT);
  setControlLimits(0, 255); // 255 = 3.3V DAC
  setSampleTime(sampleTime);
  setTunings(0.5, 0.4, 0);
  dacWrite(uPin, 0);


  Serial.println("kp: ");
        Serial.print(kp);
        Serial.print(", ki: ");
        Serial.print(ki);
        Serial.print(", kd: ");
        Serial.println(kd);
  vTaskDelay(pdMS_TO_TICKS(TIME_TO_DISCHARGE)); // wait capacitor discharge
}

void loop() {
  ws.cleanupClients();
  digitalWrite(ledPin, ledState);
  y = analogRead(yPin);
  voltageY = (VCC / RESOLUTION) * y;
  computeU();
  dacWrite(uPin, u); // ESP32 pure DAC

  // Debugging output
  Serial.print("Sending U: "); Serial.print(u);
  Serial.print(", Setpoint: "); Serial.print(((sp * VCC) / RESOLUTION));
  Serial.print(", VoltageY: "); Serial.println(voltageY);

  // Sending U, setpoint, and voltageY values for chart update
  ws.textAll("plotData:" + String(u) + "," + String(((sp * VCC) / RESOLUTION)) + "," + String(voltageY));

  delay(100); // Adjusted delay for more reasonable data updates

  // Setpoint profile
  unsigned long t = millis();
  if (t >= TIME_TO_DISCHARGE + 5000 && t < TIME_TO_DISCHARGE + 2 * 5000) {
    sp = 0.8 * RESOLUTION;
  } else if (t >= TIME_TO_DISCHARGE + 2 * 5000) {
    sp = 0.2 * RESOLUTION;
  }
}


float float_map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
