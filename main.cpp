#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
// #include <FreeRTOS.h> // included automatically
// #include <task.h> // // included automatically
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
#include <WebServer.h>


// router
#define WIFI_SSID       "Aprian"
#define WIFI_PASSWORD   "telpwifi"
#define MQTT_HOST       "192.168.0.11"

#define MQTT_PORT       1883

#define FORCE_STOP      true
#define NEOPIXEL_PIN    48
#define NUMPIXELS       1

#define TX_PIN 17
#define RX_PIN 18

const char *LWTTopic  = "esp32/status";
const char *DeviceControlTopic  = "esp32/led";
const char *PZEMTopic  = "esp32/pzem";
const char *XYMDTopic  = "esp32/xymd";
const int  maxCycle = 5;

WiFiUDP ntpUDP;
WiFiClient wifiClient;
WebServer server(80);
PubSubClient client(wifiClient);
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 3600, 60000); // +7 jam untuk WIB
TaskHandle_t mqttTaskHandle;
TaskHandle_t blinkRedLEDHandle;
TaskHandle_t getPZEMTask;
TaskHandle_t getXYMDTask;
String header;

uint8_t wait, waitSTA, retry;
uint8_t r = 0, g = 0, b = 0;
bool statusLED = false;

// PZEM-016
float voltage, current, power;
float frequency, powerfactor;
int energy;

// XY-MD02
float temperature, humidity;

// file yang dibutuhkan:
// - esp32s3 basic mqtt pubsub
// - esp32s3 modbus 4     }
// - esp32s3 webserver 2  } esp32s3 modbus webserver

void getPZEM (void *pvParam);
void getXYMD (void *pvParam);

void showRedLED(bool status) {
  if (status) {
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  } else {
    pixels.setPixelColor(0, pixels.Color(0, 0, 0)); // Matikan LED
  }
  pixels.show();
}

void blinkRedLED(void *pvParam) {
  while (true) {  // Loop tak terbatas
    showRedLED(true);
    vTaskDelay(pdMS_TO_TICKS(200));
    showRedLED(false);
    vTaskDelay(pdMS_TO_TICKS(800));
  }
}

void reconnectMQTT() {
  xTaskCreatePinnedToCore(blinkRedLED, "mqttLost", 2048, NULL, 1, &blinkRedLEDHandle, 1);
  while (!client.connected()) {
    long randNum = random(10000, 99999); // angka acak 5 digit
    String clientID = "ESP32S3Client-" + String(randNum);
    Serial.println("MQTT connect...");
    // connect timeout = 3000 ms
    if (client.connect(clientID.c_str(), LWTTopic, 2, true, "offline")) {
      retry = 0;
      timeClient.update();
      time_t rawTime = timeClient.getEpochTime();
      struct tm * timeinfo = localtime(&rawTime);
      char dateStr[11]; // Format: YYYY-MM-DD
      sprintf(dateStr, "%04d-%02d-%02d", 
        timeinfo->tm_year + 1900, 
        timeinfo->tm_mon + 1, 
        timeinfo->tm_mday);
      String todayDate = String(dateStr);
      String LWTmsg = "online-" + todayDate + "-" + timeClient.getFormattedTime();
      Serial.println("MQTT connected");
      client.publish(LWTTopic, LWTmsg.c_str(), true);
      client.subscribe(DeviceControlTopic);
    } else {
      Serial.printf("Gagal connect MQTT, code: %d\n", client.state());
      vTaskDelay(5000 / portTICK_PERIOD_MS); // Non-blocking delay
      retry++;
      if (retry > maxCycle) esp_restart();
    }
  }
  vTaskDelete(blinkRedLEDHandle);
  blinkRedLEDHandle = NULL;
  showRedLED(false);
}

void mqttTask(void * parameter) {
  while (true) {
    if (!client.connected()) {
      reconnectMQTT();
    }
    client.loop(); // penting
    vTaskDelay(100 / portTICK_PERIOD_MS); // beri waktu task lain
  }
}

void onMqttMsg(char* topic, byte* payload, unsigned int len) {
  String message = "";
  for (size_t i = 0; i < len; i++) {
    message += (char)payload[i];
  }

  message.toLowerCase();
  if (String(topic) == DeviceControlTopic) {
    if (message == "red") {
      r = 255; g = 0; b = 0;
    } else if (message == "green") {
      g = 255; r = 0; b = 0;
    } else if (message == "blue") {
      b = 255; r = 0; g = 0;
    } else if (message == "yellow") {
      r = 255; g = 255; b = 0;
    } else if (message == "cyan") {
      g = 255; b = 255; r = 0;
    } else if (message == "magenta") {
      r = 255; b = 255; g = 0;
    } else if (message == "white") {
      r = g = b = 255;
    } else if (message == "off") {
      r = g = b = 0;
    } else {
      Serial.println("Warna tidak dikenali");
    }
    pixels.setPixelColor(0, pixels.Color(r, g, b));
    pixels.show();
  }
  
}

void scanWiFiNetworks() {
  WiFi.scanDelete(); delay(10);
  int n = WiFi.scanNetworks();
  if (n == 0) {
    Serial.println("No Wi-Fi networks found.");
  } else {
    Serial.printf("Found %d WiFi Networks:\n", n);
    for (int i = 0; i < n; ++i) {
      Serial.printf("%d: %s (RSSI: %d) %s\n", 
                    i + 1,
                    WiFi.SSID(i).c_str(),
                    WiFi.RSSI(i),
                    (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "Open" : "Encrypted");
      delay(10);
    }
  } 
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  wait = 0;
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi status: " + String(WiFi.status()));
    vTaskDelay(1000);
    if (WiFi.status() == WL_DISCONNECTED) {
      wait++;
      Serial.println("Still Disconnected: " + String(wait));
      Serial.println();
      if (wait > 9) esp_restart();
    }
  }
  // these rows be executed after wifiEventHandler(SYSTEM_EVENT_STA_GOT_IP)
}

void wifiEventHandler(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      // Program Counter (PC) jumped to right here
      wait = 0;
      Serial.println("WiFi connected.");
      Serial.println("ESP32 got IP Address.");
      Serial.println("IP address:  " + WiFi.localIP().toString());
      Serial.println("Subnet Mask: " + WiFi.subnetMask().toString());
      Serial.println("IP Gateway:  " + WiFi.gatewayIP().toString());
      // back to check WiFi.status() != WL_CONNECTED
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection.");
      // vTaskDelay(pdMS_TO_TICKS(2000));
      waitSTA++;
      Serial.println("Retrying: " + String(waitSTA));
      Serial.println();
      if (waitSTA > 9) esp_restart();
      break;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) ;
  Serial.println("ESP32-S3 Starts...");

  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); 
  while (!Serial1) ;
  
  if (!ModbusRTUClient.begin(9600, SERIAL_8N1)) {
    Serial.println("Failed to start Modbus RTU Client!");
    while (1) ;
  }

  pixels.begin();
  pixels.setBrightness(50); // nilai 0-255, bisa disesuaikan
  pixels.show(); // update LED

  esp_log_level_set("wifi", ESP_LOG_VERBOSE);
  WiFi.setHostname("ESP32-S3");
  WiFi.onEvent(wifiEventHandler);

  Serial.println("Wi-Fi Scanning process...");
  scanWiFiNetworks();
  delay(500);

  connectToWifi();
  delay(500);

  client.setServer((char *)MQTT_HOST, MQTT_PORT);
  client.setCallback(onMqttMsg); // for handling received msg
  client.setKeepAlive(5); // in seconds, default 15s

  timeClient.begin();

  server.on("/", []() {
    String html = R"rawliteral(
    <!DOCTYPE html>
    <html lang="en">
    <head>
      <meta charset="UTF-8">
      <meta http-equiv="refresh" content="5">
      <meta name="viewport" content="width=device-width, initial-scale=1">
      <title>ESP32 Sensor Monitoring</title>
      <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.3/dist/css/bootstrap.min.css" rel="stylesheet">
      <style>
        body { background-color: #f8f9fa; padding-top: 20px; }
        .card { border-radius: 1rem; }
      </style>
    </head>
    <body>
      <div class="container">
        <h2 class="text-center mb-4">Monitoring PZEM-016 & XY-MD02</h2>
        <div class="row g-4">
          <div class="col-md-6">
            <div class="card border-primary shadow">
              <div class="card-header bg-primary text-white text-center fw-bold">
                PZEM-016 (Power Meter)
              </div>
              <div class="card-body">
    )rawliteral";

    // Tambahkan data PZEM
    html += "<p><strong>Voltage:</strong> " + String(voltage, 1) + " V</p>";
    html += "<p><strong>Current:</strong> " + String(current, 3) + " A</p>";
    html += "<p><strong>Active Power:</strong> " + String(power, 1) + " W</p>";
    html += "<p><strong>Energy:</strong> " + String(energy) + " Wh</p>";
    html += "<p><strong>Frequency:</strong> " + String(frequency, 1) + " Hz</p>";
    html += "<p><strong>Power Factor:</strong> " + String(powerfactor, 2) + "</p>";

    html += R"rawliteral(
              </div>
            </div>
          </div>
          <div class="col-md-6">
            <div class="card border-success shadow">
              <div class="card-header bg-success text-white text-center fw-bold">
                XY-MD02 (Temperature & Humidity)
              </div>
              <div class="card-body">
    )rawliteral";

    // // Tambahkan data XY-MD02
    // html += "<p><strong>Temperature:</strong> " + String(temperature, 1) + " &deg;C</p>";
    // html += "<p><strong>Humidity:</strong> " + String(humidity, 1) + " %</p>";

    html += R"rawliteral(
              </div>
            </div>
          </div>
        </div>
      </div>
    </body>
    </html>
    )rawliteral";

    // Kirim ke browser
    server.send(200, "text/html", html);
  });

  server.begin();

  // RTOS / Multi-Threading Task
  xTaskCreatePinnedToCore(mqttTask, "mqttTask", 4096, NULL, 1, &mqttTaskHandle, 0);
  xTaskCreatePinnedToCore(getPZEM, "getPZEM", 4096, NULL, 1, &getPZEMTask, 1);
  // xTaskCreatePinnedToCore(getXYMD, "getXYMD", 4096, NULL, 1, &getXYMDTask, 1);
  
  // already called in reconnectMQTT()
  // xTaskCreatePinnedToCore(blinkRedLED, "mqttLost", 2048, NULL, 1, &blinkRedLEDHandle, 1);

  delay(100); // Jeda sebelum mulai
  xTaskNotifyGive(getPZEMTask);
}

void loop() {
  // put your main code here, to run repeatedly:
  server.handleClient();
}

void getPZEM (void *pvParam) {
  while (true) {
    // Tunggu notifikasi dari task sebelumnya (atau awal mula)
    // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Reset notifikasi setelah diterima
    if (!ModbusRTUClient.requestFrom(1, INPUT_REGISTERS, 0x00, 10)) {
      Serial.print("failed to read registers! ");
      Serial.println(ModbusRTUClient.lastError());
    } else {
      short rawVoltage = ModbusRTUClient.read(); // correct
      short rawCurrentL = ModbusRTUClient.read(); // correct
      short rawCurrentH = ModbusRTUClient.read(); // correct
      short rawPowerL = ModbusRTUClient.read(); // correct
      short rawPowerH = ModbusRTUClient.read(); // correct
      short rawEnergyL = ModbusRTUClient.read(); // correct
      short rawEnergyH = ModbusRTUClient.read(); // correct
      short rawFrequency = ModbusRTUClient.read(); // correct
      short rawPowerFactor = ModbusRTUClient.read(); // correct
      short alarm = ModbusRTUClient.read(); // correct
      voltage = rawVoltage / 10.0;
      current = (((rawCurrentH << 8) << 8) + rawCurrentL) / 1000.0;
      power   = (((rawPowerH << 8) << 8) + rawPowerL) / 10.0;
      energy  = (((rawEnergyH << 8) << 8) + rawEnergyL); // it's uint32_t data type
      frequency = rawFrequency / 10.0;
      powerfactor = rawPowerFactor / 100.0;
      // char payload[128];
      // snprintf(payload, sizeof(payload),
      //        "{\"voltage\":%.1f,\"current\":%.3f,\"power\":%.1f,\"energy\":%d,\"frequency\":%.1f,\"pf\":%.2f}",
      //        voltage, current, power, energy, frequency, powerfactor);

      // client.publish(PZEMTopic, payload, true);
      client.publish("esp32/pzem/voltage", (String(voltage,1) + " V").c_str(), true);
      client.publish("esp32/pzem/current", (String(current,3) + " A").c_str(), true);
      client.publish("esp32/pzem/power", (String(power,1) + " W").c_str(), true);
      client.publish("esp32/pzem/energy", (String(energy) + " Wh").c_str(), true);
      client.publish("esp32/pzem/frequency", (String(frequency,1) + " Hz").c_str(), true);
      client.publish("esp32/pzem/powerfactor", String(powerfactor,2).c_str(), true);

      // Serial.println("Volt: " + String(voltage) + " V");
      // Serial.println("Curr: " + String(current) + " A");
      // Serial.println("Powr: " + String(power) + " W");
      // Serial.println("Engy: " + String(energy) + " Wh");
      // Serial.println("Freq: " + String(frequency) + " Hz");
      // Serial.println("PF  : " + String(powerfactor) + "");
      // Serial.println("Alrm: " + String(alarm) + "");
      // Serial.println();
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    // xTaskNotifyGive(getXYMDTask);
  } // end while()
}

void getXYMD (void *pvParam) {
  while(true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Tunggu notifikasi
    if (!ModbusRTUClient.requestFrom(2, INPUT_REGISTERS, 0x01, 2)) {
      Serial.print("failed to read registers! ");
      Serial.println(ModbusRTUClient.lastError());
    } else {
      short rawtemperature = ModbusRTUClient.read();
      short rawhumidity = ModbusRTUClient.read();
      temperature = rawtemperature / 10.0;
      humidity = rawhumidity / 10.0;
      // char payload[64];
      // snprintf(payload, sizeof(payload),
      //         "{\"temperature\":%.1f,\"humidity\":%.1f}", temperature, humidity);

      // client.publish(XYMDTopic, payload, true);
      client.publish("esp32/xymd/temperature", (String(temperature,1) + " C").c_str(), true);
      client.publish("esp32/xymd/humidity", (String(humidity,1) + " %").c_str(), true);

      // Serial.println("Temp: " + String(temperature) + " C");
      // Serial.println("Humd: " + String(humidity) + " % RH");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    xTaskNotifyGive(getPZEMTask);
  } // end while()
}