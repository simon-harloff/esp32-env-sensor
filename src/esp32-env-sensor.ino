#include "credentials.h"

#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <U8g2lib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

// OLED display setup
#define OLED_MOSI 23
#define OLED_CLK 18
#define OLED_DC 16
#define OLED_CS 5
#define OLED_RST 17
U8G2_SH1106_128X64_NONAME_1_4W_SW_SPI disp(U8G2_R0, OLED_CLK, OLED_MOSI, OLED_CS, OLED_DC, OLED_RST);

// air quality sensor
#define SDA_PIN 21
#define SCL_PIN 22
Adafruit_BME680 bme;

// sensor oled display vars
const char defaultTemperatureLine[]   = "   T: --.-C";
const char defaultHumidityLine[]      = "   H: --.-%";
const char defaultCO2Line[]           = " CO2: ---.-ppm";
const char defaultTVOCLine[]          = "TVOC: ---.-ppb";
const char temperatureLineTemplate[]  = "   T: %2.1fC";
const char humidityLineTemplate[]     = "   H: %2.1f%%";
const char co2LineTemplate[]          = " CO2: %3.1fppm";
const char tvocLineTemplate[]         = "TVOC: %3.1fppb";

struct BME680Result {
  bool hasValues = false;
  float temperature;
  float humidity;
  float pressure;
  float co2;
  float tvoc;
};

struct CCS811Result {
  bool hasValues = false;
  float co2;
  float tvoc;
};

struct DHT11Result {
  bool hasValues = false;
  float temperature;
  float humidity;
};

// wifi vars
unsigned long wifiConnectAttemptTime = 0;
int wifiStatus = 42;

// mqtt vars
WiFiClient espClient;
PubSubClient mqttClient(espClient);
const char mqttId[] = "esp32_aqi_sensor";
const unsigned int mqttSendInterval = 5000;
unsigned long lastMqttSendTime = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("[setup] begin");

  // initialise air quality sensor
  // Wire.begin(SDA_PIN, SCL_PIN);
  auto bmeResult = bme.begin();
  if (!bmeResult) {
    Serial.println("[setup] error setting up air quality sensor");
  }

  // initialize OLED display
  disp.begin();

  // initialize WiFi
  WiFi.mode(WIFI_STA);

  // initialize MQTT connection
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

  // intialize the BME sensor
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  Serial.println("[setup] done");
}

void loop() {

  // setup wireless connection
  auto wiFiConnected = setupWiFi();
  
  // setup MQTT connection IF wireless is connected
  bool mqttConnected = false;
  if (wiFiConnected) {
    mqttConnected = setupMQTT();
  }

  auto bmeResult = readBME();
  drawStatistics(bmeResult);

  // send results via MQTT if the connection is established
  if (mqttConnected) {
    sendMQTT(bmeResult);
  }

  delay(1000);
}

void sendMQTT(BME680Result bmeResult) {
  
  // if we are not connected to MQTT broker then return
  if (!mqttClient.connected()) return;

  mqttClient.loop();

  // if no results are available from the sensors then return
  if (!bmeResult.hasValues) return;

  if (lastMqttSendTime == 0 || (lastMqttSendTime + mqttSendInterval) < millis()) {
    Serial.println("[runMQTT] Send MQTT reading.");

    if (bmeResult.hasValues) {
      char temperatureString[5];
      snprintf(temperatureString, 5, "%2.1f", bmeResult.temperature);     
      
      char humdityString[5];
      snprintf(humdityString, 5, "%2.1f", bmeResult.humidity);
      
      mqttClient.publish("temperature", temperatureString);
      mqttClient.publish("humidity", humdityString);

      if (false) {
        char co2String[7];
        snprintf(co2String, 7, "%3.1f", bmeResult.co2);
        
        char tvocString[7];
        snprintf(tvocString, 7, "%3.1f", bmeResult.tvoc);

        mqttClient.publish("co2", co2String);
        mqttClient.publish("tvoc", tvocString);
      } 
    }

    lastMqttSendTime = millis();
  }
}

bool setupMQTT() {
  if ( !mqttClient.connected() ) {
    Serial.println("[setupMQTT] MQTT client NOT connected.");
    mqttClient.connect(mqttId, MQTT_USER, MQTT_PASSWORD);
    return false;
  }

  return true;
}

bool setupWiFi() { 
  uint8_t status = WiFi.status();
  
  if (status == WL_CONNECTED) {
    wifiStatus = 113;
    return true;
  } else if (status == WL_DISCONNECTED && (wifiConnectAttemptTime == 0 || millis() > wifiConnectAttemptTime + 30000)) {
    wifiStatus = 108;
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    wifiConnectAttemptTime = millis();
  } else if (status == WL_CONNECT_FAILED) {
    wifiStatus = 33;
  }

  return false;
}

void drawStatistics(BME680Result bmeResult) {
  disp.firstPage();
  do {
    disp.setFont(u8g2_font_8x13B_tf);

    char temperatureLine[sizeof(defaultTemperatureLine)];
    char humidityLine[sizeof(defaultHumidityLine)];
    char co2Line[sizeof(defaultCO2Line)];
    char tvocLine[sizeof(defaultTVOCLine)];
    
    if (bmeResult.hasValues) {
      snprintf(temperatureLine, sizeof(temperatureLine), temperatureLineTemplate, bmeResult.temperature);
      snprintf(humidityLine, sizeof(humidityLine), humidityLineTemplate, bmeResult.humidity);
    } else {
      strncpy(temperatureLine, defaultTemperatureLine, sizeof(temperatureLine));
      strncpy(humidityLine, defaultHumidityLine, sizeof(humidityLine));
    }

    if (false) {
      snprintf(co2Line, sizeof(co2Line), co2LineTemplate, bmeResult.co2);
      snprintf(tvocLine, sizeof(tvocLine), tvocLineTemplate, bmeResult.tvoc);
    } else {
      strncpy(co2Line, defaultCO2Line, sizeof(co2Line));
      strncpy(tvocLine, defaultTVOCLine, sizeof(tvocLine));
    }

    disp.drawStr(0, 12, temperatureLine);
    disp.drawStr(0, 28, humidityLine);
    disp.drawStr(0, 44, co2Line);
    disp.drawStr(0, 60, tvocLine);
    disp.setFont(u8g2_font_iconquadpix_m_all);
    disp.drawGlyph(112, 12, wifiStatus);
  } 
  while ( disp.nextPage() );
}

BME680Result readBME() {
  BME680Result result;

  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println("BME680 Error beginning reading");
    return result;
  }

  if (!bme.endReading()) {
    Serial.println("BME680 Error ending reading");
    return result;
  }

  result.temperature = bme.temperature;
  result.humidity = bme.humidity;
  result.pressure = (bme.pressure / 100.0);
  
  return result;
}

DHT11Result readDHT() {
  DHT11Result result;
  
  // float temperature = dht.readTemperature();
  // float humidity = dht.readHumidity();

  // if (isnan(temperature) && isnan(humidity)) return result;

  // result.hasValues = true;
  // result.temperature = temperature;
  // result.humidity = humidity;
 
  return result;
}

CCS811Result readCCS811() {
  CCS811Result result;
  
  // // if the CCS is not available return
  // if (!ccs.available()) return result;

  // // if there is an error then return
  // if (ccs.readData() != 0) {
  //   Serial.println("CCS811 Error");
  //   return result;
  // }

  // result.hasValues = true;
  // result.co2 = ccs.geteCO2();
  // result.tvoc = ccs.getTVOC();

  return result;
} 

