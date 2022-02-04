#include "credentials.h"

#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <U8g2lib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <Adafruit_CCS811.h>

// OLED display setup
#define OLED_MOSI 23
#define OLED_CLK 18
#define OLED_DC 16
#define OLED_CS 5
#define OLED_RST 17
U8G2_SH1106_128X64_NONAME_1_4W_SW_SPI disp(U8G2_R0, OLED_CLK, OLED_MOSI, OLED_CS, OLED_DC, OLED_RST);

// BME680 sensor
Adafruit_BME680 bme;

// CCS811 sensor
Adafruit_CCS811 ccs;

// sensor oled display vars
const char defaultTemperatureLine[]   = "   T: --.-C";
const char defaultHumidityLine[]      = "   H: --.-%";
const char defaultPressureLine[]      = "   P: ---.-hPa";
const char defaultCO2Line[]           = " CO2: ---.-ppm";
const char defaultTVOCLine[]          = "TVOC: ---.-ppb";
const char temperatureLineTemplate[]  = "   T: %2.1fC";
const char humidityLineTemplate[]     = "   H: %2.1f%%";
const char pressureLineTemplate[]     = "   P: %3.1fhPa";
const char co2LineTemplate[]          = " CO2: %3.1fppm";
const char tvocLineTemplate[]         = "TVOC: %3.1fppb";

struct BME680Result {
  bool hasValues = false;
  float temperature;
  float humidity;
  float pressure;
};

struct CCS811Result {
  bool hasValues = false;
  float co2;
  float tvoc;
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

  // intialize the CCS811 sensor
  auto ccsResult = ccs.begin(0x5A);
  if (!ccsResult) {
    Serial.println("[setup] CCS811 error");
  }

  // initialise BME680 sensor
  auto bmeResult = bme.begin(0x76);
  if (!bmeResult) {
    Serial.println("[setup] BME680 error");
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

  auto bmeResult = readBME680();
  auto ccsResult = readCCS811();
  drawStatistics(bmeResult, ccsResult);

  // send results via MQTT if the connection is established
  if (mqttConnected) {
    sendMQTT(bmeResult, ccsResult);
  }

  delay(1000);
}

void sendMQTT(BME680Result bmeResult, CCS811Result ccsResult) {
  
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

      char pressureString[5];
      snprintf(pressureString, 5, "%3.1f", bmeResult.pressure);
      
      mqttClient.publish("temperature", temperatureString);
      mqttClient.publish("humidity", humdityString);
      mqttClient.publish("pressure", pressureString);
    }

    if (ccsResult.hasValues) {
      char co2String[7];
      snprintf(co2String, 7, "%3.1f", ccsResult.co2);
      
      char tvocString[7];
      snprintf(tvocString, 7, "%3.1f", ccsResult.tvoc);

      mqttClient.publish("co2", co2String);
      mqttClient.publish("tvoc", tvocString);
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

void drawStatistics(BME680Result bmeResult, CCS811Result ccsResult) {
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

    if (ccsResult.hasValues) {
      snprintf(co2Line, sizeof(co2Line), co2LineTemplate, ccsResult.co2);
      snprintf(tvocLine, sizeof(tvocLine), tvocLineTemplate, ccsResult.tvoc);
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

BME680Result readBME680() {
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

  result.hasValues = true;
  result.temperature = bme.temperature;
  result.humidity = bme.humidity;
  result.pressure = (bme.pressure / 100.0);

  // Serial.printf("BME680 result - temp: %2.1f; humid: %2.1f; hPA: %3.1f\n", result.temperature, result.humidity, result.pressure);
  
  return result;
}

CCS811Result readCCS811() {
  CCS811Result result;
  
  // if the CCS is not available return
  if (!ccs.available()) return result;

  // if there is an error then return
  if (ccs.readData() != 0) {
    Serial.println("CCS811 Error");
    return result;
  }

  result.hasValues = true;
  result.co2 = ccs.geteCO2();
  result.tvoc = ccs.getTVOC();

  return result;
} 

