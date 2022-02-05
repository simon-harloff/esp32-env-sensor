#include "config.h"

#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <U8g2lib.h>
#include <Adafruit_Sensor.h>
#include <bsec.h>

struct BME680Result {
  bool hasValues = false;
  float temperature;
  float humidity;
  float pressure;
  float gasResistance;
  float iaq;
  int iaqAccuracy;
  float co2;
  float voc;
};

// OLED display setup
#define OLED_MOSI 23
#define OLED_CLK 18
#define OLED_DC 16
#define OLED_CS 5
#define OLED_RST 17
U8G2_SH1106_128X64_NONAME_1_4W_SW_SPI disp(U8G2_R0, OLED_CLK, OLED_MOSI, OLED_CS, OLED_DC, OLED_RST);

// BME680 sensor
Bsec iaqSensor;
BME680Result lastBmeResult;
int bmeSensorStatus = 47788;

// sensor oled display vars
const char defaultSensorReadingLine[]   = "---.--";
const char sensorReadingTemplate[]  = "%3.2f";

// wifi vars
unsigned long wifiConnectAttemptTime = 0;
int wifiStatus = 57879;

// mqtt vars
WiFiClient espClient;
PubSubClient mqttClient(espClient);
const char mqttId[] = "esp32_aqi_sensor";
const unsigned int mqttSendInterval = 5000;
unsigned long lastMqttSendTime = 0;
int mqttConnectionStatus = 32;

void setup() {
  Serial.begin(115200);
  Serial.println("[setup] begin");

  // initialize I2C on default pins
  Wire.begin();

  // initialise BME680 sensor
  iaqSensor.begin(BME680_I2C_ADDR_PRIMARY, Wire);
  auto iaqSensorOK = checkBME680Status();
  if (iaqSensorOK) {
    bsec_virtual_sensor_t sensorList[10] = {
      BSEC_OUTPUT_RAW_TEMPERATURE,
      BSEC_OUTPUT_RAW_PRESSURE,
      BSEC_OUTPUT_RAW_HUMIDITY,
      BSEC_OUTPUT_RAW_GAS,
      BSEC_OUTPUT_IAQ,
      BSEC_OUTPUT_STATIC_IAQ,
      BSEC_OUTPUT_CO2_EQUIVALENT,
      BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    };

    iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
    if (!checkBME680Status()) {
      Serial.println("[setup] BME680 configuration error");
    }
  } else {
    Serial.println("[setup] BME680 initialization error");
  }

  // initialize OLED display
  disp.begin();

  // initialize WiFi
  WiFi.mode(WIFI_STA);

  // initialize MQTT connection
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);

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
  drawStatistics(bmeResult, lastBmeResult);
  if (bmeResult.hasValues) {
    lastBmeResult = bmeResult;
  }
  
  // send results via MQTT if the connection is established
  if (mqttConnected) {
    sendMQTT(bmeResult);
  }

  delay(1000);
}

void drawStatistics(BME680Result currentBmeResult, BME680Result previousBmeResult) {
  BME680Result bmeResult = currentBmeResult;
  if (!currentBmeResult.hasValues) {
    bmeResult = previousBmeResult;
  }

  disp.firstPage();
  do {
    
    // draw the grid that will hold our sensor readings
    disp.drawVLine(40, 13, 116);
    disp.drawVLine(85, 13, 116);
    disp.drawHLine(0, 13, 128);
    disp.drawHLine(0, 39, 128);

    // draw status glyps for Wifi and BME sensor status
    disp.setFont(u8g2_font_siji_t_6x10);
    disp.drawGlyph(116, 12, wifiStatus);
    disp.drawGlyph(104, 12, mqttConnectionStatus);
    disp.drawGlyph(92, 12, bmeSensorStatus);

    // draw the headings in each cell of the grid for the various readings
    disp.setFont(u8g2_font_squeezed_r6_tr);
    disp.drawStr(0, 23, "TMP (C)");
    disp.drawStr(0, 48, "HUM (%)");
    disp.drawStr(43, 23, "PRS (hPa)");
    disp.drawStr(43, 48, "IAQ");
    disp.drawStr(88, 23, "CO2 (PPM)");
    disp.drawStr(88, 48, "VOC (PPM)");

    if (bmeResult.hasValues) {
      char output[sizeof(defaultSensorReadingLine)];
      disp.setFont(u8g2_font_helvB08_tf);

      snprintf(output, sizeof(output), sensorReadingTemplate, bmeResult.temperature);
      disp.drawStr(0, 34, output); // temperature

      snprintf(output, sizeof(output), sensorReadingTemplate, bmeResult.humidity);
      disp.drawStr(0, 59, output); // humidity

      snprintf(output, sizeof(output), sensorReadingTemplate, bmeResult.pressure);
      disp.drawStr(43, 34, output); // pressure

      snprintf(output, sizeof(output), sensorReadingTemplate, bmeResult.iaq);
      disp.drawStr(43, 59, output); // IAQ

      snprintf(output, sizeof(output), sensorReadingTemplate, bmeResult.co2);
      disp.drawStr(88, 34, output); // CO2

      snprintf(output, sizeof(output), sensorReadingTemplate, bmeResult.voc);
      disp.drawStr(88, 59, output); // VOC
    } else {
      disp.setFont(u8g2_font_helvB08_tf);
      disp.drawStr(0, 34, defaultSensorReadingLine); // temperature
      disp.drawStr(0, 59, defaultSensorReadingLine); // humidity
      disp.drawStr(43, 34, defaultSensorReadingLine); // pressure
      disp.drawStr(43, 59, defaultSensorReadingLine); // IAQ
      disp.drawStr(88, 34, defaultSensorReadingLine); // CO2
      disp.drawStr(88, 59, defaultSensorReadingLine); // VOC
    }
  } 
  while ( disp.nextPage() );
}

void sendMQTT(BME680Result bmeResult) {
  // if we are not connected to MQTT broker then return
  if (!mqttClient.connected()) return;

  mqttClient.loop();

  // if no results are available from the sensors then return
  if (!bmeResult.hasValues) return;

  if (lastMqttSendTime == 0 || (lastMqttSendTime + mqttSendInterval) < millis()) {
    // Serial.println("[runMQTT] Send MQTT reading.");

    if (bmeResult.hasValues) {
      char temperatureString[7];
      char humdityString[7];
      char pressureString[7];
      char co2String[7];
      char vocString[7];
      char iaqString[6];
      char iaqAccuracyString[2];

      snprintf(temperatureString, sizeof(temperatureString), "%3.2f", bmeResult.temperature);     
      snprintf(humdityString, sizeof(humdityString), "%3.2f", bmeResult.humidity);
      snprintf(pressureString, sizeof(pressureString), "%3.2f", bmeResult.pressure);
      snprintf(co2String, sizeof(co2String), "%3.2f", bmeResult.co2);
      snprintf(vocString, sizeof(vocString), "%3.2f", bmeResult.voc);
      snprintf(iaqString, sizeof(iaqString), "%2.2f", bmeResult.iaq);
      snprintf(iaqAccuracyString, sizeof(iaqAccuracyString), "%d", bmeResult.iaqAccuracy);
      
      mqttClient.publish("temperature", temperatureString);
      mqttClient.publish("humidity", humdityString);
      mqttClient.publish("pressure", pressureString);
      mqttClient.publish("co2", co2String);
      mqttClient.publish("voc", vocString);
      mqttClient.publish("iaq", iaqString);
      mqttClient.publish("iaq_accuracy", iaqAccuracyString);
    }

    lastMqttSendTime = millis();
  }
}

bool setupMQTT() {
  if ( !mqttClient.connected() ) {
    mqttConnectionStatus = 32;
    mqttClient.connect(mqttId, MQTT_USER, MQTT_PASSWORD);
    return false;
  }

  mqttConnectionStatus = 57965;
  return true;
}

bool setupWiFi() { 
  uint8_t status = WiFi.status();
  
  if (status == WL_CONNECTED) {
    wifiStatus = 57882;
    return true;
  } else if (status == WL_DISCONNECTED && (wifiConnectAttemptTime == 0 || millis() > wifiConnectAttemptTime + 30000)) {
    wifiStatus = 57631;
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    wifiConnectAttemptTime = millis();
  } else if (status == WL_CONNECT_FAILED) {
    wifiStatus = 57463;
  }

  return false;
}

bool checkBME680Status() {
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      Serial.println("BSEC error code : " + String(iaqSensor.status));
    } else {
      Serial.println("BSEC warning code : " + String(iaqSensor.status));
    }
    
    return false;
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      Serial.println("BME680 error code : " + String(iaqSensor.bme680Status));
    } else {
      Serial.println("BME680 warning code : " + String(iaqSensor.bme680Status));
    }

    return false;
  }

  return true;
}

BME680Result readBME680() {
  BME680Result result;

  if (iaqSensor.run()) { // If new data is available  
    result.hasValues = true;  
    result.temperature = iaqSensor.temperature;
    result.humidity = iaqSensor.humidity;
    result.pressure = iaqSensor.pressure / 100.0;
    result.gasResistance = iaqSensor.gasResistance;
    result.iaq = iaqSensor.iaq;
    result.iaqAccuracy = iaqSensor.iaqAccuracy;
    result.co2 = iaqSensor.co2Equivalent;
    result.voc = iaqSensor.breathVocEquivalent;

    if (iaqSensor.iaqAccuracy == 0) {
      bmeSensorStatus = 57788;
    } else if (iaqSensor.iaqAccuracy == 1) {
      bmeSensorStatus = 57795;
    } else {
      bmeSensorStatus = 57794;
    }
  }

  return result;
}