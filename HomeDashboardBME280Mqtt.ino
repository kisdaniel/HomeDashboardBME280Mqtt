#include <FS.h> //this needs to be first, or it all crashes and burns...

// the following override is not working due to this issue: https://github.com/knolleary/pubsubclient/pull/282
// #define MQTT_MAX_PACKET_SIZE    254
// you need to change PubSubClient.h MQTT_MAX_PACKET_SIZE to 254

#include "HomeDashboardBME280Mqtt.h"

#include <ESP8266WiFi.h> //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager

#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson

#include <PubSubClient.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//define your default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40];
char mqtt_port[6] = "1883";
char mqtt_user[100];
char mqtt_password[100];
char device_name[34] = "BME280Sensor";
// char open_close_timeout[6] = "45000";

char inTopic[60];
char outTopic[60];

//flag for saving data
bool mqttFailedBefore = false;

long lastConnectRetry = 0;

Adafruit_BME280 bme; // I2C

DynamicJsonBuffer jsonBuffer(512);

WiFiManager wifiManager;
WiFiClient espClient;
PubSubClient client(espClient);

WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqtt_user, 100);
WiFiManagerParameter custom_mqtt_password("password", "mqtt password", mqtt_password, 100);
WiFiManagerParameter custom_device_name("deviceName", "Device name", device_name, 32);
// WiFiManagerParameter custom_open_close_timeout("openCloseTimeout", "Opening/closing in millisec", open_close_timeout, 6);

float humidity, temperature, pressure;

char temperatureString[6];
char humidityString[6];
char pressureString[7];

void checkButtonCommands()
{
  bool wifiReset = digitalRead(WIFI_RESET_BUTTON) == PRESSED;
  if (wifiReset) {
    publishState();
  }
}

//callback notifying us of the need to save config
void saveConfigCallback()
{
  Serial.println("saving config");

  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_password, custom_mqtt_password.getValue());
  strcpy(device_name, custom_device_name.getValue());
  
  JsonObject &json = jsonBuffer.createObject();
  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;
  json["mqtt_user"] = mqtt_user;
  json["mqtt_password"] = mqtt_password;

  json["device_name"] = device_name;
  

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile)
  {
    Serial.println("failed to open config file for writing");
  }
  json.printTo(Serial);
  json.printTo(configFile);
  configFile.close();
  jsonBuffer.clear();
  //end save
}

void loadConfig()
{
  if (SPIFFS.begin())
  {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json"))
    {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile)
      {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        JsonObject &json = jsonBuffer.parseObject(buf.get());

        json.printTo(Serial);

        if (json.success())
        {
          Serial.println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(device_name, json["device_name"]);
        }
        else
        {
          Serial.println("failed to load json config");
        }
        configFile.close();
        jsonBuffer.clear();
      }
    }
    else
    {
      Serial.println("No config file present");
    }
  }
  else
  {
    Serial.println("failed to mount FS");
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (strcmp(topic, MQTT_TOPIC_REGISTRATION) == 0)
  {
    if (length == 2 && strncmp((char *)payload, "{}", length) == 0)
    {
      // resends a registration message
      deviceRegistration();
    }
    else
    {
      Serial.println("skip other device registration");
    }
  }
  else
  {
    JsonObject &inputObject = jsonBuffer.parseObject(payload);
    const char *command = inputObject["command"];
    if (strcmp(command, "read") == 0)
    {
      jsonBuffer.clear();
      publishState();
    }
    else if (strcmp(command, "heartbeat") == 0)
    {
      jsonBuffer.clear();
      publishState();
    }
    jsonBuffer.clear();
  }
}

void deviceRegistration()
{
  JsonObject &json = jsonBuffer.createObject();
  json["name"] = device_name;
  json["type"] = "BME280";

  char jsonChar[160];
  json.printTo((char *)jsonChar, json.measureLength() + 1);
  Serial.print("Register device (");
  Serial.print(MQTT_TOPIC_REGISTRATION);
  Serial.print("), result: ");
  Serial.println(client.publish(MQTT_TOPIC_REGISTRATION, jsonChar));
}

void connectToMqttIfNotConnected()
{
  if (!client.connected())
  {
    long now = millis();
    if (lastConnectRetry + 30000 < now)
    {
      client.setCallback(mqttCallback);
      client.setServer(mqtt_server, atoi(mqtt_port));

      analogWrite(STATUS_LED, NETWORK_STATUS_CONNECTING_TO_MQTT);

      boolean connectResult = false;

      if (strlen(mqtt_user) > 0)
      {
        connectResult = client.connect(device_name, mqtt_user, mqtt_password);
      }
      else
      {
        connectResult = client.connect(device_name);
      }

      if (connectResult)
      {
        lastConnectRetry = millis();
        Serial.println("Connected to MQTT");
        mqttFailedBefore = false;

        deviceRegistration();
        client.subscribe(MQTT_TOPIC_REGISTRATION);
        strcpy(inTopic, "/homedashboard/");
        strcat(inTopic, device_name);
        strcpy(outTopic, inTopic);
        strcat(inTopic, "/in");
        strcat(outTopic, "/out");

        client.subscribe(inTopic);
        Serial.print("Subscribe to ");
        Serial.println(inTopic);
        publishState();
        analogWrite(STATUS_LED, NETWORK_STATUS_CONNECTED);
      }
      else
      {
        analogWrite(STATUS_LED, NETWORK_STATUS_ONLY_WIFI);
        lastConnectRetry = millis();
        if (!mqttFailedBefore)
        {
          Serial.print("Failed to connect to MQTT server: ");
          Serial.print(mqtt_server);
          Serial.print(", port: ");
          Serial.println(atoi(mqtt_port));
        }
        mqttFailedBefore = true;
      }
    }
  }
  else
  {
    client.loop();
  }
}

void publishState()
{
  if (client.connected())
  {
    humidity = bme.readHumidity();
    temperature = bme.readTemperature() * 1.8 + 32.0;
    // dp = t - 0.36 * (100.0 - h); -- harmatpont
    pressure = bme.readPressure() / 100.0F;

    dtostrf(temperature, 5, 1, temperatureString);
    dtostrf(humidity, 5, 1, humidityString);
    dtostrf(pressure, 6, 1, pressureString);

    JsonObject &json = jsonBuffer.createObject();
    json["name"] = device_name;

    json["temperature"] = pressure;
    json["pressure"] = pressure;
    json["humidity"] = humidity;

    char jsonChar[200];
    json.printTo((char *)jsonChar, json.measureLength() + 1);
    int result = client.publish(outTopic, jsonChar, json.measureLength());

    // Serial.print("publish state to topic:");
    // Serial.print(outTopic);
    // Serial.print(", length: ");
    // Serial.print(json.measureLength());

    // Serial.print(", result: ");
    // Serial.println(result);

    // Serial.println(jsonChar);
    // Serial.println(openState);

    jsonBuffer.clear();

    // lastStateSent = millis();
  }
}

void initPins()
{
  Serial.println("init pins");
  
  pinMode(STATUS_LED, OUTPUT);
  pinMode(WIFI_RESET_BUTTON, INPUT_PINMODE);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  
  Serial.println(F("try BME280"));
  while (!bme.begin(BME280_DEVICE_ID))
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    delay(200);
    digitalWrite(STATUS_LED, LOW);
    delay(400);
    digitalWrite(STATUS_LED, HIGH);
  }

  analogWrite(STATUS_LED, NETWORK_STATUS_NOT_CONNECTED);

  Serial.println("waiting 2 sec...");
  delay(2000);
  bool wifiReset = digitalRead(WIFI_RESET_BUTTON) == PRESSED;
  if (wifiReset)
  {

    Serial.println("reset pressed...");
    flashLedIn();
    flashLedOut();

    wifiReset = digitalRead(WIFI_RESET_BUTTON) == PRESSED;
    if (wifiReset)
    {
      flashLedIn();
      flashLedOut();

      Serial.println("reset requested...");
      // SPIFFS.format();
      wifiManager.resetSettings();
    }
  }
  analogWrite(STATUS_LED, NETWORK_STATUS_NOT_CONNECTED);

  //attachInterrupt(digitalPinToInterrupt(PHOTOCELL_INPUT_PIN), photocellpinChange, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(OPEN_SWITCH_PIN), openSwitch_PIN, CHANGE);
}

void flashLedOut()
{
  for (int i = 1023; i > 0; i--)
  {
    analogWrite(STATUS_LED, i);
    delay(1);
  }
}
void flashLedIn()
{
  for (int i = 0; i < 1024; i++)
  {
    analogWrite(STATUS_LED, i);
    delay(1);
  }
}

void setup()
{
  Serial.begin(115200);

  Serial.println(MQTT_MAX_PACKET_SIZE);
  Serial.println("starting...");

  //clean FS, for testing
  //SPIFFS.format();
  //wifiManager.resetSettings();

  //read configuration from FS json
  Serial.println("mounting FS...");

  loadConfig();

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  // set static ip
  // wifiManager.setSTAStaticIPConfig(IPAddress(10, 0, 1, 99), IPAddress(10, 0, 1, 1), IPAddress(255, 255, 255, 0));

  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_password);
  wifiManager.addParameter(&custom_device_name);
  // wifiManager.addParameter(&custom_open_close_timeout);

  WiFi.hostname(device_name);
  wifiManager.setConfigPortalTimeout(60 * 3);

  initPins();

  if (!wifiManager.autoConnect("HomeDashboardBME280AP", "password"))
  {
    Serial.println("failed to connect and hit timeout, you should reset to reconfigure");
  }
  else
  {
    Serial.println("connected...");
    Serial.println("local ip");
    Serial.println(WiFi.localIP());
  }

  //read updated parameters

  //save the custom parameters to FS
}

void loop()
{
  if (!WiFi.isConnected())
  {
    analogWrite(STATUS_LED, NETWORK_STATUS_NOT_CONNECTED);
    Serial.println("Wifi connection lost...");
    if (WiFi.reconnect())
    {
      analogWrite(STATUS_LED, NETWORK_STATUS_ONLY_WIFI);
      Serial.print("successfully reconnected, local ip:");
      Serial.println(WiFi.localIP());
      connectToMqttIfNotConnected();
    }
  }
  else
  {
    connectToMqttIfNotConnected();
  }
  checkButtonCommands();
}
