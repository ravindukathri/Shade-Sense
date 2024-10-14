// Board - NanoESP (Pretzel)

#include <NanoESP.h>
#include <NanoESP_MQTT.h>
#include <SoftwareSerial.h>

// Wi-Fi credentials
#define SSID "YOUR-WIFI-USERNAME"
#define PASSWORD "YOUR-WIFI-PASSWORD"

#define LED_WLAN 13 // WLAN LED to verify Calibrartion status
#define DEBUG true

// USE Calibration Values
int minLDR = 350;
int maxLDR = 750;

unsigned long lastLDRUpdate = 0;
const unsigned long LDRInterval = 1000;

NanoESP nanoesp1 = NanoESP();
NanoESP_MQTT mqtt = NanoESP_MQTT(nanoesp1);

String servoPosition;
String message;
int LED = 3;
int mappedValue;

void setup()
{
  Serial.begin(19200);
  nanoesp1.init();
  nanoesp1.configWifiStation(SSID, PASSWORD);

  connectToWifi();

  pinMode(A0, INPUT);
  pinMode(LED, OUTPUT);

  // Print IP in Terminal
  Serial.println(nanoesp1.getIp());

  connectToMQTT();
}

void loop()
{
  int id, len;

  if (nanoesp1.recvData(id, len))
  {
    String topic, value;
    if (mqtt.recvMQTT(id, len, topic, value))
    {
      Serial.println("New Message:\nTopic=" + topic + " Value=" + value);
      if (topic == "three/position")
      {
        int valuenew = value.toInt();
        mappedValue = map(valuenew, 0, 100, 0, 255);
        analogWrite(LED, mappedValue);
        mqtt.publish(0, "three/pwm", String(value).c_str());
      }
      else if (topic == "main/outdoorcalibration")
      {
        // built in led 2 blinks
        Serial.println("Command Outdoor calibration received");
        digitalWrite(LED_WLAN, HIGH);
        delay(100);
        digitalWrite(LED_WLAN, LOW);
        delay(100);
        digitalWrite(LED_WLAN, HIGH);
        delay(100);
        digitalWrite(LED_WLAN, LOW);
        delay(1000);
        maxLDR = analogRead(A0) + 10;
        Serial.println(maxLDR);
        Serial.println("Maximum LDR value measured");
        digitalWrite(LED_WLAN, HIGH);
        delay(100);
        digitalWrite(LED_WLAN, LOW);
        delay(100);
        digitalWrite(LED_WLAN, HIGH);
        delay(100);
        digitalWrite(LED_WLAN, LOW);
        delay(1000);
        // build in led 3 blinks
        digitalWrite(LED_WLAN, HIGH);
        delay(300);
        digitalWrite(LED_WLAN, LOW);
        delay(300);
        digitalWrite(LED_WLAN, HIGH);
        delay(300);
        digitalWrite(LED_WLAN, LOW);
        delay(300);
        digitalWrite(LED_WLAN, HIGH);
        delay(300);
        digitalWrite(LED_WLAN, LOW);
        delay(3000);
        minLDR = analogRead(A0);
        Serial.println(minLDR);
        Serial.println("Minimum LDR value measured");
        digitalWrite(LED_WLAN, HIGH);
        delay(300);
        digitalWrite(LED_WLAN, LOW);
        delay(300);
        digitalWrite(LED_WLAN, HIGH);
        delay(300);
        digitalWrite(LED_WLAN, LOW);
        delay(300);
        digitalWrite(LED_WLAN, HIGH);
        delay(300);
        digitalWrite(LED_WLAN, LOW);
        delay(1000);
        mqtt.publish(0, "main/outdoorcalibration1", "0");
        Serial.println("Command Outdoor calibration sent");
      }
    }
  }
  mqtt.stayConnected(0); // keep the connection to broker alive

  // Handle LDR readings at specified intervals
  unsigned long currentMillis = millis();
  if (currentMillis - lastLDRUpdate >= LDRInterval)
  {
    check_brightness(); // Read and publish the LDR value
    lastLDRUpdate = currentMillis;
  }
}

void check_brightness()
{
  int ldrValue = analogRead(A0); // LDR PIN
  Serial.println(maxLDR);
  Serial.println(minLDR);
  float lightintencity = map(ldrValue, minLDR, maxLDR, 0, 100);
  mqtt.publish(0, "two/ldr", String(lightintencity).c_str());
}

void connectToWifi()
{
  while (true)
  {
    if (nanoesp1.wifiConnected())
    {
      Serial.println("Wifi connected");
      break;
    }
    else
    {
      Serial.println("Wifi not Connected");
      Serial.println("Retrying to Connect to WiFi");
    }
  }
}

void connectToMQTT()
{
  while (true)
  {
    if (mqtt.connect(0, "YOUR-MQTT-SERVER-IP", 1883, "NanoESP1"))
    {
      Serial.println("MQTT connected");
      subscribe_topics();
      break;
    }
    else
    {
      Serial.println("MQTT not Connected");
      Serial.println("Retrying to Connect to MQTT");
    }
  }
}

void subscribe_topics()
{
  if (mqtt.subscribe(0, "three/position"))
    Serial.println("Subscribed to three/position");
  if (mqtt.subscribe(0, "main/outdoorcalibration"))
    Serial.println("Subscribed to main/outdoorcalibration");
}
