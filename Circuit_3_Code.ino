// Board - NanoESP (Pretzel)

#include <NanoESP.h>
#include <NanoESP_MQTT.h>
#include <SoftwareSerial.h>
#include <Servo.h>

Servo myservo;

// Wi-Fi credentials
#define SSID "YOUR-WIFI-USERNAME"
#define PASSWORD "YOUR-WIFI-PASSWORD

#define LED_WLAN 13
#define DEBUG true

#define ON_OFF A0

int servoemer = 0;

NanoESP nanoesp = NanoESP();
NanoESP_MQTT mqtt = NanoESP_MQTT(nanoesp);

String servoPosition;
String message;

int servoPositionInt = 0; // Set initial servo position to zero

void setup()
{
  Serial.begin(19200);
  pinMode(ON_OFF, INPUT);
  pinMode(LED_WLAN, OUTPUT);

  nanoesp.init();
  nanoesp.configWifiStation(SSID, PASSWORD);

  connectToWifi();

  myservo.attach(5);               // Servo at pin D5
  myservo.write(servoPositionInt); // Set Initial Servo Position

  // Print IP in Terminal
  Serial.println(nanoesp.getIp());

  connectToMQTT();
}

void loop()
{
  int id, len;
  int on_off_value = analogRead(ON_OFF); // Corrected typo in analogRead()
  // Serial.println(on_off_value);

  if (on_off_value <= 10)
  {
    if (servoemer == 0)
    {
      delay(1);
    }
    else
    {
      Serial.println(on_off_value);
      servoemer = 0;
      myservo.write(0);
      servoPosition = "0";
      Serial.println("Servo position set to: " + servoPosition);
      mqtt.publish(0, "two/servo_pos", servoPosition.c_str());
      Serial.println("Set the sliders to 0: Disconnect Mode");
    }
  }
  else if (on_off_value >= 1000)
  {
    if (servoemer == 140)
    {
      delay(1);
    }
    else
    {
      Serial.println(on_off_value);
      servoemer = 140;
      myservo.write(140);
      servoPosition = "140";
      Serial.println("Servo position set to: " + servoPosition);
      mqtt.publish(0, "two/servo_pos", servoPosition.c_str());
      Serial.println("Set the sliders to 140: Disconnect Mode");
    }
  }
  else
  {
    if (nanoesp.recvData(id, len))
    {
      String topic, value;
      if (mqtt.recvMQTT(id, len, topic, value))
      {
        Serial.println("New Message:\nTopic=" + topic + " Value=" + value);
        if (topic == "two/servo")
        {
          servoPosition = value; // Store the received value
          servo_function();
        }
      }
    }
    mqtt.stayConnected(0); // Keep the connection to the broker alive
  }
}

void servo_function()
{
  Serial.println("Servo position set to: " + servoPosition);
  mqtt.publish(0, "two/servo_pos", servoPosition.c_str());
  servoPositionInt = servoPosition.toInt();
  myservo.write(servoPositionInt);
}

void subscribe_topics()
{
  if (mqtt.subscribe(0, "two/servo"))
  {
    Serial.println("Subscribed to two/servo");
  }
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
    if (mqtt.connect(0, "YOUR-MQTT-SERVER-IP", 1883, "NanoESP2"))
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