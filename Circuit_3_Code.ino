#include <NanoESP.h>
#include <NanoESP_MQTT.h>
#include <SoftwareSerial.h>
#include <Servo.h>

Servo myservo;

// MQTT Basic functions
// Change SSID and PASSWORD.

#define SSID "IoT-AP"
#define PASSWORD "MySecurePassword"

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

  if (nanoesp.wifiConnected()) {
    Serial.println("WiFi connected");
    digitalWrite(LED_WLAN, HIGH);
  }
  else {
    Serial.println("WiFi not connected");
  }
  
  myservo.attach(5);  // Servo at pin D5
  myservo.write(servoPositionInt);  // set it to zero

  // Print IP in Terminal
  Serial.println(nanoesp.getIp());

  if (mqtt.connect(0, "192.168.199.85", 1883, "NanoESP")) 
  {
    subscribe_topics();
  }
}

void loop() 
{
  int id, len;
  int on_off_value = analogRead(ON_OFF);  // Corrected typo in analogRead()
  //Serial.println(on_off_value);

  if (on_off_value <= 10)
  {
   if(servoemer == 0)
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
    if(servoemer == 140)
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
          servoPosition = value;  // Store the received value
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