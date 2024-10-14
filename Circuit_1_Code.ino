// Board - NodeMCU

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

// Wi-Fi credentials
#define SSID "YOUR-WIFI-USERNAME"
#define PASSWORD "YOUR-WIFI-PASSWORD"

// MQTT Broker
const char *mqtt_server = "YOUR-MQTT-SERVER-IP";

// Pin definitions
#define LED_WLAN 13 // WLAN LED PIN 13
#define RECV_PIN D5 // GPIO for IR receiver
#define LDR_PIN A0  // Analog pin for LDR

// Define IR Remote Buttons
#define BUTTON_1 0xFF30CF
#define BUTTON_2 0xE718FF00
#define BUTTON_3 0xA15EFF00
#define BUTTON_4 0xF708FF00
#define BUTTON_5 0xE31CFF00
#define BUTTON_6 0xA55AFF00
#define BUTTON_7 0xBD42FF00
#define BUTTON_0 0xE916FF00
#define BUTTON_100 0xE619FF00
#define BUTTON_manual 0xF807FF00
#define BUTTON_auto 0xEA15FF00

// IR Receiver instance
IRrecv irrecv(RECV_PIN);
decode_results results;

// LDR calibration
int minLDR = 200;
int maxLDR = 650;
// int minLDR, maxLDR;

// MQTT client
WiFiClient espClient;
PubSubClient client(espClient);

// Variables
String servoPosition;
String mode;
String setpoint;
float lightintencity;
int setpointInt;
unsigned long lastLDRUpdate = 0;
const unsigned long LDRInterval = 1000;

// Function declarations
void setup_wifi();
void callback(char *topic, byte *payload, unsigned int length);
void reconnect();
void check_brightness();
void manualRemote();

void setup()
{
  Serial.begin(115200);

  pinMode(LED_WLAN, OUTPUT);
  pinMode(D1, OUTPUT); // GPIO5
  pinMode(D2, OUTPUT); // GPIO4

  // Connect to Wi-Fi
  setup_wifi();

  // Setup MQTT
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // Start IR Receiver
  irrecv.enableIRIn();
}

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  // Handle IR remote
  manualRemote();

  // Handle LDR readings at specified intervals
  unsigned long currentMillis = millis();
  if (currentMillis - lastLDRUpdate >= LDRInterval)
  {
    check_brightness();
    lastLDRUpdate = currentMillis;
  }
}

// Connect to Wi-Fi
void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(SSID);

  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// MQTT message handler
void callback(char *topic, byte *payload, unsigned int length)
{
  String message;
  for (unsigned int i = 0; i < length; i++)
  {
    message += (char)payload[i];
  }
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(message);

  // Handle incoming messages
  if (String(topic) == "two/servo")
  {
    servoPosition = message; // Store the received value
  }
  else if (String(topic) == "main/setmode")
  {
    mode = message;
    if (mode == "0")
    {
      digitalWrite(D1, HIGH);
      digitalWrite(D2, LOW);
    }
    else if (mode == "1")
    {
      digitalWrite(D1, LOW);
      digitalWrite(D2, HIGH);
    }
  }
  else if (String(topic) == "main/indoorcalibration")
  {
    client.publish("main/setservo", "0");
    delay(2000);
    minLDR = analogRead(LDR_PIN);
    Serial.println(minLDR);
    delay(2000);
    client.publish("main/setservo", "140");
    delay(2000);
    maxLDR = analogRead(LDR_PIN) + 5;
    Serial.println(maxLDR);
    delay(2000);
    client.publish("main/indoorcalibration1", "0");
  }
}

// Reconnect to MQTT broker
void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("NodeMCUClient"))
    {
      Serial.println("connected");
      client.subscribe("two/servo");
      client.subscribe("main/setmode");
      client.subscribe("main/indoorcalibration");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// LDR check and publish
void check_brightness()
{
  Serial.println("Printing values in Brightness function");
  Serial.println(minLDR);
  Serial.println(maxLDR);
  int ldrValue = analogRead(LDR_PIN);
  lightintencity = map(ldrValue, minLDR, maxLDR, 0, 100);
  client.publish("one/ldr", String(lightintencity).c_str());
}

// Handle IR remote inputs
void manualRemote()
{
  if (irrecv.decode(&results))
  {
    unsigned long buttonValue = results.value;
    Serial.println(buttonValue);
    switch (buttonValue)
    {
    case 16724175:
      Serial.println("1 Pressed");
      client.publish("main/number", "1");
      break;
    case 16718055:
      client.publish("main/number", "2");
      break;
    case 16743045:
      client.publish("main/number", "3");
      break;
    case 16716015:
      client.publish("main/number", "4");
      break;
    case 16726215:
      client.publish("main/number", "5");
      break;
    case 16734885:
      client.publish("main/number", "6");
      break;
    case 16728765:
      client.publish("main/number", "7");
      break;
    case 16730805:
      client.publish("main/number", "8");
      break;
    case 16732845:
      client.publish("main/number", "9");
      break;
    case 16754775:
      client.publish("main/mode", "1");
      break;
    case 16769055:
      client.publish("main/mode", "0");
      break;
    default:
      Serial.println("Unknown button pressed.");
      break;
    }
    irrecv.resume();
  }
}
