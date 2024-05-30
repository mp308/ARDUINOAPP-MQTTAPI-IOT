#include <PubSubClient.h>
#include <WiFi.h>
#include <DHT.h>

#define ON 1
#define OFF 0
#define DHTTYPE DHT22

const char* ssid = "Wokwi-GUEST";
const char* password =  "";
WiFiClient espClient;

// initial MQTT client
const char* mqttServer = "mqtt.netpie.io";
const int mqttPort = 1883;
const char* clientID = "dbcd1612-6619-49cf-acc1-ded83aa82946";
const char* mqttUser = "XeH8bm7UNCJHb2A4WZhQgZPNishUPKGc";
const char* mqttPassword = "iHitGnex8rmtEvD8awSqeRC9yzrU3NEU";
const char* topic_pub = "@shadow/data/update";
const char* topic_sub = "@msg/lab_ict_kps/command";
// send buffer
String publishMessage;

PubSubClient client(espClient);

const int DHT_PIN = 21;
DHT dht(DHT_PIN, DHTTYPE);
const int trig_PIN = 23;
const int echo_PIN = 22;
const int RED_LED_PIN = 19;

int redLED_status = OFF;

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnectMQTT() {
  char mqttinfo[80];
  snprintf(mqttinfo, 75, "Attempting MQTT connection at %s:%d (%s/%s)...", mqttServer, mqttPort, mqttUser, mqttPassword);
  while (!client.connected()) {
    Serial.println(mqttinfo);
    String clientId = clientID;
    if (client.connect(clientId.c_str(), mqttUser, mqttPassword)) {
      Serial.println("...Connected");
      client.subscribe(topic_sub);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void messageReceivedCallback(char* topic, byte* payload, unsigned int length) {
  char payloadMsg[80];

  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    payloadMsg[i] = (char)payload[i];
  }
  payloadMsg[length] = '\0';
  Serial.println();
  Serial.println("-----------------------");
  processMessage(payloadMsg);
}

void processMessage(String recvCommand) {

  if (recvCommand == "RED_ON") {
    digitalWrite(RED_LED_PIN, HIGH);
    redLED_status = ON;
  } else if (recvCommand == "RED_OFF") {
    digitalWrite(RED_LED_PIN, LOW);
    redLED_status = OFF;
  } 
}

void setup() {
   Serial.begin(115200);
  setup_wifi();
  client.setServer(mqttServer, mqttPort);
   client.setCallback(messageReceivedCallback);

  pinMode(trig_PIN, OUTPUT);
  pinMode(echo_PIN, INPUT);
  pinMode(RED_LED_PIN, OUTPUT);

   redLED_status = OFF;

}

void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

 dht.read();
  float t = dht.readTemperature();
  float h = dht.readHumidity();

  digitalWrite(trig_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_PIN, LOW);

  long duration = pulseIn(echo_PIN, HIGH);
  float distance_cm = duration * 0.034 / 2;



  String publishMessage = "{\"data\": {\"red_led\": " + String(redLED_status) + 
                          ",\"temperature\": " + String(t) +
                          ", \"humidity\": " + String(h) +
                          ", \"ultrasonic_distance\": " + String(distance_cm) +
                          "}}";

  Serial.println(publishMessage);
  client.publish(topic_pub, publishMessage.c_str());
  
  delay(2000);
}
