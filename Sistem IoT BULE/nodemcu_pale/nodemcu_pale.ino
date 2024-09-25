#include <ESP8266WiFi.h>
#include <PubSubClient.h>

WiFiClient espClient;
PubSubClient client(espClient);

// Konfigurasi WiFi
// const char *ssid = "WIFI_Premium";
// const char *password = "senyumdulu";
//const char *ssid           = "Dr Parwi";
//const char *password       = "ponorogo1";
const char *ssid           = "Kolam Lele";
const char *password       = "tanyaazzam";

// Konfigurasi MQTT
const char *mqtt_server = "IP Server MQTT";
const int mqtt_port = "portMqtt";
const char *mqtt_username = "usernameMqtt";
const char *mqtt_password = "passwordMqtt";

// Konfigurasi Pin pada LED
const int ledPin = 14;  // GPIO 14 on ESP-12E NodeMCU board

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);  // Set pin LED sebagai output
  client.setCallback(callback);

  // Sambungkan ke WiFi
  connectWiFi();

  // Sambungkan ke MQTT
  connectMQTT();
}

void loop() {
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();

  // Baca data dari Arduino Nano melalui Serial
  while (Serial.available()) {
    char dataType = Serial.read();

    // Map tipe sensor ke topik MQTT
    const char *sensorTypes[3] = { "T", "A", "P" };
    const char *mqttTopics[3] = { "sensor/NTC/tempInC", "sensor/MQ135/ammonia", "sensor/PH/phValue" };

    // Cari indeks tipe sensor
    int sensorIndex = -1;
    for (int i = 0; i < 3; ++i) {
      if (dataType == sensorTypes[i][0]) {
        sensorIndex = i;
        break;
      }
    }

    // Jika indeks valid, simpan nilai sensor dan kirim ke MQTT
    if (sensorIndex >= 0 && sensorIndex < 3) {
      // Baca nilai sensor dari Serial
      float sensorValue = Serial.parseFloat();

      // Debugging (opsional)
      Serial.print("Received ");
      Serial.print(sensorTypes[sensorIndex]);
      Serial.print(": ");
      Serial.println(sensorValue);

      // Kirim nilai sensor ke MQTT
      String topic = mqttTopics[sensorIndex];
      String payload = String(sensorValue);
      client.publish(topic.c_str(), payload.c_str());
    }
  }

  delay(1000);  // Delay untuk stabilitas Serial Communication
}

void connectWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void connectMQTT() {
  Serial.println("Connecting to MQTT...");
  client.setServer(mqtt_server, mqtt_port);
  while (!client.connected()) {
    if (client.connect("NodeMCU_Client", mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT");
      client.subscribe("katup");
    } else {
      delay(1000);
      Serial.println("Connecting to MQTT...");
    }
  }
}

// This functions is executed when some device publishes a message to a topic that your NodeMCU is subscribed to
void callback(char *topic, byte *message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageInfo;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageInfo += (char)message[i];
  }
  Serial.println();

  // If a message is received on the topic room/lamp, you check if the message is either on or off. Turns the lamp GPIO according to the message
  if (strcmp(topic, "katup") == 0) {
    Serial.print("Changing katup to ");
    if (messageInfo == "on") {
      digitalWrite(ledPin, HIGH);
      Serial.print("On");
    } else if (messageInfo == "off") {
      digitalWrite(ledPin, LOW);
      Serial.print("Off");
    }
  }
  Serial.println();
}
