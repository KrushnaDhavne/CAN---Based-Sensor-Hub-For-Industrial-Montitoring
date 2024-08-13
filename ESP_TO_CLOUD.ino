//FINAL WORKING CODE OF THIS PROJECRT

//#include <CAN.h>
//#include <WiFi.h>
//#include <PubSubClient.h>  
//#include <esp_wifi.h>
//
//#define TX_GPIO_NUM 5
//#define RX_GPIO_NUM 4
//
//const char* ssid =   "C2-10";                          //ssid - service set Identifier (Replace it with your ssid name)
//const char* password =  "Bokya@123";                     // replace with ssid paasword
//const char* mqttBroker = "mqtt3.thingspeak.com";                  // broker address - replace it with your broker address/cloud broker - test.mosquitto.org
//const int   mqttPort = 1883;                            // broker port number
//const char* clientID = "FA0MLSgZOSwSBDMOKhgbLjU";                   // client-id --> replace it in case willing to connect with same broker
//const char* mqtt_topic_1 = "channels/2591627/publish/fields/field1"; // topic names
//const char* mqtt_topic_2 = "channels/2591627/publish/fields/field2";   
//const char* mqtt_topic_3 =  "channels/2591627/publish/fields/field3";
//
//
//WiFiClient MQTTclient;
//PubSubClient client(MQTTclient);
//long lastReconnectAttempt = 0;
//boolean reconnect()
//{
//  //boolean connect (clientID, [username, password], [willTopic, willQoS, willRetain, willMessage], [cleanSession])
//  if (client.connect(clientID,"FA0MLSgZOSwSBDMOKhgbLjU","BWagG7HwvuGRKNzzWVaPnWzY")) {
//
//    Serial.println("Attempting to connect broker");
//    
//  }
//  return client.connected();
//}
//
//
//void setup() {
//  // Initialize Serial communication
//  Serial.begin(115200);
//  while (!Serial);
//  delay(1000);
//
//  // Initialize WiFi
//  Serial.println("Attempting to connect...");
//  WiFi.mode(WIFI_STA);
//  // esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]); // for wifi spoofing (Uncomment if needed)
//  WiFi.begin(ssid, password); // Connect to WiFi.
//  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
//    Serial.println("Couldn't connect to WiFi.");
//    while(1); // Stop here if WiFi connection fails
//  }
//  Serial.print("ESP32 IP ADDRESS : ");
//  Serial.println(WiFi.localIP());
//
//  // Add details for MQTT Broker (if needed)
//  client.setServer(mqttBroker, mqttPort); // Uncomment and add MQTT code here if necessary
//  lastReconnectAttempt = 0;
//
//  // Initialize CAN bus
//  CAN.setPins(RX_GPIO_NUM, TX_GPIO_NUM);
//  Serial.println("CAN Receiver");
//
//  // Start the CAN bus at 500 kbps
//  if (!CAN.begin(500E3)) {
//    Serial.println("Starting CAN failed!");
//    while (1); // Stop here if CAN initialization fails
//  }
//}
//
//
//
//
//
//void loop() {
//  if (!client.connected()) {
//    long now = millis();  // Returns the number of milliseconds passed since the Arduino board began running the current program
//    if (now - lastReconnectAttempt > 5000) { // Try to reconnect
//      lastReconnectAttempt = now;
//      if (reconnect()) { 
//        lastReconnectAttempt = 0;
//      }
//    }
//  }
//  else 
//  {
//    Serial.println("Connected to Broker --- !!"); 
//    //client.loop();
//
//    int packetSize = CAN.parsePacket();
//
//    if (packetSize) {
//      uint8_t temperature;
//      uint8_t motion;
//      uint8_t heat;
//
//      if (CAN.packetId() == 0x65D && packetSize == 3) {
//        temperature = CAN.read();  // Read temperature as integer
//        motion = CAN.read();       // Read motion
//        heat = CAN.read();         // Read heat
//
//        // Print the received data to Serial Monitor
//        Serial.print("Temperature: ");
//        Serial.println(temperature);
//
//        Serial.print("Motion: ");
//        Serial.println(motion);
//
//        Serial.print("Heat: ");
//        Serial.println(heat);
//
//        // Publish the received data to the MQTT broker
//        client.publish(mqtt_topic_1, String(temperature).c_str());
//        delay(200);
//        client.publish(mqtt_topic_2, String(motion).c_str());
//        delay(200);
//        client.publish(mqtt_topic_3, String(heat).c_str());
//        delay(200);
//        Serial.println("Message Published");
//      }
//    }
//  }
//}



//#include <CAN.h>
//#include <WiFi.h>
//#include <PubSubClient.h>
//#include <esp_wifi.h>
//
//#define TX_GPIO_NUM 5
//#define RX_GPIO_NUM 4
//
//const char* ssid = "C2-10";                          // SSID
//const char* password = "Bokya@123";                  // Password
//const char* mqttBroker = "mqtt3.thingspeak.com";     // Broker address
//const int mqttPort = 1883;                           // Broker port number
//const char* clientID = "FA0MLSgZOSwSBDMOKhgbLjU";    // Client ID
//const char* mqtt_topic_1 = "channels/2591627/publish/fields/field1"; // Topic names
//const char* mqtt_topic_2 = "channels/2591627/publish/fields/field2";
//const char* mqtt_topic_3 = "channels/2591627/publish/fields/field3";
//
//WiFiClient MQTTclient;
//PubSubClient client(MQTTclient);
//long lastReconnectAttempt = 0;
//
//boolean reconnect() {
//  if (client.connect(clientID, "FA0MLSgZOSwSBDMOKhgbLjU", "BWagG7HwvuGRKNzzWVaPnWzY")) {
//    Serial.println("Connected to MQTT Broker");
//  }
//  return client.connected();
//}
//
//void setup() {
//  // Initialize Serial communication
//  Serial.begin(115200);
//  while (!Serial);
//  delay(1000);
//
//  // Initialize WiFi
//  Serial.println("Attempting to connect to WiFi...");
//  WiFi.mode(WIFI_STA);
//  WiFi.begin(ssid, password);
//  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
//    Serial.println("Couldn't connect to WiFi.");
//    while (1); // Stop here if WiFi connection fails
//  }
//  Serial.print("ESP32 IP Address: ");
//  Serial.println(WiFi.localIP());
//
//  // MQTT Setup
//  client.setServer(mqttBroker, mqttPort);
//  lastReconnectAttempt = 0;
//
//  // Initialize CAN bus
//  CAN.setPins(RX_GPIO_NUM, TX_GPIO_NUM);
//  Serial.println("CAN Receiver");
//
//  // Start the CAN bus at 500 kbps
//  if (!CAN.begin(500E3)) {
//    Serial.println("Starting CAN failed!");
//    while (1); // Stop here if CAN initialization fails
//  }
//}
//
//void loop() {
//  if (!client.connected()) {
//    long now = millis();
//    if (now - lastReconnectAttempt > 5000) { // Try to reconnect every 5 seconds
//      lastReconnectAttempt = now;
//      if (reconnect()) {
//        lastReconnectAttempt = 0;
//      }
//    }
//  } else {
//    client.loop();
//
//    int packetSize = CAN.parsePacket();
//    if (packetSize) {
//      uint8_t temperature;
//      uint8_t motion;
//      uint8_t heat;
//
//      if (CAN.packetId() == 0x65D && packetSize == 3) {
//        temperature = CAN.read();  // Read temperature as integer
//        motion = CAN.read();       // Read motion
//        heat = CAN.read();         // Read heat
//
//        // Print the received data to Serial Monitor
//        Serial.print("Temperature: ");
//        Serial.println(temperature);
//
//        Serial.print("Motion: ");
//        Serial.println(motion);
//
//        Serial.print("Heat: ");
//        Serial.println(heat);
//
//        // Publish the received data to the MQTT broker
//        client.publish(mqtt_topic_1, String(temperature).c_str());
//        delay(200);
//        client.publish(mqtt_topic_2, String(motion).c_str());
//        delay(200);
//        client.publish(mqtt_topic_3, String(heat).c_str());
//        delay(200);
//        Serial.println("Message Published");
//      }
//    }
//  }
//}
//



//#include <CAN.h>
//#include <WiFi.h>
//#include <PubSubClient.h>
//#include <esp_wifi.h>
//
//#define TX_GPIO_NUM 5
//#define RX_GPIO_NUM 4
//
//const char* ssid = "C2-10";                          // SSID
//const char* password = "Bokya@123";                  // Password
//const char* mqttBroker = "mqtt3.thingspeak.com";     // Broker address
//const int mqttPort = 1883;                           // Broker port number
//const char* clientID = "FA0MLSgZOSwSBDMOKhgbLjU";    // Client ID
//const char* mqtt_topic_1 = "channels/2591627/publish/fields/field1"; // Topic names
//const char* mqtt_topic_2 = "channels/2591627/publish/fields/field2";
//const char* mqtt_topic_3 = "channels/2591627/publish/fields/field3";
//
//WiFiClient MQTTclient;
//PubSubClient client(MQTTclient);
//long lastReconnectAttempt = 0;
//
//boolean reconnect() {
//  if (client.connect(clientID, "FA0MLSgZOSwSBDMOKhgbLjU", "BWagG7HwvuGRKNzzWVaPnWzY")) {
//    Serial.println("Connected to MQTT Broker");
//  } else {
//    Serial.print("Failed to connect to MQTT Broker, rc=");
//    Serial.println(client.state());
//  }
//  return client.connected();
//}
//
//void setup() {
//  // Initialize Serial communication
//  Serial.begin(115200);
//  while (!Serial);
//  delay(1000);
//
//  // Initialize WiFi
//  Serial.println("Attempting to connect to WiFi...");
//  WiFi.mode(WIFI_STA);
//  WiFi.begin(ssid, password);
//  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
//    Serial.println("Couldn't connect to WiFi.");
//    while (1); // Stop here if WiFi connection fails
//  }
//  Serial.print("ESP32 IP Address: ");
//  Serial.println(WiFi.localIP());
//
//  // MQTT Setup
//  client.setServer(mqttBroker, mqttPort);
//  lastReconnectAttempt = 0;
//
//  // Initialize CAN bus
//  CAN.setPins(RX_GPIO_NUM, TX_GPIO_NUM);
//  Serial.println("CAN Receiver");
//
//  // Start the CAN bus at 500 kbps
//  if (!CAN.begin(500E3)) {
//    Serial.println("Starting CAN failed!");
//    while (1); // Stop here if CAN initialization fails
//  }
//  Serial.println("CAN Initialized successfully");
//}
//
//void loop() {
//  if (!client.connected()) {
//    long now = millis();
//    if (now - lastReconnectAttempt > 5000) { // Try to reconnect every 5 seconds
//      lastReconnectAttempt = now;
//      if (reconnect()) {
//        lastReconnectAttempt = 0;
//      }
//    }
//  } else {
//    client.loop();
//
//    int packetSize = CAN.parsePacket();
//    if (packetSize) {
//      Serial.print("CAN Packet Received: ");
//      Serial.print("ID: ");
//      Serial.print(CAN.packetId(), HEX);
//      Serial.print(", Size: ");
//      Serial.println(packetSize);
//
//      if (CAN.packetId() == 0x65D && packetSize == 5) {
//        uint8_t temperature = CAN.read();  // Read temperature
//        uint8_t motion = CAN.read();       // Read motion
//        uint8_t heat = CAN.read();         // Read heat
//
//        // Print the received data to Serial Monitor
//        Serial.print("Temperature: ");
//        Serial.println(temperature);
//
//        Serial.print("Motion: ");
//        Serial.println(motion);
//
//        Serial.print("Heat: ");
//        Serial.println(heat);
//
//        // Publish the received data to the MQTT broker
//        if (client.publish(mqtt_topic_1, String(temperature).c_str())) {
//          Serial.println("Temperature message published");
//        } else {
//          Serial.println("Failed to publish temperature message");
//        }
//        delay(200);
//
//        if (client.publish(mqtt_topic_2, String(motion).c_str())) {
//          Serial.println("Motion message published");
//        } else {
//          Serial.println("Failed to publish motion message");
//        }
//        delay(200);
//
//        if (client.publish(mqtt_topic_3, String(heat).c_str())) {
//          Serial.println("Heat message published");
//        } else {
//          Serial.println("Failed to publish heat message");
//        }
//        delay(200);
//      } else {
//        Serial.println("Unexpected CAN packet received");
//      }
//    }
//  }
//}


#include <CAN.h>
#include <WiFi.h>
#include <PubSubClient.h>

#define TX_GPIO_NUM 5
#define RX_GPIO_NUM 4

const char* ssid = "Saurabh";                          // SSID
const char* password = "12345678";                  // Password
const char* mqttBroker = "mqtt3.thingspeak.com";     // Broker address
const int mqttPort = 1883;                           // Broker port number
const char* clientID = "FA0MLSgZOSwSBDMOKhgbLjU";    // Client ID
const char* mqtt_topic_1 = "channels/2591627/publish/fields/field1"; // Topic names
const char* mqtt_topic_2 = "channels/2591627/publish/fields/field2";
const char* mqtt_topic_3 = "channels/2591627/publish/fields/field3";
const char* mqtt_topic_4 = "channels/2591627/publish/fields/field4";
//const char* mqtt_topic_5 = "channels/2591627/publish/fields/field5";



WiFiClient MQTTclient;
PubSubClient client(MQTTclient);
long lastReconnectAttempt = 0;

boolean reconnect() {
  if (client.connect(clientID, "FA0MLSgZOSwSBDMOKhgbLjU", "BWagG7HwvuGRKNzzWVaPnWzY")) {
    Serial.println("Connected to MQTT Broker");
  } else {
    Serial.print("Failed to connect to MQTT Broker, rc=");
    Serial.println(client.state());
  }
  return client.connected();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);

  Serial.println("Attempting to connect to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Couldn't connect to WiFi.");
    while (1);
  }
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqttBroker, mqttPort);
  lastReconnectAttempt = 0;

  CAN.setPins(RX_GPIO_NUM, TX_GPIO_NUM);
  Serial.println("CAN Receiver");

  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
  Serial.println("CAN Initialized successfully");
}

void loop() {
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    client.loop();

    int packetSize = CAN.parsePacket();
    if (packetSize) {
      Serial.print("CAN Packet Received: ");
      Serial.print("ID: ");
      Serial.print(CAN.packetId(), HEX);
      Serial.print(", Size: ");
      Serial.println(packetSize);

      if (CAN.packetId() == 0x65D && packetSize == 5) {
        uint8_t temperature = CAN.read();  // Read temperature
        uint8_t motion = CAN.read();       // Read motion
        uint8_t heat = CAN.read();         // Read heat
        uint8_t gas = CAN.read();         // Read gas
       // uint8_t voltage = CAN.read();         // Read voltage

        // Skip reading further bytes if you don't need them

        // Print the received data to Serial Monitor
        Serial.print("Temperature: ");
        Serial.println(temperature);

        Serial.print("Motion: ");
        Serial.println(motion);

        Serial.print("Heat: ");
        Serial.println(heat);

        Serial.print("Gas: ");
        Serial.println(gas);

//        Serial.print("Voltage: ");
//        Serial.println(voltage);

        // Publish the received data to the MQTT broker
        if (client.publish(mqtt_topic_1, String(temperature).c_str())) {
          Serial.println("Temperature message published");
        } else {
          Serial.println("Failed to publish temperature message");
        }
        delay(200);

        if (client.publish(mqtt_topic_2, String(motion).c_str())) {
          Serial.println("Motion message published");
        } else {
          Serial.println("Failed to publish motion message");
        }
        delay(200);

        if (client.publish(mqtt_topic_3, String(heat).c_str())) {
          Serial.println("Heat message published");
        } else {
          Serial.println("Failed to publish heat message");
        }
        delay(200);

        if (client.publish(mqtt_topic_4, String(gas).c_str())) {
          Serial.println("Gas message published");
        } else {
          Serial.println("Failed to publish gas message");
        }
        delay(200);
//        if (client.publish(mqtt_topic_5, String(voltage).c_str())) {
//          Serial.println("voltage message published");
//        } else {
//          Serial.println("Failed to publish voltage message");
//        }
//        delay(200);
      } else {
        Serial.print("Unexpected CAN packet received: ");
        Serial.print("ID: ");
        Serial.print(CAN.packetId(), HEX);
        Serial.print(", Size: ");
        Serial.println(packetSize);
      }
    }
  }
}