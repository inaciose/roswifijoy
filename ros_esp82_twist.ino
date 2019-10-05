#include <ESP8266WiFi.h>

// i2c
#include <Wire.h>

// arduino ota
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// ros
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

//#define DEBUG

// i2c
#define SDA_PIN 4
#define SCL_PIN 5
//const int16_t I2C_MASTER = 0x42;
const int16_t I2C_SLAVE = 0x08;

// wifi
const char* ssid     = "vfxira";
const char* password = "0101010101";
// Set the rosserial socket server IP address
IPAddress server(192,168,1,40);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

// ros
ros::NodeHandle nh;
geometry_msgs::Twist twist_msg;
ros::Publisher twist_pub("cmd_vel", &twist_msg);  

void setup() {

  Serial.begin(115200);

  // setup wifi
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // arduino ota
  WiFi.mode(WIFI_STA);

  // Connect the ESP8266 the the wifi AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // bof arduino ota
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  // eof arduino ota
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // setup ros publisher
  nh.advertise(twist_pub);

  nh.spinOnce();
  delay(5);

  // setup i2c
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(5);
  
}

void loop() {
  // for buffer conversion
  union u_tag {
    uint8_t b[4];
    uint16_t i[2];
    uint32_t l;
   } i2c_data[4];

  // for receive buffer
  uint8_t buf[12] = {0};
  int i = 0;

  // arduino ota
  ArduinoOTA.handle();

  // set buffer conversion elements to zero
  for(int f = 0; f < 4; f++) i2c_data[f].l = 0;

  // request 6 bytes from slave
  Wire.requestFrom(I2C_SLAVE, 8); 

  while (Wire.available()) {
    buf[i] = (uint8_t)Wire.read();
    i++;
  }

  // convert to 32bits
  i2c_data[0].b[0] = buf[0];
  i2c_data[0].b[1] = buf[1];
  i2c_data[1].b[0] = buf[2];
  i2c_data[1].b[1] = buf[3];
  i2c_data[2].b[0] = buf[4];
  i2c_data[2].b[1] = buf[5];
  i2c_data[3].b[0] = buf[6];
  i2c_data[3].b[1] = buf[7];
  
  #ifdef DEBUG
    Serial.print(i2c_data[1].i[0]); Serial.print("\t");
    Serial.print(i2c_data[2].i[0]); Serial.print("\t");
    Serial.print(i2c_data[3].i[0]); Serial.print("\t");
    Serial.print("\n");
  #endif
  
  delay(2);

  if (nh.connected()) {
    //Serial.println("Connected");
    // twist
    twist_msg.linear.x = (512.0 - i2c_data[1].i[0]) / 512;
    twist_msg.angular.z = (512.0 - i2c_data[2].i[0]) / 512;

    // lower limit
    if(twist_msg.linear.x > -0.1 && twist_msg.linear.x < 0.1) twist_msg.linear.x = 0;
    if(twist_msg.angular.z > -0.1 && twist_msg.angular.z < 0.1) twist_msg.angular.z = 0;
    
    twist_pub.publish(&twist_msg);
  } else {
  //  Serial.println("x");
  }
  nh.spinOnce();
  // Loop exproximativly at 10Hz
  delay(99);
}
