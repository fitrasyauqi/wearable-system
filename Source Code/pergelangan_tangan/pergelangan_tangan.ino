#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#define BOARD_ID 1

// REPLACE WITH YOUR RECEIVER MAC Address 48:E7:29:B6:77:A0
uint8_t broadcastAddress[] = {0x48, 0xE7, 0x29, 0xB6, 0x77, 0xA0};

Adafruit_MPU6050 mpu1;

// Structure definition for the data to be sent
// Must match the structure on the receiver side
typedef struct struct_message {
  int id;
  float ax1, ay1, az1, gx1, gy1, gz1;
} struct_message;

// Create a struct_message instance called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// Callback function for when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  // Initialize Serial Monitor and I2C communication
  Wire.begin();
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialize MPU6050
  if (!mpu1.begin()){
    Serial.println("Failed to find MPU6050 chips");
    while(1){
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!!");

  // Configure accelerometer and gyro ranges
  mpu1.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu1.setGyroRange(MPU6050_RANGE_500_DEG);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the callback function to get the transmission status
  esp_now_register_send_cb(OnDataSent);
  
  // Register the peer device
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add the peer to the ESP-NOW network       
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 
void loop() {
  // Get sensor readings from MPU6050
  sensors_event_t a1, g1, temp1;
  mpu1.getEvent(&a1, &g1, &temp1);

  // Assign sensor data to struct fields
  myData.id = BOARD_ID;
  myData.ax1 = a1.acceleration.x;
  myData.ay1 = a1.acceleration.y;
  myData.az1 = a1.acceleration.z;
  myData.gx1 = g1.gyro.x;
  myData.gy1 = g1.gyro.y;
  myData.gz1 = g1.gyro.z;

  // Send data via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(200);
}