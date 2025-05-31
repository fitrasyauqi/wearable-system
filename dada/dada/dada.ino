#include <Arduino.h>
#include "model_RF.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>

Adafruit_MPU6050 mpu3;
Eloquent::ML::Port::RandomForest rm_classifier;

// Structure to receive data
// Must match the structure on the sender side
typedef struct struct_message {
  int id;
  float ax, ay, az, gx, gy, gz;
} struct_message;

// Create a struct_message instance called myData
struct_message myData;

// Create structures to hold the readings from each board
struct_message board1;
struct_message board2;

// Create an array to store data from multiple boards
struct_message boardsStruct[2] = {board1, board2};

bool myDataReceived = false;
const int buzzer = 2;
String prev_output = " ";

// Callback function to execute when data is received
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  boardsStruct[myData.id - 1].ax = myData.ax;
  boardsStruct[myData.id - 1].ay = myData.ay;
  boardsStruct[myData.id - 1].az = myData.az;
  boardsStruct[myData.id - 1].gx = myData.gx;
  boardsStruct[myData.id - 1].gy = myData.gy;
  boardsStruct[myData.id - 1].gz = myData.gz;
  myDataReceived = true;
}

void setup() {
  // Initialize Serial Monitor and I2C communication
  Wire.begin();
  Serial.begin(115200);
  pinMode(buzzer, OUTPUT);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialize MPU6050
  if (!mpu3.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Configure accelerometer and gyro ranges
  mpu3.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu3.setGyroRange(MPU6050_RANGE_500_DEG);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the callback function to receive data
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Get sensor readings from MPU6050
  sensors_event_t a3, g3, temp3;
  mpu3.getEvent(&a3, &g3, &temp3);

  // Arrays to store data for averaging
  float data[18];
  float total_acc1x = 0.0, total_acc1y = 0.0, total_acc1z = 0.0, total_giro1x = 0.0, total_giro1y = 0.0, total_giro1z = 0.0;
  float total_acc2x = 0.0, total_acc2y = 0.0, total_acc2z = 0.0, total_giro2x = 0.0, total_giro2y = 0.0, total_giro2z = 0.0;
  float total_acc3x = 0.0, total_acc3y = 0.0, total_acc3z = 0.0, total_giro3x = 0.0, total_giro3y = 0.0, total_giro3z = 0.0;

  int i = 0;
  while (i < 15) {
    // If new data has been received, update data array
    if (myDataReceived) {
      data[0] = boardsStruct[0].ax;
      data[1] = boardsStruct[0].ay;
      data[2] = boardsStruct[0].az;
      data[3] = boardsStruct[0].gx;
      data[4] = boardsStruct[0].gy;
      data[5] = boardsStruct[0].gz;
      data[6] = boardsStruct[1].ax;
      data[7] = boardsStruct[1].ay;
      data[8] = boardsStruct[1].az;
      data[9] = boardsStruct[1].gx;
      data[10] = boardsStruct[1].gy;
      data[11] = boardsStruct[1].gz;
    } else {
      // Set default values if no data is received
      for (int j = 0; j < 12; j++) {
        data[j] = 0.0;
      }
    }
    // Update data array with MPU6050 readings from this device
    data[12] = a3.acceleration.x;
    data[13] = a3.acceleration.y;
    data[14] = a3.acceleration.z;
    data[15] = g3.gyro.x;
    data[16] = g3.gyro.y;
    data[17] = g3.gyro.z;

    // Accumulate values for averaging
    total_acc1x += data[0];
    total_acc1y += data[1];
    total_acc1z += data[2];
    total_giro1x += data[3];
    total_giro1y += data[4];
    total_giro1z += data[5];
    total_acc2x += data[6];
    total_acc2y += data[7];
    total_acc2z += data[8];
    total_giro2x += data[9];
    total_giro2y += data[10];
    total_giro2z += data[11];
    total_acc3x += data[12];
    total_acc3y += data[13];
    total_acc3z += data[14];
    total_giro3x += data[15];
    total_giro3y += data[16];
    total_giro3z += data[17];
    i += 1;
    delay(33);
  }

  // Calculate averages
  float avg_acc1x = total_acc1x / 15;
  float avg_acc1y = total_acc1y / 15;
  float avg_acc1z = total_acc1z / 15;
  float avg_giro1x = total_giro1x / 15;
  float avg_giro1y = total_giro1y / 15;
  float avg_giro1z = total_giro1z / 15;
  float avg_acc2x = total_acc2x / 15;
  float avg_acc2y = total_acc2y / 15;
  float avg_acc2z = total_acc2z / 15;
  float avg_giro2x = total_giro2x / 15;
  float avg_giro2y = total_giro2y / 15;
  float avg_giro2z = total_giro2z / 15;
  float avg_acc3x = total_acc3x / 15;
  float avg_acc3y = total_acc3y / 15;
  float avg_acc3z = total_acc3z / 15;
  float avg_giro3x = total_giro3x / 15;
  float avg_giro3y = total_giro3y / 15;
  float avg_giro3z = total_giro3z / 15;

  // Create features array for classification
  float features[18] = {avg_acc1x, avg_acc1y, avg_acc1z, avg_giro1x, avg_giro1y, avg_giro1z,
                        avg_acc2x, avg_acc2y, avg_acc2z, avg_giro2x, avg_giro2y, avg_giro2z,
                        avg_acc3x, avg_acc3y, avg_acc3z, avg_giro3x, avg_giro3y, avg_giro3z};

  // Print features for debugging
  for (int i = 0; i < 18; i++) {
    Serial.print(features[i]);
    if (i < 17) {
      Serial.print(", ");
    }
  }
  Serial.println();

  // Start classification and measure time taken
  unsigned long start_time = micros();
  
  String output_str = rm_classifier.predictLabel(features);
  Serial.println(output_str);
  
  unsigned long end_time = micros();
  unsigned long elapsed_time = end_time - start_time;
  
  Serial.print("Classification time: ");
  Serial.print(elapsed_time);
  Serial.println(" µs");

  // Control buzzer based on classification output
  if (output_str == "BENAR" && prev_output != "BENAR") {
    tone(buzzer, 5000);
    delay(150);
    noTone(buzzer);
    delay(150);
  }
  else if (output_str == "SALAH") {
    noTone(buzzer);
    delay(150);
  }

  prev_output = output_str;
  delay(500);
}