#define CAMERA_MODEL_AI_THINKER  // Uncomment this for AI Thinker ESP32-CAM
#include "esp_camera.h"
#include <WiFi.h>
#include <Arduino.h>
#include <MPU6050.h>
#include <VL53L0X.h>

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define FLASH_GPIO_NUM     4

#define SDA_PIN 13
#define SCL_PIN 14

#define PORT_NUMBER       80

MPU6050 mpu;
VL53L0X lox;

// Wi-Fi credentials
const char* ssid = "Shadi safadi";        
const char* password = "1234shadi";  

WiFiServer server(PORT_NUMBER); // Set up a web server on port 80
camera_fb_t *fb;
esp_err_t err;
float Ax, Ay, Az, Gx, Gy, Gz;
uint16_t distance;

void scanI2C() {
  Serial.println("Scanning for I2C devices...");
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(address, HEX);
    }
  }
  Serial.println("Finished scanning for I2C devices...");
}

void handleCaptureRequest(WiFiClient& client, float dxt, float dyt, float dzt, float Gxt, float Gyt, float Gzt) {

  // Capture the image
  digitalWrite(FLASH_GPIO_NUM, HIGH); // Turn flash on
  fb = esp_camera_fb_get();  // Capture the image
  if (!fb) {
    Serial.println("Camera capture failed");
    client.print("HTTP/1.1 500 Internal Server Error\r\n");
    client.print("Content-Type: text/html\r\n\r\n");
    client.print("<html><body><h2>Camera Capture Failed</h2></body></html>");
    client.stop();
    return;
  }

  Serial.println("Camera capture succeeded");

  // Send JSON with sensor data and the image
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println("Connection: close");
  client.println();

  client.print("--frame\r\n");
  client.print("Content-Type: application/json\r\n");
  client.print("Content-Length: ");
  
  distance = lox.readRangeSingleMillimeters();
  if (lox.timeoutOccurred()) {
    Serial.println("VL53L0X timeout");
  }

  String json = "{";
  json += "\"displacement_cm_x\":" + String(dxt) + ",";
  json += "\"displacement_cm_y\":" + String(dyt) + ",";
  json += "\"displacement_cm_z\":" + String(dzt) + ",";
  json += "\"gyro_x\":" + String(Gxt) + ",";
  json += "\"gyro_y\":" + String(Gyt) + ",";
  json += "\"gyro_z\":" + String(Gzt) + ",";
  json += "\"distance_cm\":" + String(distance/10.0f);
  json += "}";

  client.print(json.length());
  client.println();
  client.println();
  client.print(json);
  client.println();

  client.print("--frame\r\n");
  client.print("Content-Type: image/jpeg\r\n");
  client.print("Content-Length: " + String(fb->len));
  client.println();
  client.println();
  client.write(fb->buf, fb->len);
  
  client.print("\r\n--frame--\r\n");

  esp_camera_fb_return(fb);
  digitalWrite(FLASH_GPIO_NUM, LOW); // Turn flash off
  return;
}

void startCameraServer() {
  float dxt=0, dyt=0, dzt=0, dx=0, dy=0, dz=0, vx=0, vy=0, vz=0, Gxt=0, Gyt=0, Gzt=0, passedTime=0, numSamples=0;
  float dt = 0.02f;  // 20 ms delay = 0.02 s
  float prevAx = 0, alpha = 0.5; // tune as needed
  unsigned long lastTime, nowTime;

  server.begin();
  Serial.println("Camera server started");
  lastTime = millis();

  while (true) {
    WiFiClient client = server.available(); // Listen for incoming clients
    if (!client) {
      Serial.println("New Client waiting");
      // Read sensor values
      Ax = (mpu.getAccelerationX()/ 16384.0f)* 980.665f; // in cm/s^2
      Ax = alpha * Ax + (1 - alpha) * prevAx;
      prevAx = Ax;
      Ay = (mpu.getAccelerationY()/ 16384.0f)* 980.665f;
      Az = (mpu.getAccelerationZ()/ 16384.0f)* 980.665f;
      // Integrate acceleration to update velocity
      vx += Ax * dt;
      vy += Ay * dt;
      vz += Az * dt;
      // Integrate velocity to update displacement
      dxt += vx * dt;
      dyt += vy * dt;
      dzt += vz * dt;

      Gxt += mpu.getRotationX()/ 131.0;
      Gyt += mpu.getRotationY()/ 131.0;
      Gzt += mpu.getRotationZ()/ 131.0;

      //TODO: delete
      Ax=mpu.getAccelerationX();
      Gx=mpu.getRotationX();
      Serial.print("Raw Ax: "); Serial.println((Ax/ 16384.0f));//(  Ax > 150 ? Ax : 0 );
      Serial.print(" Raw Gx: "); Serial.println(Gx);

      numSamples++;
      delay(20);
      continue;
    }

    Serial.println("New Client Connected");

    // Wait until the client sends a request
    while (!client.available()) {
      Serial.println("Waiting For Client Request.");
      // Read sensor values
      Ax = (mpu.getAccelerationX()/ 16384.0f)* 980.665f; // in cm/s^2
      Ax = alpha * Ax + (1 - alpha) * prevAx;
      prevAx = Ax;
      Ay = (mpu.getAccelerationY()/ 16384.0f)* 980.665f;
      Az = (mpu.getAccelerationZ()/ 16384.0f)* 980.665f;
      // Integrate acceleration to update velocity
      vx += Ax * dt;
      vy += Ay * dt;
      vz += Az * dt;
      // Integrate velocity to update displacement
      dxt += vx * dt;
      dyt += vy * dt;
      dzt += vz * dt;

      Gxt += mpu.getRotationX()/ 131.0;
      Gyt += mpu.getRotationY()/ 131.0;
      Gzt += mpu.getRotationZ()/ 131.0;
      numSamples++;
      delay(10);
    }
    nowTime = millis();
    passedTime = (nowTime - lastTime)/1000.0;
    lastTime = millis();
    String request = client.readStringUntil('\r');
    client.flush(); // Clear the client buffer

    if (request.indexOf("GET /capture") >= 0) {
      Serial.println("Image and sensor data requested");
      handleCaptureRequest(client, dxt, dyt, dzt, Gxt, Gyt, Gzt);
      client.stop();
      dxt=0; dyt=0; dzt=0; Gxt=0; Gyt=0; Gzt=0; numSamples=0;
      vx = vy = vz = 0;
      continue;
    }
    dxt=0; dyt=0; dzt=0; Gxt=0; Gyt=0; Gzt=0; numSamples=0;
    vx = vy = vz = 0;
    client.stop();
  }
}

void setup() {
  // Start the serial monitor for debugging
  Serial.begin(115200);
  Serial.println("Starting...");

  // Initialize I2C with custom SDA and SCL pins
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // I2C, 100 kHz for stability
  Wire.setClock(100000); 

  // Scan and print I2C devices
  scanI2C();
  Serial.println("Scanned for I2C devices");

  // Initialize MPU6050
  mpu.initialize();
  delay(100);
  if (!mpu.testConnection()) {
    Serial.println("Failed to initialize MPU6050!");
    while (1);
  }
  Serial.println("MPU6050 initialized.");

  // Initialize VL53L0X
  if (!lox.init()) {
    Serial.println("Failed to initialize VL53L0X!");
    while (1);
  }
  Serial.println("VL53L0X initialized.");
  lox.setTimeout(5000);
  lox.startContinuous();
  Serial.println("VL53L0X configured.");

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected to Wi-Fi");
  Serial.println(WiFi.localIP());

  // Camera configuration
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_UXGA;
  config.jpeg_quality = 12;  // 0-63 lower number means higher quality
  config.fb_count = 1;

  // Initialize the camera
  err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera initialization failed with error 0x%x\n", err);
    ESP.restart();  // Restart if camera initialization fails
  }
  Serial.println("Camera initialized");

  // Set the flash GPIO pin to output mode
  pinMode(FLASH_GPIO_NUM, OUTPUT);
  digitalWrite(FLASH_GPIO_NUM, LOW); // Initially off

  // Start the camera server
  startCameraServer();
}

void loop() {
  // Nothing to do here, the server handles everything
  delay(1000);
}
