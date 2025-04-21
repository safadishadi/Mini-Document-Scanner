#define CAMERA_MODEL_AI_THINKER  // Uncomment this for AI Thinker ESP32-CAM
#include "esp_camera.h"
#include <WiFi.h>
#include <Arduino.h>
//http://172.20.10.3:80
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

#define PORT_NUMBER       80

// Camera pin configuration for the ESP32-CAM (AI Thinker board)

//#include "camera_pins.h"

// Wi-Fi credentials
const char* ssid = "Shadi safadi";        // Your Wi-Fi SSID
const char* password = "1234shadi";  // Your Wi-Fi Password

// Web server object
WiFiServer server(PORT_NUMBER);  // Set up a web server on port 80
camera_fb_t *fb;
esp_err_t err;

void startCameraServer() {
  server.begin();
  Serial.println("Camera server started");

  while (true) {
    WiFiClient client = server.available();  // Listen for incoming clients
    if (!client) {
      Serial.println("New Client waiting");
      delay(100);
      continue;
    }

    Serial.println("New Client Connected");

    // Wait until the client sends a request
    while (!client.available()) {
      Serial.println("New Client Unavailable");
      delay(100);
    }

    String request = client.readStringUntil('\r');
    Serial.println(request);
    client.flush();  // Clear the client buffer

    digitalWrite(FLASH_GPIO_NUM, HIGH);//Turn flash on

    // Capture an image
    fb = esp_camera_fb_get();  // Capture the image
    if (!fb) {
      Serial.println("Camera capture failed");
      client.print("HTTP/1.1 500 Internal Server Error\r\n");
      client.print("Content-Type: text/html\r\n\r\n");
      client.print("<html><body><h2>Camera Capture Failed</h2></body></html>");
      client.stop();
      return;
    }

    // Send HTTP response headers
    client.print("HTTP/1.1 200 OK\r\n");
    client.print("Content-Type: image/jpeg\r\n");
    client.print("Content-Length: ");
    client.print(fb->len);
    client.print("\r\n\r\n");

    // Send the captured image as a JPEG file
    client.write(fb->buf, fb->len);

    // Return the frame buffer to free memory
    esp_camera_fb_return(fb);
    

    //digitalWrite(FLASH_GPIO_NUM, LOW);//Turn flash off

    delay(1500);  // A brief delay before closing the connection
    Serial.println("Image sent");
    digitalWrite(FLASH_GPIO_NUM, LOW);
    delay(100);
    digitalWrite(FLASH_GPIO_NUM, HIGH);
    delay(100);
    digitalWrite(FLASH_GPIO_NUM, LOW);
    delay(100);
    digitalWrite(FLASH_GPIO_NUM, HIGH);
    client.stop();  // Close the client connection
  }
}

void setup() {
  // Start the serial monitor for debugging
  Serial.begin(115200);
  Serial.println("Starting...");

  // Connect to Wi-Fi
  Serial.println("Connecting to Wi-Fi...");
  Serial.println(PORT_NUMBER);
  Serial.println("Starting...2");
  WiFi.begin(ssid, password);
  Serial.println("Starting...3");
  unsigned long startMillis = millis();
  Serial.println("Starting...4");
  unsigned long timeout = 30000;  // 30 seconds timeout
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    if (millis() - startMillis > timeout) {
      Serial.println("\nWi-Fi connection failed");
      Serial.println("Restarting...");
      ESP.restart();  // Restart if Wi-Fi connection fails
    }
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
  config.jpeg_quality = 12;  //0-63 lower number means higher quality
  config.fb_count = 1;

  // Initialize the camera
  Serial.println("Initializing camera 1 ...");
  err = esp_camera_init(&config);
  Serial.println("Initializing camera 2 ...");
  if (err != ESP_OK) {
    Serial.printf("Camera initialization failed with error 0x%x\n", err);
    Serial.println("Restarting...");
    ESP.restart();  // Restart if camera initialization fails
  }
  Serial.println("Camera initialized");

  //set the flash gpio pin to output mode
  pinMode(FLASH_GPIO_NUM,OUTPUT);
  digitalWrite(FLASH_GPIO_NUM, LOW);//initially off
  // Start the camera server
  startCameraServer();
}

void loop() {
  // Nothing to do here, the server handles everything
  delay(10000);
}
