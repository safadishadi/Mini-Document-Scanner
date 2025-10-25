# Mini-Document-Scanner


**Project Overview:**

This project is a software and hardware implementation of a miniature sized document scanner. Using a circuit board with sensors and a microcontroller, the functionality is an intuitive user friendly interface that automates the image capturing and the transmittion of the images via wifi to a server that hosts the main algorithm, which stitches the images and produces a final result.

**System Requirements**

Hardware:

* ESP32-CAM module.
* VL53L0X Time-of-Flight distance sensor.
* MPU6050 accelerometer-gyroscope module.
* Standard USB-to-Micro-USB cable for power and programming.
* circuit board, connection wires, resistors andcapacitors as shown in hardware diagram.
* Laptop/PC/Powerbank with a USB output acts as a power supply.

Software:

* Arduino IDE ≥ 2.2 with ESP32 board package.
* Python ≥ 3.9 on host PC.
* Required Python libraries:
  * pip install opencv-python numpy socket
* Required Arduino IDE libraries:
  * VL53L0X by Pololu.
  * MPU6050 by Electronic Cats.
* Operating Systems tested: Windows 10/11.

**Software Overview:**

* The microcontrollers code is implemented in c in arduinoIDE, it initializes and controlles the sensores (Camera, Laser Distance, Gyroscope, Accelerometer), and is responsible for doing spatial and temporal calculation to figure position, orientation and velocity, in order to send it as metadata with each image.
* The main stitching algorithm is implemented in python, it consists of a basic server that recieves images corresponding to parts of the document and reconstructs them into a final result.

**Operation Guide:**

- Updating embedded code and uploading it to esp32:

  - Connect GPIO 0 to GND.
  - In Arduino IDE press upload.
  - Remove GPIO 0 from GND only once compilation and upload are done.
  - Press reset on ESP32 board.
- Connecting esp32 to Wifi:

  - update the desired network name and password in the embedded code specified field. upon powering up, the esp32 will connect automatically.
- Connecting laptop or PC to the same local network.
- Typing 172.20.10.3 in browser to see captured images (optional).
- Running Python code that starts capturing images and stitching them in rows, and then stitching the rows into a full scan:

  * start by placing the board on a level plane.
  * restart circuit to activate DMP calibration.
  * after calibration is done, lift board to face document with a distance greater than 60cm.
  * bring board closer than 60cm to automatically start scanning.
  * do a double capture of the first image of first row.
  * move as horizontal rows.
  * at the end of each row, lower the board at a decent (not slow) speed, to start next row.
  * do a double capture for the last image of last row.
  * flip or distance the board (>60cm) to automatically stop capturing.
  * python code should automatically stitch the images and give final result (or error code).

**Project Map:**

```
├── src/
│   ├── EmbeddedCode/
│      └── EmbeddedCode.ino
│   └── PythonCode/
│      └── main.py
└── README.md
```


**Results:**

1. Optimal Document Size:
   Best performance was achieved when scanning documents between 40×60 cm and  100×80 cm , where image overlap and focus were most consistent.
2. Stitching Success Rate:
   The system reached up to  95% successful stitching , with around 80% producing clear, well-aligned final scans under stable lighting and motion conditions.

**Limitations:**

* Image quality depends on stable lighting and proper manual focus (20–60 cm).
* Lack of features or textures can disrupt stitching.
* Wi-Fi connection stability directly affects image transmission timing.
* Limited ESP32-CAM memory restricts on-board processing limits stitching to host-side only.

**Future Improvements:**

* Add automatic focus and exposure control.
* Replace MPU6050 with BNO055 for better orientation accuracy
* Enable cloud-based or distributed stitching for faster results.

**References and Acknowledgments:**

* ESP32-CAM , Espressif Systems documentation.
* VL53L0X Datasheet , STMicroelectronics.
* MPU6050 Register Map , InvenSense.
* OpenCV Documentation , Feature detection & stitching modules.


**Authors:**

Hani Sabehiy

Shadi Safadi
