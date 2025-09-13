# Mini-Document-Scanner

**Project Overview:**

This project is is a software and hardware implementation of a miniature sized document scanner.  Using a circuit board with sensors and a microcontroller, the functionality is an intuitive user friendly interface that automate the image capturing and the transmittion of the images via wifi to a server that hosts the main image stitching algorithm that produces the final result.

**Software Components:**

* The microcontrollers code is implemented in c in arduinoIDE, it initializes and controlles the sensores (Camera, Laser Distance, Gyroscope, Accelerometer, Antenna).
* The main stitching algorithm is implemented in python, it consists of a basic server that Recieves images corresponding to parts of the document and reconstructs them into a final result.

**Operation Guide:**

- Updating embedded code and uploading it to esp32:

  - Connect GPIO 0 to GND
  - In Arduino IDE press upload
  - Remove GPIO 0 from GND only once compilation and upload are done
  - Press reset on ESP32 board
- Connecting esp32 to Wifi:

  - update the desired network name and password in the embedded code specified field. upon powering up, the esp32 will connect automatically.
- Connecting laptop or PC to the same local network
- Typing 172.20.10.3 in browser to see captured images
- Running Python code that startscapturing images and stitching themin rows, andthen stitching the rows into a full scan:

  * start by placing the board on a level plane, withits buttom rotated towards the upstanding document.
  * restart circuit to activate DMP calibration.
  * after calibration is done, lift board to face document with a distance greater than 60cm.
  * wait until z displacement measurment stabilizes as a positive number.
  * bring board closer than 60cm to automatically start scanning.
  * move as horizontal rows, while doubling the capturing at the start of each row.
  * at the end of each row, lower the board at a decent, not slow speed, (z measurement goes negative) to start next row.
  * do a double capture for the last image of last row.
  * flip or distance the board (>60cm) to automatically stop capturing.
  * python code should automatically stitch and give final result (or error code).

**Project Map:**

```
├── src/
│   ├── EmbeddedCode/
│      ├── EmbeddedCode.ino
│      └── TODO
│   └── PythonCode/
│      ├── TODO
│      └── TODO
├── tests/
│   ├── TODO
│   └── TODO
└── README.md
```

**Authors:**

Hani Sabihi

Shadi Safadi
