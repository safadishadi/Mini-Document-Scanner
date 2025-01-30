# Mini-Document-Scanner

**Project Overview:**

This project is is a software and hardware implementation of a miniature sized document scanner.  Using a circuit board with sensors and a microcontroller, the functionality is an intuitive user friendly interface that automate the image capturing and the transmittion of the images via wifi to a server that hosts the main image stitching algorithm that produces the final result.

**Software Components:**

* The microcontrollers code is implemented in c in arduinoIDE, it initializes and controlles the sensores (Camera, Laser Distance, Gyroscope, Accelerometer, Antenna).
* The main stitching algorithm is implemented in python, it consists of a basic server that Recieves images corresponding to parts of the document and reconstructs them into a final result.

**Authors:**

Hani Sabihi

Shadi Safadi
