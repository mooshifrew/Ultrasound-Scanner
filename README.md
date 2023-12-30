# Ultrasound-Scanner

The goal of this project was to create a system that could detect 3 hand gestures in 2-dimensions, as well as take 3D scans of basic geometry. 
 
## Description

This system utilizes various mechanical and hardware components, including 3D printed and steel parts, motors, buttons, a microcontroller, motor shield, and ultrasonic sensors. The software was implemented using Arduino, which controls the components and receives data
from the sensors, and Python, which processes the data and outputs visualizations. The results of the testing show that the system was able to produce differentiable images of 3 hand signs in the 2D scanning mode, and a successful scanning of cylindrical and conical objects in the 3D scanning mode.

System fully assembled: 

![system](https://github.com/mooshifrew/Ultrasound-Scanner/assets/23649910/a627ff21-0805-41e8-ac79-76ba5ac95d49)

2D-scan sample results: 

![2D-signs](https://github.com/mooshifrew/Ultrasound-Scanner/assets/23649910/77e58910-0ffb-48c6-bc1c-c85bf1fdf742)
![2D-scan](https://github.com/mooshifrew/Ultrasound-Scanner/assets/23649910/aee5dfc9-05f0-452b-a382-e09829dbae01)

3D-scanning was tested on a cylinder (water bottle) and cone (clay): 

![cylinder3D](https://github.com/mooshifrew/Ultrasound-Scanner/assets/23649910/611b119c-61f0-4e5d-a95c-5c0840bd0bd5)
![cone3D](https://github.com/mooshifrew/Ultrasound-Scanner/assets/23649910/5525e2bb-275d-46b6-b09d-d58664e03cf2)

## System Overview
The scanning mechanism edited in Solidworks and then converted to STL and 3D-printed:

![3D schematic](https://github.com/mooshifrew/Ultrasound-Scanner/assets/23649910/5ac61440-9f82-465f-901d-1c90eb72ab15)

Code was written in Python and Arduino. The code at a high level runs as follows: 
![Scanner Code](https://github.com/mooshifrew/Ultrasound-Scanner/assets/23649910/d0a96ad5-6c08-45bb-a885-410ac5e373ec)

Packages/libraries used were: 
- Serial
- Numpy
- pyVista
- pyplot
- [Adafruit_Motorshield](https://www.arduino.cc/reference/en/libraries/adafruit-motor-shield-v2-library/)

Hardware components: 
- 12V Unipolar NEMA 17 Stepper (x2)
- Arduino Uno
- Adafruit Motorshield v2.3
- Button (x2)
- Resistor (4.7kOhm) (x2)
- HC-SR04 Ultrasonic Sensor (x5)
- Wire
- Breadboard
- Power-Supply

Hardware components were arranged as follows: 
![circuit](https://github.com/mooshifrew/Ultrasound-Scanner/assets/23649910/ee638805-4212-4530-b868-2a5d9dbfaaaa)


## Usage

To use this project: 
1. Upload and run Arduino_control_code.ino
2. Run scan_processing.py
3. Connect to correct port when prompted
4. Press button to begin scan
   - Green for 2D scan
   - Red for 3D scan


## Credits
Contributors on this project were:
- [Michael Frew](https://github.com/mooshifrew)
- [Vicki Li](https://github.com/v1cki)
- [Ljubica Lolic](https://github.com/ljubicalolic)
- [Sophia Lollino](https://github.com/slollino) 

3D parts were adapted from the design by [SuperMakeSomething](https://www.thingiverse.com/thing:1413891) 
