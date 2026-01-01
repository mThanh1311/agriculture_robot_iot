#  Agriculture Robot – STM32 + ESP32 + AI + IoT

##  Project Overview
This project focuses on building an **agriculture robot platform** using **STM32** as the main embedded controller for real-time sensor processing and control, combined with **ESP32** for AI capabilities and IoT cloud connectivity.

The system is designed to:
- Collect motion and environmental data from multiple sensors
- Perform sensor fusion and control algorithms on STM32
- Transfer processed data to ESP32 for AI inference and cloud publishing
- Visualize and monitor data on **ThingsBoard**

This repository also serves as a **development roadmap** using a ToDo checklist.

---

##  Knowledge & Technologies

###  Microcontrollers
- STM32 (HAL / bare-metal / interrupt-based programming)
- ESP32 / ESP32-CAM

###  Communication Protocols
- I2C
- UART
- PWM
- GPIO / EXTI

###  Sensors
- **MPU6050**
  - Accelerometer
  - Gyroscope
  - Orientation and motion sensing
- **SHT3x**
  - Temperature
  - Humidity
- Additional temperature and environmental sensors (future)

###  Control & Algorithms
- Sensor Fusion (Kalman / Complementary Filter)
- Tilt angle estimation
- PID Controller
- Real-time control loop

###  AI & IoT
- Edge AI on ESP32 (TinyML / lightweight CNN)
- ThingsBoard integration
- Cloud data visualization

---

##  ToDo List

### STM32 & MPU6050
- [ ] STM32 I2C communication with MPU6050
- [ ] Read raw accelerometer and gyroscope data
- [ ] Sensor calibration

### Sensor Fusion & PID Control
- [ ] Sensor Fusion
  - [ ] Accelerometer + Gyroscope fusion
  - [ ] Kalman Filter implementation
- [ ] Tilt angle calculation
- [ ] PID Controller
  - [ ] Proportional term
  - [ ] Integral term
  - [ ] Derivative term
  - [ ] Parameter tuning
- [ ] PWM output generation

### Environmental Sensors
- [ ] STM32 I2C communication with SHT3x
- [ ] Temperature measurement
- [ ] Humidity measurement

### STM32 ↔ ESP32 Communication
- [ ] UART communication setup
- [ ] Data packet structure definition
- [ ] Sensor data transfer
- [ ] Error handling

### AI Integration (ESP32)
- [ ] Plant / bean disease recognition model
- [ ] ESP32-CAM image acquisition
- [ ] AI inference pipeline
- [ ] Merge AI output with sensor data

### IoT & ThingsBoard
- [ ] Send telemetry data to ThingsBoard
- [ ] Device configuration
- [ ] Data structure design

### Dashboard & Visualization
- [ ] ThingsBoard dashboard design
- [ ] Real-time charts and widgets

### Final Demo
- [ ] Full system integration
- [ ] Demo video / images
- [ ] Performance evaluation

---

##  Expected Results
- Real-time sensor data acquisition
- Stable sensor fusion and PID control
- Reliable STM32–ESP32 communication
- AI-based plant disease detection
- Live visualization on ThingsBoard

---

##  Future Work
- Add soil moisture and light sensors
- Improve AI model accuracy and optimization
- Autonomous navigation
- Power optimization
- Field-ready agriculture robot prototype

---

##  Planned Repository Structure
```
firmware-stm32/
firmware-esp32/
ai-model/
thingsboard/
```


---

##  Demo & Documentation
Coming soon.
