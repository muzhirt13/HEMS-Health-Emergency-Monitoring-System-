# HEMS-Health-Emergency-Monitoring-System-
OBE5043: Advanced Embedded System Project
Developed with guidance and support from Dr. Zuki

This project—HEMS (Health Emergency Monitoring System)—was developed as part of the course OBE5043: Advanced Embedded System Project.
Special acknowledgment and heartfelt appreciation go to Dr. Zuki, whose expertise, continuous support, and guidance greatly contributed to the successful implementation of this system.

📌 Project Overview

HEMS is an integrated real-time health and emergency monitoring system built using the ESP32-C3 and ESP RainMaker Cloud Platform.
It continuously monitors vital health parameters and can automatically trigger emergency alerts when dangerous conditions are detected.

HEMS monitors:

Body Temperature — via MCP9700 analog sensor

Heart Rate (BPM) — via KS0015 pulse sensor

Fall Detection — via MPU6050 accelerometer

Emergency Buzzer Alerts

False Alarm Detection via “Help Found” button

Real-time OLED readout of sensor data

Automatically sends mobile notifications for:

Fall detected

High BPM

High temperature

False alarm / alert cleared

Users can also:

View live BPM & temperature from the app

Use an in-app toggle to activate/deactivate sensors

Control sensors using Google Assistant voice commands

Acknowledge alarms through the cloud or hardware button

🧩 System Components & Technologies
🟦 ESP32-C3 (XIAO ESP32-C3)

RISC-V CPU

Wi-Fi + BLE

Ultra-low power

Integrated ADC

Ideal for IoT + real-time monitoring

🌡 MCP9700 Temperature Sensor

Linear analog output

10mV/°C sensitivity

ADC-based temperature measurement

Accurate in 0–70°C range

❤️ KS0015 Pulse Sensor

IR-based photoplethysmography

Analog waveform sensing

Converted into a simplified BPM value

Provides early detection for abnormal heart rate

📉 MPU6050 Accelerometer

3-axis digital accelerometer

Used for fall detection using:

Y-axis threshold

Total acceleration magnitude

A fall-latch that keeps buzzer ON until acknowledged

🔈 Buzzer + Help Found Button

Buzzer sounds during emergencies

Button clears alarm, sends False Alarm notification

Integrated with RainMaker state sync

🖥 SSD1306 OLED Display (I²C)

Displays:

Temperature

BPM

Sensor status

Emergency warnings

Fall detected messages

📱 ESP RainMaker (Cloud + Mobile App + Voice Assistant)

Provides:

Wi-Fi provisioning

Live sensor telemetry

Notifications

Device control (on/off for each sensor)

Google Assistant support

Time-series data logging

🛠 ESP-IDF (Framework)

Handles:

FreeRTOS tasks

I²C communication

ADC sampling

GPIO control

Wi-Fi & cloud integration

Logging/monitoring tools

📡 System Architecture Diagram
      +---------------------------------------------------+
      |                   HEMS System                     |
      +---------------------------------------------------+
      |  ESP32-C3                                          |
      |  - MCP9700 (Temp)                                  |
      |  - KS0015 (Pulse)                                  |
      |  - MPU6050 (Fall)                                  |
      |  - OLED Display                                    |
      |  - Buzzer + Help Button                            |
      +---------------------------------------------------+
                      |
                      ▼
     +----------------------------------------+
     |          ESP RainMaker Cloud           |
     | - Mobile App (iOS/Android)             |
     | - Alerts & Notifications               |
     | - Live Telemetry (BPM/Temp)            |
     | - Google Assistant Voice Control       |
     +----------------------------------------+

