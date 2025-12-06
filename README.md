# HomeClimate
OBE5043:Advance Embedded System Project 

Student ID : 24001946
Project Title : Smart Home Climate Monitoring & Alert System (ESP32 + RainMaker)

Overview
This project implements a web-connected climate monitoring system using the ESP32 and the ESP RainMaker cloud platform.
It measures temperature and humidity, triggers local alerts using LEDs and a buzzer, and allows users to remotely control the system through:
 - RainMaker mobile / web dashboard
 - Amazon Alexa & Google Assistant voice commands
 - Over-the-Air (OTA) firmware updates

The system runs on ESP-IDF with FreeRTOS, ensuring clean task separation and real-time performance.

Repository structure
/main
 ├── app_main.c        → Main project source code (tasks, logic, cloud, OTA)
 ├── CMakeLists.txt    → Build configuration + firmware versioning
 └── app_priv.h        → Optional helper definitions from RainMaker examples

/README.md             → Project documentation (this file)
/sdkconfig             → ESP-IDF configuration (auto-generated)
/components            → Additional ESP-IDF components (if required)

Key Features
+Environmental Monitoring
 - Reads temperature & humidity using DHT11
 - Updates sensor values every 2 seconds
 - Pushes readings to RainMaker cloud dashboard

+Alert System
 - LEDs indicate temperature/humidity alerts:
   - Green → Normal
   - Red → High temperature
   - Blue → High humidity

 - Buzzer activates when thresholds are exceeded
 - Alerts follow configurable user settings

+Cloud Integration (ESP RainMaker)
 - Remote monitoring through app/web
 - Control from anywhere
 - Supports:
   - Temperature threshold
   - Humidity threshold
   - Alarm enable/disable
   - Buzzer enable/disable
   - Operating modes (Normal / Hot Day / Cold Day)

+Voice Assistant Support
 - Fully compatible with Alexa and Google Assistant
 - Example commands:
     “Alexa, set HomeClimate mode to Hot Day.”

+OTA Firmware Updates
 - Update firmware remotely without USB cable
 - Version tracking via:
    - CMakeLists.txt
    - esp_rmaker_node_add_fw_version()
 - Safe rollback handled by ESP-IDF bootloader

+FreeRTOS Multitasking
 - sensor_task → Reads DHT11
 - cloud_task → Reports values and evaluates alerts
 - Lightweight inter-task communication using global variables

Testing Summary
 - Sensor readings stable
 - Alerts respond correctly to threshold changes
 - Voice commands recognized consistently
 - OTA updates validated successfully
 - FreeRTOS tasks run without starvation or watchdog resets

Future Improvements
 - Upgrade sensor to BME280 for better accuracy
 - Add CO₂ or air-quality sensors
 - Implement historical data logging
 - Add OLED/LCD local display
 - Improve inter-task communication using queues or event groups
 - Create a full web interface for visualization

License
This project is for academic and research purposes.
You may modify and expand the code as needed for learning or development.
