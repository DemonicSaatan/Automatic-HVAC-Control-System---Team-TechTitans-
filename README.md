Automatic HVAC Control System

Developed by Team TechTitans

An automated, microcontroller-based Heating, Ventilation, and Air Conditioning (HVAC) control system implemented using the ESP32. This system dynamically monitors ambient temperature, humidity, and environmental parameters to automate climate regulation, optimize energy consumption, and provide real-time control.

Features: 

Automated Climate Regulation: Automatically triggers heating, cooling, or ventilation routines based on real-time environmental thresholds.
ESP32 Powered: Utilizes the dual-core processing, GPIO flexibility, and wireless connectivity of the ESP32 platform.
Energy Optimization: Reduces unnecessary power draw by dynamically cycling heating/cooling actuators only when setpoints are exceeded.
Real-Time Environmental Monitoring: Continuously samples sensor data (temperature and humidity) for fast response times.

Hardware Requirements:

Microcontroller: ESP32 Development Board (ESP-WROOM-32 or equivalent)
Sensors: Temperature & Humidity sensor (e.g., DHT11 / DHT22 or BME280)
Actuators & Relays: Relay module (for AC / Heater switching)
                    DC Fan / Blower (for ventilation control)
                    Status Indicator LEDs (heating/cooling indicators)
Power Supply: 5V / 12V DC power source appropriate for relays and ESP32
Interconnects: Breadboard and jumper wires

Circuit & Pin Configuration

Update the pin assignments below to match your specific pin definitions in ESP32_Code.ino.

Component               ESP32 Pin                 Function
Sensor Data (DHT/BME)    GPIO 4               Ambient temperature/humidity reading
Cooling / AC Relay       GPIO 18              Triggers cooling system
Heater Relay             GPIO 19              Triggers heating system
Fan / Ventilation PWM    GPIO 21              Controls ventilation speed
Status / Alert LED       GPIO 2               System health & mode indicator

Software & Library Setup: 
Prerequisites
Arduino IDE (v1.8+ or 2.x) or PlatformIO
ESP32 Board Support Package installed in Arduino IDE Boards Manager

Required Libraries

Install the following via the Arduino IDE Library Manager (Sketch > Include Library > Manage Libraries...): 

DHT sensor library by Adafruit (or Adafruit BME280 Library)

Adafruit Unified Sensor 

Installation & Flashing:

1. Clone the repository:
Bash

git clone https://github.com/DemonicSaatan/Automatic-HVAC-Control-System---Team-TechTitans-.git
cd Automatic-HVAC-Control-System---Team-TechTitans-

Open the sketch: 

Launch Arduino IDE and open ESP32_Code.ino.

Configure Settings:

Review and set your desired temperature setpoints, thresholds, and pin definitions at the top of the file.

Select Board & Port:

Go to Tools > Board and choose your ESP32 board (e.g., ESP32 Dev Module).
Go to Tools > Port and select the appropriate serial COM port.
Upload:

Click Upload (or press Ctrl + U).
Open the Serial Monitor at 115200 baud to view sensor readings and control logs.

Team TechTitans
Project Repository: Automatic-HVAC-Control-System---Team-TechTitans-
