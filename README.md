# jetson_arduino

# Air Quality Measurement Project

## 📖 Overview
This system measures indoor air quality using a **CM1106 CO2 Sensor** and a **Grove - Dust Sensor**, then analyzes and outputs the data.  
By connecting the sensors to an Arduino equipped with a Grove Base Shield, you can monitor the measured CO2 and fine dust concentrations via the serial monitor.

---

## 📦 Key Features
1. **Fine Dust Concentration Measurement (µg/m³)**  
   - Utilizes the Grove - Dust Sensor to measure fine dust levels.  
   - Samples the duration of LOW signals and calculates air quality status based on this.

2. **CO2 Concentration Measurement (ppm)**  
   - Measures CO2 levels using the CM1106 sensor.  
   - Reads data from the sensor via UART communication and validates the values.

3. **Data Output**  
   - Displays real-time concentration data on the serial monitor.  
   - Assesses air quality based on both fine dust and CO2 readings.

---

## 🛠️ Hardware Configuration

### 1. Components Used
- Arduino Uno  
- Grove Base Shield  
- **Grove - Dust Sensor**  
- **CM1106 CO2 Sensor**  
- Jumper wires and a USB cable

### 2. Hardware Connections
#### Grove - Dust Sensor Connection
- Connect to the **D8 port** on the Grove Base Shield.  
- If not using the Base Shield:
  - **5V** → Arduino 5V pin  
  - **GND** → Arduino GND pin  
  - **D8** → Arduino digital pin 8  

#### CM1106 Sensor Connection
- Connect to the **D2 port** on the Grove Base Shield.  
- If not using the Base Shield:
  - **5V** → Arduino 5V pin  
  - **GND** → Arduino GND pin  
  - **TXD (yellow wire)** → Arduino RX pin (D2)  
  - **RXD (blue wire)** → Arduino TX pin (D3)  

---

## 💻 Software Setup

### 1. Required Library
- **SoftwareSerial**: For UART communication.

### 2. Code
Below is the Arduino code used in this project:

```cpp
#include <SoftwareSerial.h>

// Set up SoftwareSerial for the CM1106 sensor
SoftwareSerial mySerial(2, 3); // RX: D2, TX: D3

// Grove - Dust Sensor pin
int dust_pin = 8;
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 30000; // 30-second sampling time
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float dust_concentration = 0;

// CM1106 data storage
byte receivedData[9];
int co2_concentration = 0;

void setup() {
    Serial.begin(9600);
    mySerial.begin(9600); // Initialize CM1106
    pinMode(dust_pin, INPUT);
    starttime = millis();
    Serial.println("Air quality measurement system initialized");
}

void loop() {
    // Dust Sensor measurement
    duration = pulseIn(dust_pin, LOW);
    lowpulseoccupancy += duration;

    if ((millis() - starttime) > sampletime_ms) {
        ratio = lowpulseoccupancy / (sampletime_ms * 10.0);
        dust_concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62;

        Serial.println("==============================");
        Serial.print("Fine Dust Concentration (µg/m³): ");
        Serial.println(dust_concentration);

        lowpulseoccupancy = 0;
        starttime = millis();
    }

    // CM1106 CO2 measurement
    readCM1106();
    Serial.print("CO2 Concentration (ppm): ");
    if (co2_concentration != -1) {
        Serial.println(co2_concentration);
    } else {
        Serial.println("Data Error");
    }

    delay(1000); // 1-second interval
}

void readCM1106() {
    if (mySerial.available() >= 9) {
        mySerial.readBytes(receivedData, 9);

        if (receivedData[0] == 0xFF && receivedData[1] == 0x86) {
            co2_concentration = (receivedData[2] << 8) | receivedData[3];
        } else {
            co2_concentration = -1;
        }
    }
}
```

---

## 🧪 Testing Procedure

1. **Upload the Code to Arduino After Hardware Connection**  
   - Upload the code through the Arduino IDE.

2. **Sensor Stabilization**  
   - The CM1106 requires about 3–5 minutes of warm-up time after initialization.

3. **Check the Serial Monitor**  
   - Open the Arduino IDE serial monitor and set the baud rate to `9600 bps`.  
   - You will see the fine dust and CO2 concentrations displayed.

4. **Sample Output**  
   ```
   ==============================
   Fine Dust Concentration (µg/m³): 42.36
   CO2 Concentration (ppm): 415
   ```

---

## 📊 Key Output Values

- **Fine Dust Concentration (µg/m³)**:
  - 0–30: Good
  - 31–80: Moderate
  - 81–150: Unhealthy
  - 151+: Very Unhealthy

- **CO2 Concentration (ppm)**:
  - 0–400: Very Clean
  - 401–1000: Good
  - 1001–2000: Fair
  - 2001+: Poor

---

## 💡 Additional Implementation Ideas
- **Warning Function for Exceeding Thresholds**  
  - Display alerts when fine dust or CO2 concentrations exceed certain standards.
- **Voice Feedback System**  
  - Provide audio alerts or status messages about air quality.
- **IoT Extension**  
  - Upload data to the cloud for remote monitoring.

---

## 📋 References
- [Grove - Dust Sensor Datasheet](https://wiki.seeedstudio.com/Grove-Dust_Sensor/)  
- [CM1106 Datasheet](https://www.co2meter.com/products/cm1106-co2-sensor)
