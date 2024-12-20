# jetson_arduino

# 공기질 측정 프로젝트

## 📖 개요
**CM1106 CO2 센서**와 **Grove - Dust Sensor**를 활용하여 실내 공기질을 측정하고, 데이터를 분석 및 출력하는 시스템입니다.  
Arduino와 Grove Base Shield를 사용하여 센서를 연결하고, 측정된 CO2 및 미세먼지 농도를 시리얼 모니터를 통해 확인할 수 있습니다.

---

## 📦 주요 기능
1. **미세먼지 농도 측정 (µg/m³)**:
   - Grove - Dust Sensor를 사용하여 미세먼지 농도를 측정합니다.
   - LOW 신호 지속 시간을 샘플링하고, 이를 통해 공기질 상태를 계산합니다.

2. **CO2 농도 측정 (ppm)**:
   - CM1106 센서를 통해 CO2 농도를 측정합니다.
   - UART 통신을 통해 센서에서 데이터를 읽고 유효성을 검증합니다.

3. **데이터 출력**:
   - 시리얼 모니터를 통해 실시간 농도 데이터를 출력합니다.
   - 미세먼지와 CO2 농도에 따라 공기질 상태를 평가할 수 있습니다.

---

## 🛠️ 하드웨어 구성

### 1. 사용한 부품
- Arduino Uno
- Grove Base Shield
- **Grove - Dust Sensor**
- **CM1106 CO2 센서**
- 점퍼 케이블 및 USB 케이블

### 2. 하드웨어 연결
#### Grove - Dust Sensor 연결
- Grove Base Shield의 **D8 포트**에 연결.
- Base Shield 없이 연결하는 경우:
  - **5V** → Arduino 5V 핀
  - **GND** → Arduino GND 핀
  - **D8** → Arduino 디지털 핀 8

#### CM1106 센서 연결
- Grove Base Shield의 **D2 포트**에 연결.
- Base Shield 없이 연결하는 경우:
  - **5V** → Arduino 5V 핀
  - **GND** → Arduino GND 핀
  - **TXD (노란색)** → Arduino RX 핀 (D2)
  - **RXD (파란색)** → Arduino TX 핀 (D3)

---

## 💻 소프트웨어 설정

### 1. 필수 라이브러리
- **SoftwareSerial**: UART 통신을 위한 라이브러리.

### 2. 코드
아래는 프로젝트에서 사용한 아두이노 코드입니다.

```cpp
#include <SoftwareSerial.h>

// CM1106 센서용 직렬 통신 핀 설정
SoftwareSerial mySerial(2, 3); // RX: D2, TX: D3

// Grove - Dust Sensor 핀
int dust_pin = 8;
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 30000; // 샘플링 시간: 30초
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float dust_concentration = 0;

// CM1106 데이터 저장 변수
byte receivedData[9];
int co2_concentration = 0;

void setup() {
    Serial.begin(9600);
    mySerial.begin(9600); // CM1106 초기화
    pinMode(dust_pin, INPUT);
    starttime = millis();
    Serial.println("공기질 측정 시스템 초기화 완료");
}

void loop() {
    // Dust Sensor 측정
    duration = pulseIn(dust_pin, LOW);
    lowpulseoccupancy += duration;

    if ((millis() - starttime) > sampletime_ms) {
        ratio = lowpulseoccupancy / (sampletime_ms * 10.0);
        dust_concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62;

        Serial.println("==============================");
        Serial.print("미세먼지 농도 (µg/m³): ");
        Serial.println(dust_concentration);

        lowpulseoccupancy = 0;
        starttime = millis();
    }

    // CM1106 CO2 농도 측정
    readCM1106();
    Serial.print("CO2 농도 (ppm): ");
    if (co2_concentration != -1) {
        Serial.println(co2_concentration);
    } else {
        Serial.println("데이터 오류");
    }

    delay(1000); // 1초 간격
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

## 🧪 테스트 방법

1. **하드웨어 연결 후 Arduino에 코드 업로드**
   - Arduino IDE에서 코드를 업로드합니다.

2. **센서 안정화**
   - CM1106은 초기화 후 약 3~5분 동안 예열 시간이 필요합니다.

3. **시리얼 모니터 확인**
   - Arduino IDE의 시리얼 모니터를 열고, 통신 속도를 `9600 bps`로 설정합니다.
   - 미세먼지 농도와 CO2 농도가 출력됩니다.

4. **출력 예시**
   ```
   ==============================
   미세먼지 농도 (µg/m³): 42.36
   CO2 농도 (ppm): 415
   ```

---

## 📊 주요 출력값
- **미세먼지 농도 (µg/m³)**:
  - 0~30: 좋음
  - 31~80: 보통
  - 81~150: 나쁨
  - 151 이상: 매우 나쁨

- **CO2 농도 (ppm)**:
  - 0~400: 청정
  - 401~1000: 양호
  - 1001~2000: 보통
  - 2001 이상: 나쁨

---

## 💡 추가 구현 아이디어
- **기준값 초과 시 경고 기능**
  - 미세먼지 농도나 CO2 농도가 기준값을 초과하면 알림 출력.
- **음성 피드백 시스템**
  - 음성으로 공기질 상태를 안내.
- **IoT 확장**
  - 데이터를 클라우드에 업로드하여 원격 모니터링 구현.

---

## 📋 참고 자료
- [Grove - Dust Sensor 데이터시트](https://wiki.seeedstudio.com/Grove-Dust_Sensor/)
- [CM1106 데이터시트](https://www.co2meter.com/products/cm1106-co2-sensor)

---
