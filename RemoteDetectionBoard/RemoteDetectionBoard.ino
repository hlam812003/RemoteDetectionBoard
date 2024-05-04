#include <Wire.h>
#include <WiFi.h>
#include <TinyGPS++.h>

#define RXD2 16
#define TXD2 17

HardwareSerial neogps(2);
TinyGPSPlus gps;

const int MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
boolean fall = false;
boolean trigger1 = false;
boolean trigger2 = false;
boolean trigger3 = false;
byte trigger1count = 0;
byte trigger2count = 0;
byte trigger3count = 0;
int angleChange = 0;
int buzzerPin = 5;

const char *ssid = "QUOC HUNG LAU 1"; // SSID của WIFI
const char *pass = "@Lamvanha25011973"; // Password của WiFi

void send_event(const char *event, const char *details);
const char *host = "maker.ifttt.com";
const char *privateKey = "c3IEHJY8SL8qK3zcfznXDt";

void setup() {
  Serial.begin(115200);
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi is connected!!!");

  pinMode(buzzerPin, OUTPUT);
}

void loop() {
    boolean newData = false;

    while (neogps.available()) {
        if (gps.encode(neogps.read())) {
            newData = true;
            displayInfo();
        }
    }

    if (newData) {
        Serial.print("Number of Satellites: ");
        Serial.println(gps.satellites.value());
    }

    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
      Serial.println(F("No GPS detected: check your wires."));
    }

    mpu_read();
    ax = (AcX - 2050) / 16384.00;
    ay = (AcY - 77) / 16384.00;
    az = (AcZ - 1947) / 16384.00;
    gx = (GyX + 270) / 131.07;
    gy = (GyY - 351) / 131.07;
    gz = (GyZ + 136) / 131.07;

    float raw_amplitude = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
    int amplitude = raw_amplitude * 10;
    Serial.println(amplitude);

    if (amplitude <= 3 && trigger2 == false) {
        trigger1 = true;
        Serial.println("TRIGGER 1 ACTIVATED");
    }

    if (trigger1 == true) {
        trigger1count++;
        if (amplitude >= 10) {
            trigger2 = true;
            Serial.println("TRIGGER 2 ACTIVATED");
            trigger1 = false;
            trigger1count = 0;
        }
    }

    if (trigger2 == true) {
        trigger2count++;
        angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
        Serial.println(angleChange);

        if (angleChange >= 50 && angleChange <= 300) {
            trigger3 = true;
            trigger2 = false;
            trigger2count = 0;
            Serial.println(angleChange);
            Serial.println("TRIGGER 3 ACTIVATED");
        }
    }

    if (trigger3 == true) {
        trigger3count++;
        if (trigger3count >= 10) {
            angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
            Serial.println(angleChange);

            if ((angleChange >= 0) && (angleChange <= 15)) {
                fall = true;
                trigger3 = false;
                trigger3count = 0;
                Serial.println("FALL DETECTED");

                char details[500];
                if (gps.location.isValid()) {
                  snprintf(details, 500, "https://www.google.com/maps/search/?api=1&query=%.6f,%.6f",
                                          gps.location.lat(), gps.location.lng());
                } else {
                    snprintf(details, 500, "GPS signal not available");
                }

                send_event("FALL DETECTION", details);

                for (int j = 0; j < 10; j++) {
                    for (int i = 0; i < 80; i++) {
                        digitalWrite(buzzerPin, HIGH);
                        delay(1);
                        digitalWrite(buzzerPin, LOW);
                        delay(1);
                    }

                    for (int i = 0; i < 100; i++) {
                        digitalWrite(buzzerPin, HIGH);
                        delay(2);
                        digitalWrite(buzzerPin, LOW);
                        delay(2);
                    }

                    delay(10); // Nghỉ 10ms
                }

                digitalWrite(buzzerPin, LOW); // Tắt còi
            } else {
                trigger3 = false;
                trigger3count = 0;
                Serial.println("TRIGGER 3 DEACTIVATED");
            }
        }
    }

    if (trigger2count >= 5) {
        trigger2 = false;
        trigger2count = 0;
        Serial.println("TRIGGER 2 DECACTIVATED");
    }

    if (trigger1count >= 5) {
        trigger1 = false;
        trigger1count = 0;
        Serial.println("TRIGGER 1 DECACTIVATED");
    }
    delay(100);
}

void displayInfo() {
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID "));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID "));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID "));
  }

  Serial.println();
}

void mpu_read() {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_addr, (size_t)14, (bool)true); 
    AcX = Wire.read() << 8 | Wire.read();  
    AcY = Wire.read() << 8 | Wire.read();  
    AcZ = Wire.read() << 8 | Wire.read();  
    Tmp = Wire.read() << 8 | Wire.read();  
    GyX = Wire.read() << 8 | Wire.read();  
    GyY = Wire.read() << 8 | Wire.read();  
    GyZ = Wire.read() << 8 | Wire.read();  
}

void send_event(const char *event, const char *details) {
    Serial.print("Connecting to ");
    Serial.println(host);
    WiFiClient client;
    const int httpPort = 80;
    if (!client.connect(host, httpPort)) {
        Serial.println("Connection failed");
        return;
    }
    String url = "/trigger/";
    url += event;
    url += "/with/key/";
    url += privateKey;
    url += "?value1=";
    url += details;
    Serial.print("Requesting URL: ");
    Serial.println(url);
    client.print(String("GET ") + url + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Connection: close\r\n\r\n");
    while (client.connected()) {
        if (client.available()) {
            String line = client.readStringUntil('\r');
            Serial.print(line);
        } else {
            delay(50);
        }
    }
    Serial.println();
    Serial.println("closing connection");
    client.stop();
}