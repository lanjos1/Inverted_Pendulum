#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Objetos para os sensores
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// Pinos XSHUT (reset) para cada sensor
#define LOX1_XSHUT_PIN 26
#define LOX2_XSHUT_PIN 27

void setup() {
  Serial.begin(115200);
  
  // Aguarda a serial (opcional para placas com USB nativo)
  while (!Serial) {
    delay(10);
  }

  Serial.println("\nIniciando dois sensores VL53L0X");

  // Configura pinos XSHUT
  pinMode(LOX1_XSHUT_PIN, OUTPUT);
  pinMode(LOX2_XSHUT_PIN, OUTPUT);
  
  // Inicialmente desliga ambos os sensores
  digitalWrite(LOX1_XSHUT_PIN, LOW);
  digitalWrite(LOX2_XSHUT_PIN, LOW);
  delay(10);

  // Ativa e configura o Sensor 1
  digitalWrite(LOX1_XSHUT_PIN, HIGH);
  delay(10);
  
  if (!lox1.begin(0x30)) {  // Endereço I2C 0x30
    Serial.println(F("Falha no Sensor 1 (VL53L0X)"));
    while (1);
  }
  Serial.println(F("Sensor 1 OK"));

  // Ativa e configura o Sensor 2
  digitalWrite(LOX2_XSHUT_PIN, HIGH);
  delay(10);
  
  if (!lox2.begin(0x31)) {  // Endereço I2C 0x31
    Serial.println(F("Falha no Sensor 2 (VL53L0X)"));
    while (1);
  }
  Serial.println(F("Sensor 2 OK"));

  // Inicia medição contínua
  lox1.startRangeContinuous();
  lox2.startRangeContinuous();

  Serial.println("\nPronto para medir...");
  Serial.println("------------------------");
}

void loop() {
  VL53L0X_RangingMeasurementData_t medida1;
  VL53L0X_RangingMeasurementData_t medida2;

  // Lê Sensor 1
  lox1.rangingTest(&medida1, false); // 'false' desativa debug no serial
  
  Serial.print("Sensor 1: ");
  if (medida1.RangeStatus != 4) {  // Status 4 = fora de alcance
    Serial.print(medida1.RangeMilliMeter);
    Serial.print("mm");
  } else {
    Serial.print("fora de alcance");
  }

  // Lê Sensor 2
  lox2.rangingTest(&medida2, false);
  
  Serial.print("  |  Sensor 2: ");
  if (medida2.RangeStatus != 4) {
    Serial.print(medida2.RangeMilliMeter);
    Serial.print("mm");
  } else {
    Serial.print("fora de alcance");
  }

  Serial.println(); // Nova linha
  delay(100); // Intervalo entre leituras
}
/*
**Conexões**

| VL53L0X | ESP32                                   |
| ------- | --------------------------------------- |
| VCC     | 3.3V                                    |
| GND     | GND                                     |
| SDA     | GPIO 21                                 |
| SCL     | GPIO 22                                 |
| XSHUT   | GPIO 26 / GPIO 27 (Sensor 1 / Sensor 2) |
*/