#include <Arduino.h>
// Sensor for tempetature and pressure
#include <Adafruit_BMP280.h>
 /* I2C */
#include <Wire.h>

// Sensor for tempetature and pressure
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

const int other_led = 13;

void setup() {
  Serial.begin(9600);
  Serial.println("hello");
  pinMode(LED_BLUE, OUTPUT);
  pinMode(other_led, OUTPUT);

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


}

void loop() {
  // put your main code here, to run repeatedly:
  delay(500);
  digitalWrite(LED_BLUE, 1);
  digitalWrite(other_led, 0);
  delay(500);
  digitalWrite(LED_BLUE, 0);
  digitalWrite(other_led, 1);
  Serial.println("hello");
  bmp_temp->printSensorDetails();

  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  Serial.print(F("Temperature = "));
  Serial.print(temp_event.temperature);
  Serial.println(" *C");
  Serial.print(F("Pressure = "));
  Serial.print(pressure_event.pressure);
  Serial.println(" hPa");
}