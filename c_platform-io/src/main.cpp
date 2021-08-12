#include <Arduino.h>

#include "main.h"
#include "sensors.h"
#include "sensors_snapshot.h"

// Facade around all sensors on the board.
Sensors sensors;

void setup() {
  Serial.begin(9600);
  Serial.println("Adafruit Feather Sense (nRF52840) is alive");
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED_D13, OUTPUT);

  sensors.init();
}



void loop() {
  // put your main code here, to run repeatedly:
  delay(200);
  digitalWrite(LED_BLUE, 1);
  digitalWrite(LED_RED_D13, 0);
  delay(200);
  digitalWrite(LED_BLUE, 0);
  digitalWrite(LED_RED_D13, 1);

  auto data = sensors.getSnapshot();
  Serial.printf(
    "Temp=%05.2f°C, "
    "Pressure=%.2fhPa, " 
    "Gyro(x=%.2f, y=%.2f, z=%.2f deg/s), "
    "Accel(x=%.2f, y=%.2f, z=%.2f m/s^2), "
    "Magnetic(x=%.2f, y=%.2f, z=%.2f µT), "
    "Humidity=%.2f%"
    "\n", 
    data.get_temp(),
    data.get_pressure(),
    data.get_gyro_degree()->x,
    data.get_gyro_degree()->y,
    data.get_gyro_degree()->z,
    data.get_accel()->x,
    data.get_accel()->y,
    data.get_accel()->z,
    data.get_magnetic()->x,
    data.get_magnetic()->y,
    data.get_magnetic()->z,
    data.get_humidity()
  );

}