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
    "Pressure=%07.2fhPa, " 
    "Gyro(x=%.2f, y=%.2f, z=%.2f deg/s), "
    "Accel(x=%.2f, y=%.2f, z=%.2f m/s^2)"
    "\n", 
    data.get_temp(),
    data.get_pressure(),
    data.get_gyro_degree()->x,
    data.get_gyro_degree()->y,
    data.get_gyro_degree()->z,
    data.get_accel()->x,
    data.get_accel()->y,
    data.get_accel()->z
  );
  
  // bmp_temp->printSensorDetails();

  /*sensors_event_t temp_event, pressure_event, accel_event, gyro_event;
  sensor_bmp_temp->getEvent(&temp_event);
  sensor_bmp_pressure->getEvent(&pressure_event);
  sensor_lsm_accel->getEvent(&accel_event);
  sensor_lsm_gyro->getEvent(&gyro_event);
  /*Serial.print(F("T = "));
  Serial.print(temp_event.temperature);
  Serial.print(" *C, ");
  Serial.print(F("Pressure = "));
  Serial.print(pressure_event.pressure);
  Serial.print(" hPa, ");*/

  /*Serial.print("Accel X: ");
  Serial.print(accel_event.acceleration.x);
  Serial.print(" Y: ");
  Serial.print(accel_event.acceleration.y);
  Serial.print(" Z: ");
  Serial.print(accel_event.acceleration.z);
  Serial.println(" m/s^2, ");*/

  /*Serial.print("Gyro X: ");
  Serial.print(gyro_event.gyro.x);
  Serial.print(" Y: ");
  Serial.print(gyro_event.gyro.y);
  Serial.print(" Z: ");
  Serial.print(gyro_event.gyro.z);
  Serial.println(" radians/s");
  Serial.println();

  /*sensors_event_t accel;
  sensors_event_t gyro;
  // sensors_event_t temp;
  // lsm_temp->getEvent(&temp);
  lsm_accel->getEvent(&accel);
  lsm_gyro->getEvent(&gyro);*/



  /*// Display the results (acceleration is measured in m/s^2) 
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  // Display the results (rotation is measured in rad/s) 
  Serial.print("\t\tGyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");
  Serial.println();*/
}