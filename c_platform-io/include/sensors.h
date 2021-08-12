// Sensor for tempetature and pressure
#include <Adafruit_BMP280.h>
// I2C
#include <Wire.h>
// Sensor for Gyro + Accel
#include <Adafruit_LSM6DS33.h> 
// Magnetic field
#include <Adafruit_LIS3MDL.h>
// Humidity
#include <Adafruit_SHT31.h>

#include "main.h"
#include "sensors_snapshot.h"

class Sensors {
    private:
        // Sensor for tempetature and pressure via I2C interface
        Adafruit_BMP280 bmp280; 
        // Sensor for acceleration and gyroscope via I2C interface
        // (I2C address 0x77)
        Adafruit_LSM6DS33 lsm6ds33;
        // Sensor for magnetic field via I2C interface
        // (I2C address 0x1C)
        Adafruit_LIS3MDL lis3mdl;
        // Sensor for humidity via I2C interface
        // (I2C address 0x44)
        Adafruit_SHT31 sht31;

        // Sensors on BMP280
        Adafruit_Sensor *sensor_bmp_temp, *sensor_bmp_pressure;
        // Sensors on Adafruit_LSM6DS33 (temperature is available for internel gyroscope calibrating, we don't need it)
        Adafruit_Sensor /*sensor_lsm_temp,*/ *sensor_lsm_accel, *sensor_lsm_gyro;
        // Sensors on LIS3MDL
        // LIS3MDL doesn't use the Adafruit_Sensor abstraction
        // sensor_t * sensor_lis_magnet;
        // Sensors on SHT31
        // SHT31 doesn't use the Adafruit_Sensor abstraction
        // Adafruit_Sensor * sensor_sht31_humidity;

        void setup_sensors_bmp280() {
            if (!bmp280.begin()) {
                Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                                "try a different address!"));
                abort_error();
            }

            /* Default settings from datasheet. */
            bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                            Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                            Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                            Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                            Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

            sensor_bmp_pressure = this->bmp280.getPressureSensor();  
            sensor_bmp_temp = this->bmp280.getTemperatureSensor();  
            Serial.println("BMP280 Found!");
        }

        void setup_sensors_lsm6ds33() {
            if (!lsm6ds33.begin_I2C()) {
                Serial.println("Failed to find LSM6DS33 chip");
                abort_error();
            }
            sensor_lsm_gyro = this->lsm6ds33.getGyroSensor();
            sensor_lsm_accel = this->lsm6ds33.getAccelerometerSensor();
            Serial.println("LSM6DS33 Found!");
        }

        void setup_sensors_lis3mdl() {
            if (!lis3mdl.begin_I2C()) {
                Serial.println("Failed to find LIS3MDL chip");
                abort_error();
            }
            Serial.println("IS3MDL Found!");

            lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
            lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
            lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
            lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
        }

        void setup_sensors_sht31() {
            if (!sht31.begin(0x44)) {
                Serial.println("SHT31 not found");
                abort_error();
            }
            Serial.print("Heater Enabled State: ");
            if (sht31.isHeaterEnabled()) {
                Serial.println("ENABLED");
            }
            else {
                Serial.println("DISABLED");
            }
            Serial.println("SHT31 found");
        }

        /** HALTS the prgoramm and light up both LEDs */
        void abort_error() {
            Serial.println("Aborting, error");
            digitalWrite(LED_BLUE, 1);
            digitalWrite(LED_RED_D13, 1);
            while (1) delay(10);
        }

    public:
        Sensors() {
        }

        void init() {
            setup_sensors_bmp280();
            setup_sensors_lsm6ds33();
            setup_sensors_lis3mdl();
            setup_sensors_sht31();

            sensor_bmp_temp->printSensorDetails();
            sensor_bmp_pressure->printSensorDetails();
            sensor_lsm_accel->printSensorDetails();
            sensor_lsm_accel->printSensorDetails();
        }

        SensorsSnapshot getSnapshot() {
            sensors_event_t temp_event,
                pressure_event,
                accel_event,
                gyro_event,
                magnetic_event;
            sensor_bmp_temp->getEvent(&temp_event);
            sensor_bmp_pressure->getEvent(&pressure_event);
            sensor_lsm_gyro->getEvent(&gyro_event);
            sensor_lsm_accel->getEvent(&accel_event);
            lis3mdl.getEvent(&magnetic_event);
            float humidity = sht31.readHumidity();
            return SensorsSnapshot(
                temp_event.temperature,
                pressure_event.pressure,
                gyro_event.gyro,
                accel_event.acceleration,
                magnetic_event.magnetic,
                humidity
            );
        }
};
