#pragma once

#include <Adafruit_Sensor.h>
#include <memory> // unique_ptr

/** Holds data that is "vectored", i.e. has three components for three dimensions. */
class VectoredData {
    public:
        /** X-component */
        float x;
        /** Y-Component */
        float y;
        /** Z-Component */
        float z;

        VectoredData()
            : x(-1.0), y(-1.0), z(-1.0) {
        }
        VectoredData(sensors_vec_t& data)
            : x(data.x), y(data.y), z(data.z) {
        }
};

/** 
 * This class holds a snapshot of the relevant
 * data of all sensors of the adafruit featherboard sense.
 * 
 * These are the accumulated values from the several sensor µC.
 */
class SensorsSnapshot {
    private:
    
        constexpr static float TEMP_CALIBRATION_DIFF = 0.0;
        constexpr static float PRESSURE_CALIBRATION_DIFF = 0.0;

        // I got this values by measuring against the resting position.
        // Probably depends on each board.

        constexpr static float GYRO_X_CALIBRATION_DIFF = -0.06;
        constexpr static float GYRO_Y_CALIBRATION_DIFF = 0.12;
        constexpr static float GYRO_Z_CALIBRATION_DIFF = 0.03;

        constexpr static float ACCEL_X_CALIBRATION_DIFF = -0.01;
        constexpr static float ACCEL_Y_CALIBRATION_DIFF = -0.02;
        constexpr static float ACCEL_Z_CALIBRATION_DIFF = -0.29;

        constexpr static float MAGNETIC_X_CALIBRATION_DIFF = 0.0;
        constexpr static float MAGNETIC_Y_CALIBRATION_DIFF = 0.0;
        constexpr static float MAGNETIC_Z_CALIBRATION_DIFF = 0.0;

        /** Calibrated Temperature in °C */
        float temp = 0;
        /** Calibrated Pressure in hPa.  */
        float pressure = 0;
        /** Calibrated Gyro Data in rad/s */
        std::shared_ptr<VectoredData> gyro_rad;
        /** Calibrated Gyro Data in deg/s */
        std::shared_ptr<VectoredData> gyro_degree;
        /** Calibrated Acceleration in m/s^2 */
        std::shared_ptr<VectoredData> accel;
        /** Calibrated Magnetic Data in µT */
        std::shared_ptr<VectoredData> magnetic;

        float radiants_to_degree(float rad) {
            return rad * 180.0 / 3.1415926535941;
        }

    public:
        SensorsSnapshot(
            float& temp, 
            float& pressure, 
            sensors_vec_t& gyro,
            sensors_vec_t& accel,
            sensors_vec_t& magnetic)
            : temp(temp),
              pressure(pressure),
              gyro_rad(std::make_shared<VectoredData>(VectoredData(gyro))),
              gyro_degree(std::make_shared<VectoredData>(VectoredData())),
              accel(std::make_shared<VectoredData>(VectoredData(accel))),
              magnetic(std::make_shared<VectoredData>(VectoredData(accel)))
            {
                this->temp += TEMP_CALIBRATION_DIFF;
                this->pressure += PRESSURE_CALIBRATION_DIFF;
                this->gyro_rad->x += GYRO_X_CALIBRATION_DIFF;
                this->gyro_rad->y += GYRO_Y_CALIBRATION_DIFF;
                this->gyro_rad->z += GYRO_Z_CALIBRATION_DIFF;

                // remove noise by better smoothing the values
                this->gyro_rad->x = round(this->gyro_rad->x * 100.0) / 100.0;
                this->gyro_rad->y = round(this->gyro_rad->y * 100.0) / 100.0;
                this->gyro_rad->z = round(this->gyro_rad->z * 100.0) / 100.0;

                this->gyro_degree->x = this->radiants_to_degree(this->gyro_rad->x);
                this->gyro_degree->y = this->radiants_to_degree(this->gyro_rad->y);
                this->gyro_degree->z = this->radiants_to_degree(this->gyro_rad->z);

                this->accel->x += ACCEL_X_CALIBRATION_DIFF;
                this->accel->y += ACCEL_Y_CALIBRATION_DIFF;
                this->accel->z += ACCEL_Z_CALIBRATION_DIFF;

                // remove noise by better smoothing the values
                this->accel->x = round(this->accel->x * 100.0) / 100.0;
                this->accel->y = round(this->accel->y * 100.0) / 100.0;
                this->accel->z = round(this->accel->z * 100.0) / 100.0;

                this->magnetic->x += MAGNETIC_X_CALIBRATION_DIFF;
                this->magnetic->y += MAGNETIC_Y_CALIBRATION_DIFF;
                this->magnetic->z += MAGNETIC_Z_CALIBRATION_DIFF;
            }

        float get_temp() {
            return this->temp;
        }
        
        float get_pressure() {
            return this->pressure;
        }

        std::shared_ptr<VectoredData> get_gyro_rad() {
            return this->gyro_rad;
        }  

        std::shared_ptr<VectoredData> get_gyro_degree() {
            return this->gyro_degree;
        }    

        std::shared_ptr<VectoredData> get_accel() {
            return this->accel;
        }      

        std::shared_ptr<VectoredData> get_magnetic() {
            return this->magnetic;
        } 
        
};
