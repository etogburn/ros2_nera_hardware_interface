#ifndef ROS2_NERA_HARDWARE_INTERFACE_IMU_SENSOR_HPP
#define ROS2_NERA_HARDWARE_INTERFACE_IMU_SENSOR_HPP

#include <string>
#include <cmath>

class IMU_Sensor
{
    public:

    std::string name = "";
    std::string accelName[3] = {"", "", ""};
    std::string gyroName[3] = {"", "", ""};
    int16_t rawAccel[3] = {0,0,0};
    int16_t rawGyro[3] = {0,0,0};
    double gyro_raw_to_radps = 0;
    double accel_raw_to_mps = 0;
    double accelmps[3] = {0,0,0};
    double gyroRadps[3] = {0,0,0};
    double mag[4] = {NAN,NAN,NAN,NAN};

    IMU_Sensor() = default;

    IMU_Sensor(const std::string &imu_name, double gyro_max, double accel_max)
    {
      setup(imu_name, gyro_max, accel_max);
    }

    void setup(const std::string &imu_name, double gyro_max, double accel_max)
    {
      name = imu_name;
      accelName[0] = "linear_acceleration.x";
      accelName[1] = "linear_acceleration.y";
      accelName[2] = "linear_acceleration.z";
      gyroName[0] = "angular_velocity.x";
      gyroName[1] = "angular_velocity.y";
      gyroName[2] = "angular_velocity.z";
      accel_raw_to_mps = accel_max;
      gyro_raw_to_radps =gyro_max;
    }

    void calc() {
      calc_gyro();
      calc_accel();
    }
    void calc_gyro()
    {
      for(int i = 0; i < 3; i++) {
        gyroRadps[i] = rawGyro[i] * gyro_raw_to_radps / 32768.0;
      }
    }

    void calc_accel()
    {
      for(int i = 0; i < 3; i++) {
        accelmps[i] = rawAccel[i] * accel_raw_to_mps / 32768.0;
      }
    }

};


#endif // ROS2_NERA_HARDWARE_INTERFACE_WHEEL_HPP
