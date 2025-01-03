#ifndef ROS2_NERA_HARDWARE_INTERFACE_IMU_SENSOR_HPP
#define ROS2_NERA_HARDWARE_INTERFACE_IMU_SENSOR_HPP

#include <string>
#include <cmath>

#define X_IDX 2
#define Y_IDX 1
#define Z_IDX 0

#define INVX 1
#define INVY 0
#define INVZ 0

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
      accelName[X_IDX] = "linear_acceleration.x";
      accelName[Y_IDX] = "linear_acceleration.y";
      accelName[Z_IDX] = "linear_acceleration.z";
      gyroName[X_IDX] = "angular_velocity.x";
      gyroName[Y_IDX] = "angular_velocity.y";
      gyroName[Z_IDX] = "angular_velocity.z";
      accel_raw_to_mps = accel_max;
      gyro_raw_to_radps =gyro_max;
    }

    void calc() {
      calc_gyro();
      calc_accel();
    }
    void calc_gyro()
    {
      int invertedMult = 1;
      for(int i = 0; i < 3; i++) {
        if(isInverted(i)) invertedMult = -1;
        else invertedMult = 1;
        gyroRadps[i] = rawGyro[i] * gyro_raw_to_radps / 32768.0 * invertedMult;
      }
    }

    void calc_accel()
    {
      int invertedMult = 1;
      for(int i = 0; i < 3; i++) {
        if(isInverted(i)) invertedMult = -1;
        else invertedMult = 1;
        accelmps[i] = rawAccel[i] * accel_raw_to_mps / 32768.0 * invertedMult;
      }
    }

    bool isInverted(int axis) {{
      switch (axis) {
        case X_IDX:
          return INVX;
          break;
        case Y_IDX:
          return INVY;
          break;
        case Z_IDX:
          return INVZ;
          break;
      }

      return 0;

    }}

};


#endif // ROS2_NERA_HARDWARE_INTERFACE_WHEEL_HPP
