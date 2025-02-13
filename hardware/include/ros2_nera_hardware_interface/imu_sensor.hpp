#ifndef ROS2_NERA_HARDWARE_INTERFACE_IMU_SENSOR_HPP
#define ROS2_NERA_HARDWARE_INTERFACE_IMU_SENSOR_HPP

#include <string>
#include <cmath>

#define X_IDX 0
#define Y_IDX 1
#define Z_IDX 2

#define INVX 0
#define INVY 0
#define INVZ 0

#define ACCEL_IDX 4
#define GYRO_IDX 7

class IMU_Sensor
{
    public:

    std::string name = "";
    std::string imuStateInterface[10];
    double imuStateVals[10];
    int16_t rawAccel[3] = {0,0,0};
    int16_t rawGyro[3] = {0,0,0};
    double gyro_raw_to_radps = 0;
    double accel_raw_to_mps = 0;
    double gyroOffset[3] = {0,0,0};
    double accelmps[3] = {0,0,0};
    double gyroRadps[3] = {0,0,0};
    double mag[4] = {NAN,NAN,NAN,NAN};
    int cal_vals_needed = 0;

    IMU_Sensor() = default;

    IMU_Sensor(const std::string &imu_name, double gyro_max, double accel_max, int loop_rate)
    {
      setup(imu_name, gyro_max, accel_max, loop_rate);
    }

    void setup(const std::string &imu_name, double gyro_max, double accel_max, int loop_rate)
    {
      name = imu_name;
      imuStateInterface[0] = "orientation.x";
      imuStateInterface[1] = "orientation.y";
      imuStateInterface[2] = "orientation.z";
      imuStateInterface[3] = "orientation.w";
      imuStateInterface[4] = "linear_acceleration.x";
      imuStateInterface[5] = "linear_acceleration.z";
      imuStateInterface[6] = "linear_acceleration.y";
      imuStateInterface[7] = "angular_velocity.x";
      imuStateInterface[8] = "angular_velocity.y";
      imuStateInterface[9] = "angular_velocity.z";

      for(int i = 0; i < 4; i++) {
        imuStateVals[i] = mag[i];
      }
      accel_raw_to_mps = accel_max;
      gyro_raw_to_radps = gyro_max;
      cal_vals_needed = loop_rate * 2;

      calc();
    }

    void calc() {
      calc_gyro();
      calc_accel();

      imuStateVals[ACCEL_IDX + 0] = accelmps[X_IDX];
      imuStateVals[ACCEL_IDX + 1] = accelmps[Y_IDX];
      imuStateVals[ACCEL_IDX + 2] = accelmps[Z_IDX];
      imuStateVals[GYRO_IDX + 0] = gyroRadps[X_IDX];
      imuStateVals[GYRO_IDX + 1] = gyroRadps[Y_IDX];
      imuStateVals[GYRO_IDX + 2] = gyroRadps[Z_IDX];
    }

    void calc_gyro()
    {
      for(int i = 0; i < 3; i++) {
        gyroRadps[i] = (rawGyro[i] * gyro_raw_to_radps / 32768.0) - gyroOffset[i];
      }
    }

    void calc_accel()
    {
      for(int i = 0; i < 3; i++) {
        accelmps[i] = rawAccel[i] * accel_raw_to_mps / 32768.0;
      }
    }

    bool calibrate() {
      static int loop_counter = 0;
      static double tempVals[3] = {0,0,0};
      calc();

      loop_counter++;

      for(int i = 0; i < 3; i++) {
        tempVals[i] = tempVals[i] + gyroRadps[i]; 
        //if(i==0) std::cout << tempVals[i] << std::endl;
      }

      if(loop_counter > cal_vals_needed) {
        gyroOffset[0] = tempVals[0]/loop_counter;
        gyroOffset[1] = tempVals[1]/loop_counter;
        gyroOffset[2] = tempVals[2]/loop_counter;
        return true;
      }
      return false;
    }

};


#endif // ROS2_NERA_HARDWARE_INTERFACE_WHEEL_HPP
