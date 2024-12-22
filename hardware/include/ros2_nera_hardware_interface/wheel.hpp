#ifndef ROS2_NERA_HARDWARE_INTERFACE_WHEEL_HPP
#define ROS2_NERA_HARDWARE_INTERFACE_WHEEL_HPP

#include <string>
#include <cmath>

class Wheel
{
    public:

    std::string name = "";
    int16_t enc = 0;
    int16_t lastEnc = 0;
    int encoder = 0;
    double cmd = 0;
    int16_t speed = 0;
    double pos = 0;
    double lastPos = 0;
    double vel = 0;
    double rads_per_count = 0;

    Wheel() = default;

    Wheel(const std::string &wheel_name, int counts_per_rev)
    {
      setup(wheel_name, counts_per_rev);
    }

    
    void setup(const std::string &wheel_name, int counts_per_rev)
    {
      name = wheel_name;
      rads_per_count = (2*M_PI)/counts_per_rev;
    }

    double calc_enc_angle()
    {
      int16_t encChange =0;
      encChange = enc - lastEnc;

      encoder += encChange;
      lastEnc = enc;
      return encoder * rads_per_count;
    }

    double calc_speed()
    {
      return speed * rads_per_count / 60;
    }

    int16_t get_wheel_rpm() 
    {
      return cmd * 60 * 100/(2 * M_PI);
    }


};


#endif // ROS2_NERA_HARDWARE_INTERFACE_WHEEL_HPP
