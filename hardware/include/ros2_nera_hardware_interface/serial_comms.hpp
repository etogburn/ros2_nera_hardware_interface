#ifndef ROS2_NERA_HARDWARE_INTERFACE_ARDUINO_COMMS_HPP
#define ROS2_NERA_HARDWARE_INTERFACE_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
#include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    case 460800: return LibSerial::BaudRate::BAUD_460800;
    case 921600: return LibSerial::BaudRate::BAUD_921600;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class SerialComms
{

public:

  SerialComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }


  std::string send_msg(const std::string &msg_to_send, bool print_output = true)
  {
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);

    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
   //   serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    if (print_output)
    {
      std::cout << "Sent: " << std::hex << msg_to_send << " Recv: " << response << std::endl;
    }

    return response;
  }

  void send_empty_msg()
  {
    std::string response = send_msg("\r");
  }

  void read_encoder_values(int &val_1, int &val_2)
  {
    std::string response = send_msg("e\r");

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = response.substr(del_pos + delimiter.length());

    val_1 = std::atoi(token_1.c_str());
    val_2 = std::atoi(token_2.c_str());
  }
  // void set_motor_values(int val_1, int val_2)
  // {
  //   std::vector<unsigned char> leftMotor[7];
  //   std::vector<unsigned char> rightMotor[7];

  //   leftMotor[0] = 0xAA;
  //   leftMotor[1] = 0x02;
  //   leftMotor[2] = 0x01;
  //   leftMotor[3] = 0x01;

  //   send_packet(*leftMotor);
  //   send_packet(*rightMotor);
  // }

  // void set_motor_values(int val_1, int val_2)
  // {
  //   std::stringstream ss;
  //   ss << "m " << val_1 << " " << val_2 << "\r";
  //   send_msg(ss.str());
  // }

  void set_motor_values(int16_t val_1, int16_t val_2)
  {
    std::stringstream ss;

    uint8_t tempBuf[7];

    tempBuf[0] = 0xAA;
    tempBuf[1] = 0x02;
    tempBuf[2] = 0x01;
    tempBuf[3] = 0x01; //left motor
    tempBuf[4] = (val_1 & 0xFF00) >> 8;
    tempBuf[5] = val_1 & 0x00FF;
    tempBuf[6] = tempBuf[0] ^ tempBuf[1] ^ tempBuf[2] ^ tempBuf[3] ^ tempBuf[4] ^ tempBuf[5];

    for(uint8_t i = 0; i < 7; i++) {
      ss << tempBuf[i];
    }

    send_msg(ss.str()); //left

    tempBuf[3] = 0x02; //right motor
    tempBuf[4] = (val_2& 0xFF00) >> 8;
    tempBuf[5] = val_2 & 0x00FF;
    tempBuf[6] = tempBuf[0] ^ tempBuf[1] ^ tempBuf[2] ^ tempBuf[3] ^ tempBuf[4] ^ tempBuf[5];

    ss.str("");

    for(uint8_t i = 0; i < 7; i++) {
      ss << tempBuf[i];
    }

    send_msg(ss.str());
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // ROS2_NERA_HARDWARE_INTERFACE_ARDUINO_COMMS_HPP