#ifndef ROS2_NERA_HARDWARE_INTERFACE_ARDUINO_COMMS_HPP
#define ROS2_NERA_HARDWARE_INTERFACE_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
#include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>

//commands imported from hardware interface
#define COMMAND_OK 0x00FF
#define COMMAND_INVALID 0x00FE
#define COMMAND_NOACTION 0x00FD
#define COMMAND_READY 0x0001
#define COMMAND_WAKE 0x0000
#define COMMAND_SHUTDOWN 0x0002
#define COMMAND_MOTORENABLE 0x0100
#define COMMAND_SETMOTORSPEED 0x0101
#define COMMAND_SETLEFTSPEED 0x0103
#define COMMAND_SETRIGHTSPEED 0x0104
#define COMMAND_MOTORSTOP 0x0102
#define COMMAND_GETMOTORSPEED 0x0180
#define COMMAND_GETMOTORPOSITION 0x0181
#define COMMAND_GETACCELVALS 0x0201
#define COMMAND_GETGYROVALS 0x0210

#define COMMAND_START_BYTE 0xAA
#define COMMAND_MIN_SIZE 5

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

struct Packet {
  uint16_t command;
  uint8_t dataLength; //data length only in packet to be sent
  uint8_t responseLength; //total length of data to be recieved
  std::vector<uint8_t> data;
  bool valid;

  Packet() : data(8) {}
};

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

  void send_wake_msg()
  {
    Packet packet;
    packet.command = COMMAND_READY;
    packet.dataLength = 0;
    packet.responseLength = 0;
    packet.valid = true;
    
    send_packet(packet);
  }

  void get_encoder_values(int16_t &val_1, int16_t &val_2)
  {
    Packet packet;
    packet.command = COMMAND_GETMOTORPOSITION;
    packet.dataLength = 0;
    packet.responseLength = 4;
    packet.valid = true;

    Packet response = send_packet(packet);

    if(response.dataLength < 4) return;

    val_1 = (response.data[0] << 8) | response.data[1];
    val_2 = (response.data[2] << 8) | response.data[3];
  }

  void get_speed_values(int16_t &val_1, int16_t &val_2)
  {
    Packet packet;
    packet.command = COMMAND_GETMOTORSPEED;
    packet.dataLength = 0;
    packet.responseLength = 4;
    packet.valid = true;

    Packet response = send_packet(packet);

    if(response.dataLength < 4) return;

    val_1 = (response.data[0] << 8) | response.data[1];
    val_2 = (response.data[2] << 8) | response.data[3];
  }

  void get_accel_values(int16_t &val_1, int16_t &val_2, int16_t &val_3)
  {
    Packet packet;
    packet.command = COMMAND_GETACCELVALS;
    packet.dataLength = 0;
    packet.responseLength = 6;
    packet.valid = true;

    Packet response = send_packet(packet);

    if(response.dataLength < 4) return;

    val_1 = (response.data[0] << 8) | response.data[1];
    val_2 = (response.data[2] << 8) | response.data[3];
    val_3 = (response.data[4] << 8) | response.data[5];
  }

  void get_gyro_values(int16_t &val_1, int16_t &val_2, int16_t &val_3)
  {
    Packet packet;
    packet.command = COMMAND_GETGYROVALS;
    packet.dataLength = 0;
    packet.responseLength = 6;
    packet.valid = true;

    Packet response = send_packet(packet);

    if(response.dataLength < 4) return;

    val_1 = (response.data[0] << 8) | response.data[1];
    val_2 = (response.data[2] << 8) | response.data[3];
    val_3 = (response.data[4] << 8) | response.data[5];
  }
  

  void set_motor_values(int16_t val_1, int16_t val_2)
  {
    Packet packet;
    packet.command = COMMAND_SETMOTORSPEED;
    packet.dataLength = 4;
    packet.responseLength = 0;
    packet.valid = true;

    packet.data[0] = (val_1 >> 8);
    packet.data[1] = (val_1 & 0xFF);
    packet.data[2] = (val_2 >> 8);
    packet.data[3] = (val_2 & 0xFF);

    send_packet(packet);

  }

  Packet send_packet(const Packet &packet, bool print_output = false)
  {
    const uint8_t bufLen = packet.dataLength + COMMAND_MIN_SIZE;
    std::vector<uint8_t> buffer(bufLen);
    // Start byte
    buffer[0] = 0xAA;

    // Data length byte
    buffer[1] = packet.dataLength;

    // Command bytes
    buffer[2] = packet.command >> 8;
    buffer[3] = packet.command & 0xFF;

    for (uint8_t i = 4; i < bufLen - 1; ++i) {
        buffer[i] = packet.data[i-4];
    }

    uint8_t checksum = 0x00;
    for (uint8_t i = 0; i < bufLen - 1; ++i) {
        checksum ^= buffer[i];
    }
    buffer[bufLen-1] = checksum;

    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(buffer);

    int bufSize = packet.responseLength + COMMAND_MIN_SIZE;
    Packet response;
    std::vector<uint8_t> responseBuf(bufSize);
    try
    {
        serial_conn_.Read(responseBuf, bufSize, timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    if (print_output)
    {
     // std::cout << "Sent: " << std::hex << msg_to_send << " Recv: " << response << std::endl;
      std::cout << "Generated Packet: ";
      for (uint8_t byte : buffer) {
          std::cout << "0x" << std::hex << static_cast<int>(byte) << " ";
      }
      std::cout << std::endl;

      std::cout << "Recieved Packet: ";
      for (uint8_t byte : responseBuf) {
          std::cout << "0x" << std::hex << static_cast<int>(byte) << " ";
      }
      std::cout << std::endl;
    
    }

    response.dataLength = responseBuf[1];
    response.command = (responseBuf[2] << 8) | responseBuf[3];

    if(response.dataLength > 0) {
      for (uint8_t i = 0; i < response.dataLength; ++i) {
        response.data[i] = responseBuf[i+4];
      }
    }

    checksum = 0x00;

    for (uint8_t i = 0; i < bufSize - 1; ++i) {
        checksum ^= responseBuf[i];
    }
    
    if(checksum == responseBuf[bufSize - 1]) response.valid = true;
    else response.valid = false;

    return response;
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;

};

#endif // ROS2_NERA_HARDWARE_INTERFACE_ARDUINO_COMMS_HPP