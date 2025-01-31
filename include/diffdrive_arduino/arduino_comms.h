#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H

#include <asio.hpp>  // Подключите библиотеку asio для работы с UDP

class ArduinoComms
{
public:
  ArduinoComms()
      : socket_(io_context_), endpoint_(asio::ip::udp::v4(), 0)
  { }

  ArduinoComms(const std::string &host, int port)
      : socket_(io_context_), endpoint_(asio::ip::address::from_string(host), port)
  { }

  void setup(const std::string &host, int port);
  void sendEmptyMsg();
  void readEncoderValues(int &val_1, int &val_2);
  void setMotorValues(float val_1, float val_2);
  void setPidValues(float k_p, float k_d, float k_i, float k_o);

  bool connected() const { return socket_.is_open(); }
  

private:
  asio::io_context io_context_;
  asio::ip::udp::socket socket_;
  asio::ip::udp::endpoint endpoint_;
  
  void sendMsg(const std::string &msg_to_send);
  std::string receiveMsg();

  static constexpr size_t max_length = 1024;
  char recv_buffer_[max_length];
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
