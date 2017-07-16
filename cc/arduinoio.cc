#include "arduinoio.h"

#include <boost/bind.hpp>

namespace {

struct AsBytes {
  AsBytes(const std::string& s) : s(s) {}
  const std::string& s;
};

std::ostream& operator<<(std::ostream& os, const AsBytes& b) {
  for (char c : b.s) {
    if (isprint(c))
      os << c;
    else
      os << '\\' << uint16_t(uint8_t(c));
  }
  return os << std::endl;
}

std::string Encode(const std::string &command) {
  const char address = 0;
  const char addr_size = 1;
  const char timeout = 0;
  std::ostringstream buf;
  buf << addr_size << char(command.size()) << timeout << address << command;
  // Append fletcher16 checksum.
  char chk1 = 0;
  char chk2 = 0;
  for (char c : buf.str()) {
    chk1 += c;
    chk2 += chk1;
  }
  buf << chk2 << chk1;
  std::cout << AsBytes(buf.str());
  return buf.str();
}

class blocking_reader {
public:
  // Constructs a blocking reader, pass in an open serial_port and
  // a timeout in milliseconds.
  blocking_reader(boost::asio::serial_port &port,
                  boost::posix_time::time_duration timeout)
      : port(port), timeout(timeout), timer(port.get_io_service()),
        read_error(true) {}

  // Reads a character or times out
  // returns false if the read times out
  bool read_char(char &val) {
    val = c = '\0';
    port.get_io_service().reset();
    boost::asio::async_read(
        port, boost::asio::buffer(&c, 1),
        boost::bind(&blocking_reader::read_complete, this,
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
    // Setup a deadline time to implement our timeout.
    timer.expires_from_now(timeout);
    timer.async_wait(boost::bind(&blocking_reader::time_out, this,
                                 boost::asio::placeholders::error));
    port.get_io_service().run();
    if (!read_error) val = c;
    return !read_error;
  }

private:
  boost::asio::serial_port &port;
  boost::posix_time::time_duration timeout;
  char c;
  boost::asio::deadline_timer timer;
  bool read_error;

  // Called when an async read completes or has been cancelled
  void read_complete(const boost::system::error_code &error,
                     size_t bytes_transferred) {
    read_error = (error || bytes_transferred == 0);
    timer.cancel();
  }

  void time_out(const boost::system::error_code &error) {
    if (error) return;  // timer was cancelled.
    port.cancel();
  }
};

enum class State : char {
  READY = 'R',
  ERROR = 'E',
  UNKNOWN = 0,
};

State GetState(boost::asio::serial_port &port) {
  char c;
  blocking_reader reader(port, boost::posix_time::seconds(1));
  if (!reader.read_char(c))
    return State::UNKNOWN;
  return c == char(State::READY) ? State::READY : State::ERROR;
}

void WaitReadyForever(boost::asio::serial_port& port) {
  while (GetState(port) != State::READY);
}

} // namespace

void ArduinoIO::SendMessage(std::string command) {
  command += "\0";
  command = Encode(command);
  for (int retry = 0; retry < 1000; ++retry) {
    //WaitReadyForever(serial_port_);
    if (retry > 0)
      std::cerr << "retry " << retry << " for msg " << AsBytes(command);
    boost::asio::write(serial_port_, boost::asio::buffer(command));
    if (GetState(serial_port_) == State::READY)
      break;
  }
}

void ArduinoIO::WriteOutput(Pin pin, bool value) {
  std::string command = "SET_IO";
  command.insert(command.end(), pin);
  command.insert(command.end(), value? 1 : 0);
  SendMessage(command);
}
