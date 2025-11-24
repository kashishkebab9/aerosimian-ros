#pragma once

#include <string>
#include <atomic>
#include <thread>
#include <cerrno>
#include <cstring>
#include <chrono>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

/// Simple serial heartbeat driver for Vertiq ESCs.
/// Sends "HB\n" at 5Hz in a background thread.
class VertiqHeartbeat {
public:
  VertiqHeartbeat(const std::string &port = "/dev/ttyUSB0", int baud = 115200)
  : port_(port), baud_(baud) {}

  ~VertiqHeartbeat() {
    stop();
    closePort();
  }

  bool start() {
    if (!openPort()) {
      return false;
    }

    stop_flag_.store(false);
    worker_ = std::thread([this]() { run(); });
    return true;
  }

  void stop() {
    stop_flag_.store(true);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  bool isOpen() const {
    return (fd_ >= 0);
  }
   
  void sendSet(int a, int b, int c, int d) {
    if (fd_ < 0) {
      return;
    }

    char buf[64];
    int n = std::snprintf(buf, sizeof(buf),
                          "SET %d,%d,%d,%d\n",
                          a, b, c, d);
    if (n > 0) {
      ::write(fd_, buf, n);
      tcdrain(fd_);
    }
  }

private:
  std::string port_;
  int baud_;
  int fd_{-1};
  std::atomic<bool> stop_flag_{false};
  std::thread worker_;

  /// Run in background thread
  void run() {
    // Allow ESP to boot after DTR toggle
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    constexpr double HB_PERIOD = 0.2; // 5 Hz
    const char *msg = "HB\n";
    const size_t len = 3;

    while (!stop_flag_.load()) {
      if (fd_ >= 0) {
        ::write(fd_, msg, len);
        tcdrain(fd_);
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(HB_PERIOD));
    }
  }

  /// Map baud to termios flag
  static speed_t baudToFlag(int baud) {
    switch (baud) {
      case 9600:    return B9600;
      case 19200:   return B19200;
      case 38400:   return B38400;
      case 57600:   return B57600;
      case 115200:  return B115200;
      case 230400:  return B230400;
      case 460800:  return B460800;
      case 921600:  return B921600;
      default:      return B115200;
    }
  }

  bool openPort() {
    if (fd_ >= 0) {
      return true; // already open
    }

    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      return false;
    }

    struct termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
      closePort();
      return false;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~(PARENB | PARODD);
    tio.c_cflag &= ~CSTOPB;

    speed_t flag = baudToFlag(baud_);
    cfsetispeed(&tio, flag);
    cfsetospeed(&tio, flag);

    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 2; // deciseconds

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
      closePort();
      return false;
    }

    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  void closePort() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }
};

