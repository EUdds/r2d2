#include "SerialInterface.hpp"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

SerialInterface::SerialInterface(const char* port, int baudrate)
    : port_(port), baudrate_(baudrate), fd_(-1), is_open_(false)
{
}

SerialInterface::~SerialInterface()
{
    close();
}

bool SerialInterface::open(void)
{
    fd_ = ::open(port_, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ == -1)
    {
        perror("Unable to open port");
        return false;
    }

    struct termios options;
    tcgetattr(fd_, &options);

    speed_t baud;
    switch (baudrate_)
    {
        case 9600: baud = B9600; break;
        case 19200: baud = B19200; break;
        case 38400: baud = B38400; break;
        case 57600: baud = B57600; break;
        case 115200: baud = B115200; break;
        default:
            fprintf(stderr, "Unsupported baud rate\n");
            ::close(fd_);
            fd_ = -1;
            return false;
    }

    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcsetattr(fd_, TCSANOW, &options);

    is_open_ = true;
    return true;
}

bool SerialInterface::close(void)
{
    if (is_open_)
    {
        ::close(fd_);
        fd_ = -1;
        is_open_ = false;
    }
    return true;
}

int SerialInterface::readData(uint8_t* buffer, size_t size)
{
    if (!is_open_)
    {
        fprintf(stderr, "Port not open\n");
        return -1;
    }
    return ::read(fd_, buffer, size);
}

int SerialInterface::writeData(const uint8_t* data, size_t size)
{
    if (!is_open_)
    {
        fprintf(stderr, "Port not open\n");
        return -1;
    }
    return ::write(fd_, data, size);
}

bool SerialInterface::isOpen(void) const
{
    return is_open_;
}

size_t SerialInterface::available(void)
{
    if (!is_open_)
    {
        fprintf(stderr, "Port not open\n");
        return 0;
    }
    int bytes_available;
    ioctl(fd_, FIONREAD, &bytes_available);
    return static_cast<size_t>(bytes_available);
}