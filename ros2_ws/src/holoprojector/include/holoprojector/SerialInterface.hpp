#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

class SerialInterface
{
    public:
        SerialInterface(const char* port, int baudrate);
        ~SerialInterface();
        
        bool open(void);
        bool close(void);
        int readData(uint8_t* buffer, size_t size);
        int writeData(const uint8_t* data, size_t size);
        bool isOpen(void) const;
        size_t available(void);
    private:
        const char* port_;
        int baudrate_;
        int fd_;
        bool is_open_;
};