#include "Serial.h"
#include "log.h"
#include <termios.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

Serial::Serial(const char *deviceName) :
    m_deviceName(deviceName),
    m_openFlags(O_RDWR | O_NDELAY)
{

}

Serial::Serial(const char *deviceName, int openFlags) :
    m_deviceName(deviceName),
    m_openFlags(openFlags)
{

}

Serial::~Serial()
{
    Destroy();
}

int Serial::Init() {
    m_fd = open(m_deviceName.c_str(), m_openFlags);
    if (m_fd < 0) {
        LOG(ERROR, "con't open device %s", m_deviceName.c_str());
        return m_fd;
    }

    // set serial to NBLOCK mode
    int flags = 0;
    flags = fcntl(m_fd, F_GETFL, 0);
    flags &= ~O_NONBLOCK;

    if (fcntl(m_fd, F_SETFL, flags) < 0) {
        LOG(ERROR, "fcntl failed.");
        return -1;
    }

    if (isatty(m_fd) == 0) {
        LOG(ERROR, "no tty device");
        close(m_fd);
        return -1;
    } else {
        LOG(INFO, "tty device %s init success", m_deviceName.c_str());
    }
    return 0;
}


bool Serial::IsReady() {
    return (m_fd != -1);
}

int Serial::Setopt(SerialOpt_t *serialOpt)
{
    struct termios newtio;
    struct termios oldtio;

    bzero(&newtio, sizeof(newtio));
    bzero(&oldtio, sizeof(oldtio));

    if (tcgetattr(m_fd, &oldtio) != 0) {
        LOG(ERROR, "tcgetattr error");
        return -1;
    }

    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch (serialOpt->speed) {
    case 1200:
        cfsetspeed(&newtio, B1200);
        cfsetispeed(&newtio, B1200);
        cfsetospeed(&newtio, B1200);
        break;
    case 2400:
        cfsetspeed(&newtio, B2400);
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetspeed(&newtio, B4800);
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetspeed(&newtio, B9600);
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 19200:
        cfsetspeed(&newtio, B19200);
        cfsetispeed(&newtio, B19200);
        cfsetospeed(&newtio, B19200);
        break;
    case 38400:
        cfsetspeed(&newtio, B38400);
        cfsetispeed(&newtio, B38400);
        cfsetospeed(&newtio, B38400);
        break;
    case 57600:
        cfsetspeed(&newtio, B57600);
        cfsetispeed(&newtio, B57600);
        cfsetospeed(&newtio, B57600);
        break;
    case 115200:
        cfsetspeed(&newtio, B115200);
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 230400:
        cfsetspeed(&newtio, B230400);
        cfsetispeed(&newtio, B230400);
        cfsetospeed(&newtio, B230400);
        break;
    case 460800:
        cfsetspeed(&newtio, B460800);
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    case 921600:
        cfsetspeed(&newtio, B921600);
        cfsetispeed(&newtio, B921600);
        cfsetospeed(&newtio, B921600);
        break;
    default:
        break;
    }

    switch (serialOpt->dataBits) {
        case 5:
            newtio.c_cflag |= CS5;
            break;
        case 6:
            newtio.c_cflag |= CS6;
            break;
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
        default:
            LOG(ERROR, "unsupported data size");
            return -1;
    }

    switch (serialOpt->parity) {
        case 'n':
        case 'N':
            newtio.c_cflag &= ~PARENB;    /* Clear parity enable */
            newtio.c_iflag &= ~INPCK;     /* Disable input parity check */
            break;
        case 'o':
        case 'O':
            newtio.c_cflag |= (PARODD | PARENB); /* Odd parity instead of even */
            newtio.c_iflag |= INPCK;     /* Enable input parity check */
            break;
        case 'e':
        case 'E':
            newtio.c_cflag |= PARENB;    /* Enable parity */
            newtio.c_cflag &= ~PARODD;   /* Even parity instead of odd */
            newtio.c_iflag |= INPCK;     /* Enable input parity check */
            break;
        case 'm':
        case 'M':
            newtio.c_cflag |= PARENB;    /* Enable parity */
            newtio.c_cflag |= CMSPAR;    /* Stick parity instead */
            newtio.c_cflag |= PARODD;    /* Even parity instead of odd */
            newtio.c_iflag |= INPCK;     /* Enable input parity check */
            break;
        case 's':
        case 'S':
            newtio.c_cflag |= PARENB;    /* Enable parity */
            newtio.c_cflag |= CMSPAR;    /* Stick parity instead */
            newtio.c_cflag &= ~PARODD;   /* Even parity instead of odd */
            newtio.c_iflag |= INPCK;     /* Enable input parity check */
            break;
        default:
            LOG(ERROR, "unsupported parity");
            return -1;
    }

    switch (serialOpt->stopbits) {
        case 1:
            newtio.c_cflag &= ~CSTOPB;
            break;
        case 2:
            newtio.c_cflag |= CSTOPB;
            break;
        default:
            LOG(ERROR, "unsupported stop bits\n");
            return -1;
    }

    /*hardware control*/
    if (serialOpt->hardflow) {
        newtio.c_cflag |= CRTSCTS;
    } else {
        newtio.c_cflag |= ~CRTSCTS;
    }

    /*read optional set*/
    newtio.c_cc[VTIME] = 10;	/* Time-out value (tenths of a second) [!ICANON]. */
    newtio.c_cc[VMIN] = 0;	/* Minimum number of bytes read at once [!ICANON]. */

    /*set serial flush*/
    tcflush(m_fd, TCIOFLUSH);

    if (tcsetattr(m_fd, TCSANOW, &newtio) != 0) {
        LOG(ERROR, "tcsetattr error");
        return -1;
    }

    return 0;
}

int Serial::Write(char *buf, int len)
{
    return write(m_fd, buf, len);
}

int Serial::Read(char *buf, int len)
{
    return read(m_fd, buf, len);
}

void Serial::StartAutoRead()
{
    this->start();
    this->detach();
    isAutoRead = true;
}


void Serial::StopAutoRead()
{
    this->stop();
	isAutoRead = false;
}

int Serial::Destroy()
{
    if (isAutoRead)
        StopAutoRead();
    return close(m_fd);
}

void Serial::SetSerialHandler(void (*xerc)(uint8_t *, uint16_t))
{
    OnSerialEnd = xerc;
}

void Serial::run()
{
    struct pollfd poll_set[1];
    poll_set[0].fd = m_fd;
    poll_set[0].events = POLLIN;
    poll_set[0].revents = 0;
    int timeout = 1000;
    uint8_t buf[1024];

    while(!this->isStoped()) {
        switch (poll(poll_set, 1, timeout)) {
        case -1:
            perror("poll");
            exit(1);
            break;
        case 0:
            break;
        default:
            uint16_t len;
            memset(buf, 0, sizeof (buf));
            len = read(m_fd, buf, sizeof(buf));
            buf[len - 1] = '\0';
            OnSerialEnd(buf, len);
            break;
        }
    }
}
