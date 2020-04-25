#ifndef GPIO_HPP
#define GPIO_HPP

#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdio.h>
#include <poll.h>

#define HALL_EFFECT         433 // hall efect pin 7 gpio4
#define GPIO_PIN_ENC_A      432 // encoder channel A, gpio pin 13
#define GPIO_PIN_MOT_EN     5   // motor enable
#define GPIO_PIN_ENC_B      431 // encoder channel B, gpio pin 15
#define GPIO_PIN_22         22
#define GPIO_PIN_MOT_1_DIR  24  // motor 1 direction
#define GPIO_PIN_MOT_2_DIR  25  // motor 2 direction
#define GPIO_PIN_26         26
#define GPIO_PIN_27         27
#define MFRC522_RST_PIN     329
#define GPIO_PIN_6_TEST_DIR 6   // test direction pin motor 1
#define VALUE_HIGH          1
#define VALUE_LOW           0
#define BUFFER_MAX          10
#define PATH_BUFF_MAX       35
#define READ_BUFF_MAX       1
#define FORWARD             1
#define REVERSE             0
#define GPIO_PIN_PWM        32
#define BAT_DISON_PIN       402
#define GIMBAL_PWR_PIN      327

static int GPIOExport(int pin)
{
    char buffer[BUFFER_MAX];
    ssize_t bytes_printed;
    ssize_t bytes_written;
    int fd;
    fd = open("/sys/class/gpio/export", O_WRONLY);
    if (-1 == fd) {
        fprintf(stderr, "Failed to open export for writing!\n");
        return(-1);
    }
    bytes_printed = snprintf(buffer, BUFFER_MAX, "%d", pin);
    bytes_written = write(fd, buffer, bytes_printed);
    close(fd);
    return(0);
}

static int GPIOUnexport(int pin)
{
    char buffer[BUFFER_MAX];
    ssize_t bytes_printed;
    ssize_t bytes_written;
    int fd;
    fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (-1 == fd) {
        fprintf(stderr, "Failed to open unexport for writing!\n");
        return(-1);
    }
    bytes_printed = snprintf(buffer, BUFFER_MAX, "%d", pin);
    bytes_written = write(fd, buffer, bytes_printed);
    close(fd);
    return(0);
}

static int GPIODirection(int pin, const char *dir)
{
    char path[PATH_BUFF_MAX];
    char buffer[BUFFER_MAX];
    ssize_t bytes_printed;
    ssize_t bytes_written;
    int fd;
    bytes_printed = snprintf(path, PATH_BUFF_MAX,
                             "/sys/class/gpio/gpio%d/direction", pin);
    fd = open(path, O_WRONLY);
    if (-1 == fd) {
        fprintf(stderr, "Failed to open gpio direction for writing!\n");
        return(-1);
    }
    bytes_printed = snprintf(buffer, BUFFER_MAX, "%s", dir);
    bytes_written = write(fd, buffer, bytes_printed);
    close(fd);
    return(0);
}

static int GPIOWrite(int pin, int value)
{
    char path[PATH_BUFF_MAX];
    char buffer[BUFFER_MAX];
    ssize_t bytes_printed;
    ssize_t bytes_written;
    int fd;
    bytes_printed = snprintf(path, PATH_BUFF_MAX,
                             "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_WRONLY);
    if (-1 == fd) {
        fprintf(stderr, "Failed to open gpio value for writing!\n");
        return(-1);
    }
    bytes_printed = snprintf(buffer, BUFFER_MAX, "%d", value);
    bytes_written = write(fd, buffer, bytes_printed);
    close(fd);
    return(0);
}

static int GPIORead(int pin, int &value)
{
    char path[PATH_BUFF_MAX];
    char readChar;
    ssize_t bytes_read;
    ssize_t bytes_printed;
    int fd;
    bytes_printed = snprintf(path, PATH_BUFF_MAX,
                             "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_RDONLY);
    if (-1 == fd) {
        fprintf(stderr, "Failed to open gpio value for reading!\n");
        return(-1);
    }
    bytes_read = read(fd, &readChar, 1);
    if (readChar != '0') {
        value = 1;
    }
    else {
        value = 0;
    }
    close(fd);
    return(0);
}

static int GPIOEdge(int pin, const char *edgeType)
{
    char path[PATH_BUFF_MAX];
    char buffer[BUFFER_MAX];
    ssize_t bytes_printed;
    ssize_t bytes_written;
    int fd;
    bytes_printed = snprintf(path, PATH_BUFF_MAX,
                             "/sys/class/gpio/gpio%d/edge", pin);
    fd = open(path, O_WRONLY);
    if (-1 == fd) {
        fprintf(stderr, "Failed to open gpio value for writing!\n");
        return(-1);
    }
    bytes_printed = snprintf(buffer, BUFFER_MAX, "%s", edgeType);
    bytes_written = write(fd, buffer, bytes_printed);
    close(fd);
    return(0);
}

static int GPIOGetFd(int pin)
{
    char path[PATH_BUFF_MAX];
    ssize_t bytes_printed;
    int fd;
    bytes_printed = snprintf(path, PATH_BUFF_MAX,
                             "/sys/class/gpio/gpio%d/value", pin);
    fd = open(path, O_RDONLY);
    if (-1 == fd) {
        fprintf(stderr, "Failed to open gpio value for reading!\n");
        return(-1);
    }
    return fd;
}

static int GPIOReadWithFd(int fd)
{
    char path[PATH_BUFF_MAX];
    char readChar;
    ssize_t bytes_read;
    bytes_read = read(fd, &readChar, 1);
    int value;
    if (readChar != '0') {
        value = 1;
    }
    else {
        value = 0;
    }
    close(fd);
    return(value);
}

#endif // GPIO_HPP
