#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "i2c-dev.h"
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "sys/time.h"
#include "time.h"

int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data) {
    int i2c_file;
    if ((i2c_file = open("/dev/i2c-1", O_RDWR)) < 0) {
        perror("Failed to open i2c device\n");
        return -1;
    }
    if (ioctl(i2c_file, I2C_SLAVE, slave_addr) < 0) {
        perror("Failed to set i2c address\n");
        return -1;
    }

    int result = i2c_smbus_write_i2c_block_data(i2c_file, reg_addr, length, data);
    if (result) {
        printf("Failed to write to i2c device: %d\n", result);
        return -1;
    }

    if (close(i2c_file) < 0) {
        perror("Failed to close i2c device\n");
        return -1;
    }
    return 0;
}

int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data) {
    int i2c_file;
    if ((i2c_file = open("/dev/i2c-1", O_RDWR)) < 0) {
        perror("Failed to open i2c device\n");
        return -1;
    }
    if (ioctl(i2c_file, I2C_SLAVE, slave_addr) < 0) {
        perror("Failed to set i2c address\n");
        return -1;
    }

    int result = i2c_smbus_read_i2c_block_data(i2c_file, reg_addr, length, data);
    if (result != length) {
        printf("Failed to read from i2c device: %d\n", result);
        return -1;
    }

    if (close(i2c_file) < 0) {
        perror("Failed to close i2c device\n");
        return -1;
    }
    return 0;
}

void delay_ms(int ms) {
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = 1000000*ms;
    nanosleep(&ts, NULL);
}

void get_ms(unsigned long *count) {
    struct timeval te;
    gettimeofday(&te, NULL);
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000;
    *count = milliseconds;
}