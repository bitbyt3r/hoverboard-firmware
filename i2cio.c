#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
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

    unsigned char *fulldata;
    fulldata = (unsigned char *) malloc(length+1);
    fulldata[0] = reg_addr;
    memcpy(&fulldata[1], &data, length+1);
    if (write(i2c_file, fulldata, length+1) != length+1) {
        perror("Failed to write to i2c device");
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

    if (write(i2c_file, &reg_addr, 1) != 1) {
        perror("Failed to set read address of i2c device");
        return -1;
    }
    
    if (read(i2c_file, data, length) != length) {
        perror("Failed to read from i2c device");
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