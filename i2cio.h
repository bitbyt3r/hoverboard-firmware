#include "sys/time.h"
#include "time.h"

int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
void delay_ms(int ms);
void get_ms(unsigned long *count);