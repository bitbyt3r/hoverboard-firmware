#include <errno.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#define DEFAULT_MPU_HZ  (100)

static signed char gyro_orientation[9] = { 0, 0, -1,
                                           0,-1, 0,
                                           1, 0, 0};

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

int serial;
void set_speed(int speed) {
    char buffer[50];
    int length = sprintf(buffer, "v 0 %d\nv 1 %d\n", speed, -1*speed);
    write(serial, buffer, length);
}

void main() {
    struct int_param_s int_param;
    int result;
    mpu_init(&int_param);

    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);

    short data[3];
    mpu_get_accel_reg(data, NULL);
    printf("Got accel %d %d %d\n", data[0], data[1], data[2]);

    if ((result = dmp_load_motion_driver_firmware()) < 0) {
        printf("Failed to load DMP firmware: %d\n", result);
        exit(-1);
    }
    if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) {
        printf("Failed to set DMP Orientation\n");
        exit(-1);
    }

    if (dmp_enable_gyro_cal(1) < 0) {
        printf("Failed to enable gyro calibration\n");
        exit(-1);
    }

    unsigned short dmp_features = DMP_FEATURE_TAP | DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_CAL_GYRO;
    if (dmp_enable_feature(dmp_features)) {
        printf("Failed to enable DMP Features\n");
        exit(-1);
    }
    if (dmp_set_fifo_rate(DEFAULT_MPU_HZ)) {
        printf("Failed to set FIFO rate\n");
        exit(-1);
    }
    if (mpu_set_dmp_state(1)) {
        printf("Failed to enable DMP\n");
        exit(-1);
    }
    
    if ((serial = open("/dev/ttyO2", O_RDWR | O_SYNC)) < 0) {
        perror("Failed to open serial port");
        exit(-1);
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial, &tty) != 0) {
        perror("Failed to acquire termios device");
        exit(-1);
    }
    cfsetospeed(&tty, 115200);
    cfsetispeed(&tty, 115200);
    set_speed(0);

    while (1) {
        short gyro[3], accel[3], sensors;
        unsigned long sensor_timestamp;
        unsigned char more;
        long quat[4];
        dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
        if (sensors & INV_WXYZ_QUAT) {
            double ws = (double)quat[0] * (double)quat[0];
            double xs = (double)quat[1] * (double)quat[1];
            double ys = (double)quat[2] * (double)quat[2];
            double zs = (double)quat[3] * (double)quat[3];
            double sum = ws + xs + ys + zs;
            double magnitude = sqrt(sum);
            double w = quat[0] / magnitude;
            double x = quat[1] / magnitude;
            double y = quat[2] / magnitude;
            double z = quat[3] / magnitude;
            double sinp = 2.0 * (w*y-z*x);
            double pitch;
            if (fabs(sinp) >= 1) {
                pitch = 3.1415926 / 2;
            } else {
                pitch = asin(sinp);
            }
            // printf("%.2f %.2f %.2f %.2f\n", w, x, y, z);
            set_speed((int)(pitch*300));
        }
    }
}
