#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#define DEFAULT_MPU_HZ  (100)

static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

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

    unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT;
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
    sleep(2);

    while (1) {
            short gyro[3], accel[3], sensors;
            unsigned long sensor_timestamp;
            unsigned char more;
            long quat[4];
            /* This function gets new data from the FIFO when the DMP is in
             * use. The FIFO can contain any combination of gyro, accel,
             * quaternion, and gesture data. The sensors parameter tells the
             * caller which data fields were actually populated with new data.
             * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
             * the FIFO isn't being filled with accel data.
             * The driver parses the gesture data to determine if a gesture
             * event has occurred; on an event, the application will be notified
             * via a callback (assuming that a callback function was properly
             * registered). The more parameter is non-zero if there are
             * leftover packets in the FIFO.
             */
            dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);
            /* Gyro and accel data are written to the FIFO by the DMP in chip
             * frame and hardware units. This behavior is convenient because it
             * keeps the gyro and accel outputs of dmp_read_fifo and
             * mpu_read_fifo consistent.
             */
            if (sensors & INV_XYZ_GYRO)
                printf("Got Gyro Reading: %d %d %d\n", gyro[0], gyro[1], gyro[2]);
            if (sensors & INV_XYZ_ACCEL)
                printf("Got Accel Reading: %d %d %d\n", accel[0], accel[1], accel[2]);
            /* Unlike gyro and accel, quaternions are written to the FIFO in
             * the body frame, q30. The orientation is set by the scalar passed
             * to dmp_set_orientation during initialization.
             */
            if (sensors & INV_WXYZ_QUAT)
                printf("Get quaternion reading\n");
    }
}
