hoverboard: hoverboard.o inv_mpu.o inv_mpu_dmp_motion_driver.o i2cio.o
	gcc hoverboard.o inv_mpu.o inv_mpu_dmp_motion_driver.o i2cio.o -o hoverboard 

hoverboard.o: hoverboard.c
	gcc -c hoverboard.c

inv_mpu.o: inv_mpu.c
	gcc -c inv_mpu.c -D EMPL_TARGET_BBB -D MPU6050

inv_mpu_dmp_motion_driver.o: inv_mpu_dmp_motion_driver.c
	gcc -c inv_mpu_dmp_motion_driver.c -D EMPL_TARGET_BBB -D MPU6050

i2cio.o: i2cio.c
	gcc -c i2cio.c
