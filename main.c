#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "MotionSensor/helper_3dmath.h"
#include "MotionSensor/inv_mpu_lib/inv_mpu.h"
#include "MotionSensor/inv_mpu_lib/inv_mpu_dmp_motion_driver.h"

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
#define delay_ms(a)    usleep(a*1000)

uint8_t fifoCount;         // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];    // FIFO storage buffer

int16_t a[3];              // [x, y, z]            accel vector
int16_t g[3];              // [x, y, z]            gyro vector
int32_t _q[4];
Quaternion q;

int r;
int16_t sensors;


int init_MPU(void);

int main(){

  int dmpReady = init_MPU();
  

  return 0;
}

int init_MPU(void)
{
   // initialize device
   printf("Initializing MPU...\n");
   if (mpu_init(NULL) != 0)
   {
     printf("MPU init failed!\n");
     return -1;
   }

  printf("Setting MPU sensors...\n");
  if (mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL)!=0)
  {
    printf("Failed to set sensors!\n");
    return -1;
  }
  printf("Setting GYRO sensitivity...\n");
  if (mpu_set_gyro_fsr(2000)!=0)
  {
    printf("Failed to set gyro sensitivity!\n");
    return -1;
  }

  printf("Setting ACCEL sensitivity...\n");
  if (mpu_set_accel_fsr(2)!=0)
  {
    printf("Failed to set accel sensitivity!\n");
    return -1;
  }
  // verify connection
  printf("Powering up MPU...\n");
  uint8_t devStatus;
  mpu_get_power_state(&devStatus);
  printf(devStatus ? "MPU6050 connection successful\n" : "MPU6050 connection failed %u\n",devStatus);

  //fifo config
  printf("Setting MPU fifo...\n");
  if (mpu_configure_fifo(INV_XYZ_GYRO|INV_XYZ_ACCEL)!=0) 
  {
    printf("Failed to initialize MPU fifo!\n");
    return -1;
  }

  // load and configure the DMP
  printf("Loading DMP firmware...\n");
  if (dmp_load_motion_driver_firmware()!=0) 
  {
    printf("Failed to enable DMP!\n");
    return -1;
  }

  printf("Activating DMP...\n");
  if (mpu_set_dmp_state(1)!=0) 
  {
    printf("Failed to enable DMP!\n");
    return -1;
  }

  printf("Configuring DMP...\n");
  if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|DMP_FEATURE_GYRO_CAL)!=0) 
  {
    printf("Failed to enable DMP features!\n");
    return -1;
  }


  printf("Setting DMP fifo rate...\n");
  uint8_t rate = 100;
  if (dmp_set_fifo_rate(rate)!=0) 
  {
    printf("Failed to set dmp fifo rate!\n");
    return -1;
  }
  printf("Resetting fifo queue...\n");
  if (mpu_reset_fifo()!=0) 
  {
    printf("Failed to reset fifo!\n");
    return -1;
  }

  printf("Checking... ");
  do 
  {
    delay_ms(1000/rate);  //dmp will habve 4 (5-1) packets based on the fifo_rate
    r=dmp_read_fifo(g,a,_q,&sensors,&fifoCount);
  } 
  while (r!=0 || fifoCount<5); //packtets!!!
  printf("Done.\n");

  return 1;
}







uint8_t GetGravity(VectorFloat *v, Quaternion *q) {
        v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
        v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
        v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
        return 0;
}

uint8_t GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
        // yaw: (about Z axis)
        data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
        // pitch: (nose up/down, about Y axis)
        data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
        // roll: (tilt left/right, about X axis)
        data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
        return 0;
}



