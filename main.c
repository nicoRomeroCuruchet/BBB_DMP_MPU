#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include<fcntl.h>
#include<termios.h> // using the termios.h library
#include <stdint.h>

#include "MotionSensor/helper_3dmath.h"
#include "MotionSensor/inv_mpu_lib/inv_mpu.h"
#include "MotionSensor/inv_mpu_lib/inv_mpu_dmp_motion_driver.h"

#define DIM 3
#define YAW 0 
#define PITCH 1
#define ROLL 2

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
#define delay_ms(a)    usleep(a*1000)
#define RAD_TO_DEG  57.296

int r;
int16_t sensors;

uint8_t fifoCount;         // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];    // FIFO storage buffer

int16_t a[3];              // [x, y, z]            accel vector
int16_t g[3];              // [x, y, z]            gyro vector
int32_t _q[4];
int16_t c[3];

VectorFloat gravity;    // [x, y, z]            gravity vector

float ypr[3];
float ypr_acc[3];
int16_t acc_xyz[3];
int16_t gyro_xyz[3];

Quaternion q;
float gyro[3];
float accel[3];
float compass[3];

int initDMP(void);
void updateDMP(void);
uint8_t getGravity(VectorFloat *v, Quaternion *q);
uint8_t getYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

int main(){

  int file, count;
  char roll_acc_str[20];
  char dmp_roll_acc_str[20];
  char to_uart[30];

  if ((file = open("/dev/ttyO4", O_RDWR | O_NOCTTY | O_NDELAY))<0){
     perror("UART: Failed to open the file.\n");
     return -1;
  }
  struct termios options; // the termios structure is vital
  tcgetattr(file, &options); // sets the parameters associated with file
  // Set up the communications options:
  // 9600 baud, 8-bit, enable receiver, no modem control lines
  options.c_cflag = B230400 | CS8 | CREAD | CLOCAL;
  options.c_iflag = IGNPAR | ICRNL; // ignore partity errors, CR -> newline
  tcflush(file, TCIFLUSH); // discard file information not transmitted
  tcsetattr(file, TCSANOW, &options); // changes occur immmediately


  for(unsigned int i = 0; i < 10; ++i ) to_uart[i] = '\0';
 
  if (!initDMP()){
    perror("Error: DMP not ready!!\n");
    return -1;
  }

  float roll_gyro_offset = 0;
  for(unsigned int i = 0; i < 3000; i++){

    mpu_get_gyro_reg(gyro_xyz);
    roll_gyro_offset += gyro_xyz[0];
    
  }

  roll_gyro_offset /= 3000;

  float roll_acc;
  //printf("DMP - ROLL");
  printf("ROLL_DMP ROLL_ACC ROLL_RATE_DMP ROLL_RATE_GYRO\n");
  for(;;)
  { 
     // DMP update, YAW - PITCH - ROLL (ypr)
     updateDMP();

     // read direct registers of accelerometer
     mpu_get_accel_reg(acc_xyz);
     mpu_get_gyro_reg(gyro_xyz);

     // estimate roll and pitch directs accelerometer reads
     //pitch_acc = atan2(acc_xyz[0], sqrt(acc_xyz[1]*acc_xyz[1] + acc_xyz[2]*acc_xyz[2]))*180/M_PI;
     roll_acc = atan2(acc_xyz[1],acc_xyz[2])*180/M_PI;

     // ROLL - Rotate around X
     // PITCH - Rotate  around Y
     printf("%2.4f %2.4f %2.4f %2.4f\n", ypr[ROLL], roll_acc, gyro[ROLL] / 16.4, (gyro_xyz[0] - roll_gyro_offset)/16.4 );

     // convert to string to transmit via UART
     gcvt( (gyro[ROLL] / 16.4), 4, dmp_roll_acc_str);
     gcvt((gyro_xyz[0] - roll_gyro_offset)/16.4, 4,  roll_acc_str);

     strcpy(to_uart, "DMP_ROLL:");
     strcat(to_uart, dmp_roll_acc_str);
     strcat(to_uart, "\t");
     strcat(to_uart, "ACC_ROLL:");
     strcat(to_uart, roll_acc_str);
     strcat(to_uart,  "\n");

     if ((count = write(file, to_uart,strlen(to_uart)))<0){
       perror("Failed to write to the output\n");
       return -1;
     }
     delay_ms(5);
  }

  return 0;
}


int initDMP(void)
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

void updateDMP(void)
{
   while (dmp_read_fifo(g,a,_q,&sensors,&fifoCount)!=0); //gyro and accel can be null because of being disabled in the efeatures
   q = _q;
   getGravity(&gravity, &q);
   getYawPitchRoll(ypr, &q, &gravity);

   //scaling for degrees output
   for (int i=0;i<DIM;i++)
     ypr[i]*=180/M_PI;

   //unwrap yaw when it reaches 180
   ypr[YAW] = wrap_180(ypr[0]);

  //change sign of Pitch, MPU is attached upside down
  //ypr[PITCH]*=-1.0;

  //0=gyroX, 1=gyroY, 2=gyroZ
  //swapped to match Yaw,Pitch,Roll
  //Scaled from deg/s to get tr/s
  for (int i=0;i<DIM;i++){
  gyro[i]   = (float)(g[DIM-i-1]); //131.0/360.0;
  accel[i]  = (float)(a[DIM-i-1]);
  compass[i] = (float)(c[DIM-i-1]);
  }
}

uint8_t getGravity(VectorFloat *v, Quaternion *q) {
  v -> x = 2 * (q -> x*q -> z - q -> w*q -> y);
  v -> y = 2 * (q -> w*q -> x + q -> y*q -> z);
  v -> z = q -> w*q -> w - q -> x*q -> x - q -> y*q -> y + q -> z*q -> z;
  return 0;
}

uint8_t getYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
  // yaw: (about Z axis)
  data[YAW] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);

  // pitch: (nose up/down, about Y axis)
  //data[PITCH] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
  data[PITCH] = atan2(gravity -> x, sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));

  // roll: (tilt left/right, about X axis)
  //data[ROLL] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
  data[ROLL] = atan2(gravity -> y, gravity -> z );
  return 0;
}

