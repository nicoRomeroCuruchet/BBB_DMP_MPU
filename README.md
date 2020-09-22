BagleBone Black, the MPU6050’s onboard Digital Motion Processor (DMP) and USB TTL Serial are used in this project. 

The USB TTL Serial cables are a range of USB to serial converter cables which provide connectivity between USB and serial UART interfaces. The DMP offloads processing that would normally have to take place on the microprocessor. It maintains an internal buffer that combines data from the gyro and accelerometer and computes orientation for you. The DMP also takes care of the applying the offsets, so you don’t have to keep track of these in your project code. UART is used for the serial transfer of data, one bit at a time, for graphing and comparing ROLL angle directly from accelerometer and via DMP data fusion microprocessor. 

BBB UART Header Pins:

    NA     |UART1  |   UART2  |   UART3       |  UART4  |   UART5  
    TXD    |P9_24  |   P9_21  |   Not exposed |  P9_13  |   P8_37 
    RXD    |P9_26  |   P9_22  |   Not exposed |  P9_11  |   P8_38 

For the following example, the BBB’s UART4 is used:

P9_13 UART4_TXD  <----> connected <----> RXD_USB_TTL 

P9_11 UART4_RXD  <----> connected <----> TXD_USB_TTL 

The UART4 overlay is loaded in order to enable the UART4, for this write the following commands:

export SLOTS=/sys/devices/bone_capemgr.9/slots

sudo su -c "echo BB-UART4 > $SLOTS"

CONNECTIONS BBB - MPU6050 (I2C2 /dev/i2c‐1)

    |P9_20 |SDA PIN
    |P9_19 |SLC PIN 
    |P9_3  |VCC PIN
    |P9_1  |GND PIN
 
