CXX=arm-linux-gnueabihf-g++-4.6
CXXFLAGS= -Wall -g -O2 -std=gnu++0x -pthread 
CXX_OPTS= -Wall -g -O2 -std=gnu++0x -pthread

PROG=dmp

%.o: %.c                                                                         
	$(CXX) $(CXXFLAGS) $(CXX_OPTS) $< -o $@ 


all: $(PROG).o 
	$(CXX) $(LDFLAGS) $(CXXFLAGS) -o $(PROG) \
		main.c \
                GPIO/libGPIOdev.a\
		MotionSensor/libMotionSensor.a \
		libs/libI2Cdev.a

$(PROG).o: MotionSensor/libMotionSensor.a libs/libI2Cdev.a GPIO/libGPIOdev.a

MotionSensor/libMotionSensor.a:
	$(MAKE) -C MotionSensor/ 

libs/libI2Cdev.a:
	$(MAKE) -C libs/I2Cdev 

GPIO/libGPIOdev.a:
	$(MAKE) -C GPIO/
clean:
	cd MotionSensor && $(MAKE) clean
	cd libs/I2Cdev && $(MAKE) clean
	cd GPIO && $(MAKE) clean
	rm -rf *.o *~ *.mod
	rm -rf $(PROG)
