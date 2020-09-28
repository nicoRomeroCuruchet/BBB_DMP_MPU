CXX=arm-linux-gnueabihf-g++-4.6 -std=gnu++0x
AR=ar
CXXCFLAGS=-Wall -g -O2
CXX_OPTS=-c
LDFLAGS= -lm -lrt

LIB=libGPIOdev.a
OBJ=GPIO.o

%.o: %.c                                                                         
	$(CXX) $(CXXFLAGS) $(CXX_OPTS) $< -o $@ 

$(LIB): $(OBJ)
	$(AR) rcs $(LIB) $(OBJ)

all: $(LIB)

clean:
	rm -rf *.o
	rm -rf $(LIB)
