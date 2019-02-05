
#from Barrett's config.mk -- Configuration details
# Define the target operating system
# TARGET_OS = rtai
# TARGET_OS = xenomai

TARGET_OS=xenomai

# Define the CANbus hardware installed
# CAN_HARDWARE = esd
# CAN_HARDWARE = peak

CAN_HARDWARE=peak

# Define the type of CAN interface
# CAN_TYPE = isa
# CAN_TYPE = pci

CAN_TYPE=pci



#from Barrett's common.mk:
#set BTOLDCFG to -DBTOLDCONFIG for RIC,MIT,Ualberta,BPW
#ex: #export BTOLDCFG=-DBTOLDCONFIG
CFLAGS  += ${BTOLDCFG}
CC = gcc

# Compiler flags common to every system
CFLAGS += -g -I/usr/include
LDFLAGS += -L/usr/lib -L/usr/local/lib
LDFLAGS += -lpthread -lncurses -lm
LDFLAGS += -Wl,-Map=$(TARG).map,--cref

# Choose the correct CAN library
ifeq ($(CAN_HARDWARE),esd)
LDFLAGS += -lntcan
CFLAGS += -DESD_CAN
endif

ifeq ($(CAN_HARDWARE),peak)
LDFLAGS += -lpcan
CFLAGS += -DPEAK_CAN
endif

ifeq ($(CAN_TYPE),isa)
CFLAGS += -DISA_CAN
endif

ifeq ($(CAN_TYPE),pci)
CFLAGS += -DPCI_CAN
endif

# Choose the correct operating system flags
ifeq ($(TARGET_OS),rtai)
CFLAGS += -DRTAI
CFLAGS += -I/usr/realtime/include
LDFLAGS += -L/usr/realtime/lib -llxrt
endif

ifeq ($(TARGET_OS),xenomai)
CFLAGS += -DXENOMAI
CFLAGS += -I/usr/xenomai/include
LDFLAGS += -L/usr/xenomai/lib -rdynamic -lnative
SKIN = xeno
### Xenomai directory, xeno-config and library directory ###########
XENO_DIR          ?= /usr/xenomai
XENO_CONFIG       ?= $(XENO_DIR)/bin/xeno-config
XENO_LIB_DIR      ?= $(shell $(XENO_CONFIG) --library-dir) -Wl,-rpath $(shell $(XENO_CONFIG) --library-dir)

### User space application compile options #########################
USERAPP_LIBS      ?= -lnative -lpcan
USERAPP_LDFLAGS   ?= $(shell $(XENO_CONFIG) --$(SKIN)-ldflags) -L$(XENO_LIB_DIR)
USERAPP_CFLAGS    ?= $(shell $(XENO_CONFIG) --$(SKIN)-cflags)

CFLAGS += ${USERAPP_CFLAGS}
LDFLAGS += ${USERAPP_LDFLAGS} ${USERAPP_LIBS}
endif


#include/link to the compiled Barrett libraries
CFLAGS += -I$(shell rospack find BarrettWAM)/btclient/include -I./ -Ihandinterface/
#LDFLAGS += -L$(shell rospack find BarrettWAM)/btclient/lib -lbtwam -lbtsystem

#order of -lwamif, -lbtwam, and -lbtsystem very important!
BTLDFLAGS = -L$(shell rospack find BarrettWAM)/btclient/lib -lbtwam -lbtsystem
WAMIFLDFLAGS = -Llib -lwamif
BOTHLDFLAGS = ${WAMIFLDFLAGS} ${BTLDFLAGS}

# Source files for binary/library
SRC =  log/log.c \
			 waminterface/wamif.c \
			 handinterface/handif.c

# Name of generated binary library
TARGET = lib/libwamif.a

OBJS = $(SRC:.c=.o)


# Socket interface to WAM
SOCKETSRC = socketwamif/socketwamif.c \
            socketwamif/socketcmds.c

SOCKETTARGET = socketwamif/socketwamif

SOCKETOBJS = $(SOCKETSRC:.c=.o)


# Test programs for this binary/library
TESTPROGRAMS = 	testprograms/WAMtest \
					testprograms/WAMtestcpp \
					testprograms/socketlinkdebugging/socketclientechotest \
					testprograms/socketlinkdebugging/socketserverechotest \
					testprograms/runcalibration \
					testprograms/RThandtest \
				  testprograms/handtest \
				  testprograms/straindump \
					testprograms/testhandicspeed \
					testprograms/jointtrajectorytest


CC  = gcc
CXX = g++
CFLAGS += -g

all: lib testprograms socket

%.o : %.c %.h
	@echo "[CC]    " $<
	@echo "making .c:" $(CC) $(CFLAGS) -c $< -o $@ 
	@$(CC) $(CFLAGS) -c $< -o $@ 

%.o : %.c
	@echo "[CC]    " $<
	@echo "making .c:" $(CC) $(CFLAGS) -c $< -o $@ 
	@$(CC) $(CFLAGS) -c $< -o $@ 

%.o : %.cpp %.h
	@echo "[CXX]   " $<
	@echo "making .cpp:" $(CXX) $(CFLAGS) -c $< -o $@ 
	@$(CXX) $(CFLAGS) -c $< -o $@ 

%.o : %.cpp
	@echo "[CXX]   " $<
	@echo "making .cpp:" $(CXX) $(CFLAGS) -c $< -o $@ 
	@$(CXX) $(CFLAGS) -c $< -o $@ 

lib: ${OBJS}
	@echo "[AR]     $(TARGET)"
	@echo "making lib:" $(AR) rs $(TARGET) $^ 2> /dev/null
	@$(AR) rs $(TARGET) $^ 2> /dev/null


socket: ${SOCKETTARGET}

${SOCKETTARGET}: ${SOCKETOBJS} ${TARGET}
	@echo "[LD]     $@"
	@echo "linking socket:" ${CC} ${SOCKETOBJS} $(LDFLAGS) ${BOTHLDFLAGS} -o $@
	@${CC} ${SOCKETOBJS} $(LDFLAGS) ${BOTHLDFLAGS} -o $@


testprograms: ${TESTPROGRAMS}

%: %.c ${TARGET}
	@echo "[LD]     $@"
	@echo "making test:" ${CC} $(CFLAGS) -c $< -o temp.o
	@${CC} $(CFLAGS) -c $< -o temp.o
	@echo "linking test:" ${CC} temp.o $(LDFLAGS) ${BOTHLDFLAGS} -o $@
	@${CC} temp.o ${WAMIFLDFLAGS} $(LDFLAGS) ${BTLDFLAGS} -o $@
	@rm temp.o

%: %.cpp ${TARGET}
	@echo "[LD]     $@"
	@echo "making test:" ${CXX} $(CFLAGS) -c $< -o temp.o
	@${CXX} $(CFLAGS) -c $< -o temp.o
	@echo "linking test:" ${CXX} temp.o $(LDFLAGS) ${BOTHLDFLAGS} -o $@
	@${CXX} temp.o $(LDFLAGS) ${BOTHLDFLAGS} -o $@
	@rm temp.o

clean:	
	@echo "Cleaning ${OBJS} ${TARGET} ${TESTS} ${SOCKETOBJS} ${SOCKETTARGET}"
	@rm -rf ${SOCKETOBJS} ${SOCKETTARGET}
	@rm -rf ${OBJS} ${TARGET}
	@rm -rf ${TESTPROGRAMS}




