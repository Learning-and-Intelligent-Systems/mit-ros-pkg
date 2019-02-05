
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
CFLAGS += -g -I./include -I../../include -I/usr/include
LDFLAGS += -L/usr/lib -L/usr/local/lib -L../../lib 
LDFLAGS += -lpthread -lncurses -lm -lbtwam -lbtsystem
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


OBJS = btdiag.o
TARG = btdiag
CFLAGS += -I$(shell rospack find BarrettWAM)/btclient/include
LDFLAGS += -L$(shell rospack find BarrettWAM)/btclient/lib -lbtwam -lbtsystem

btdiag: ${OBJS}
	$(CC) -o ${TARG} ${OBJS} ${LDFLAGS}

clean:
	rm btdiag
	rm btdiag.o

#ROS's node.mk uses g++, which doesn't like to compile Barrett's code
#SRC = btdiag.c
#OUT = btdiag
#PKG = WAMinterface
#LFLAGS += ${CFLAGS} ${LDFLAGS}
#include $(shell rospack find mk)/node.mk


