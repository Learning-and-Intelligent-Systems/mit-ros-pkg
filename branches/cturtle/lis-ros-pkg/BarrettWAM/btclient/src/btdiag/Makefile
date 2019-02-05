### Makefile for Barrett Software (btdiag)

# Be sure to edit config.mk to match your installation
include ../../config.mk
include ../../common.mk

OBJS = btdiag.o
TARG = btdiag

# Done with defines, now for the real work
default: btdiag

btdiag: ${OBJS}
	$(CC) -o ${TARG} ${OBJS} ${LDFLAGS}
  
clean:
	rm *.o ${TARG} 

system:
	$(MAKE) -C ../btsystem lib
	$(MAKE) -C ../btsystem install

wam:
	$(MAKE) -C ../btwam lib
	$(MAKE) -C ../btwam install
   
all: system wam btdiag

