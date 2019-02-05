### Makefile for Barrett Software (examples)

# Be sure to edit config.mk to match your installation
include ../../config.mk
include ../../common.mk

OBJS = main.o
TARG = ex1

# Done with defines, now for the real work
default: ex1

ex1: ${OBJS}
	$(CC) -o ${TARG} ${OBJS} ${LDFLAGS}
  
clean:
	rm *.o ${TARG} 

system:
	$(MAKE) -C ../../src/btsystem lib
	$(MAKE) -C ../../src/btsystem install

wam:
	$(MAKE) -C ../../src/btwam lib
	$(MAKE) -C ../../src/btwam install
   
all: system wam ex1

