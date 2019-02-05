### Makefile for Barrett Software (libbtsystem)

# Be sure to edit config.mk to match your installation
include ../../config.mk
include ../../common.mk

LIB_OBJS =  btsystem.o \
            btlogger.o btcan_peakesd.o btrobot.o btmath.o \
            btos.o btpath.o btcontrol.o btserial.o bthaptics.o btstatecontrol.o \
            btgeometry.o btparser.o btseg.o
INCL_FILES = btcan.h btsystem.h  btlogger.h \
            btmath.h btrobot.h btos.h btpath.h btcontrol.h btserial.h bthaptics.h \
            btstatecontrol.h btgeometry.h btparser.h btseg.h
            
LIB_TARG = libbtsystem.a
SLIB_TARG = libbtsystem.so
PATH_INSTALL_LIB = ../../lib
PATH_INSTALL_INC = ../../include

# Done with defines, now for the real work

default: lib

# Make *.o files rebuild on any change to *.h
$(LIB_OBJS) : $(INCL_FILES)

lib: ${LIB_OBJS}
	$(AR) rcs $(LIB_TARG) ${LIB_OBJS}
	$(RM) *.o

slib: ${LIB_OBJS}
	${CC} -g ${LDFLAGS} -shared -o ${SLIB_TARG} ${LIB_OBJS}
   
clean: 
	rm *.o ${LIB_TARG} $(PATH_INSTALL_LIB)/${LIB_TARG}
   
install:
	cp $(LIB_TARG) $(PATH_INSTALL_LIB)
	cp $(INCL_FILES) $(PATH_INSTALL_INC)

listing: ${LIB_OBJS}
	objdump -S $< > $@


