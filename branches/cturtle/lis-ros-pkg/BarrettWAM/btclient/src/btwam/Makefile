### Makefile for Barrett Software (libbtwam)

# Be sure to edit config.mk to match your installation
include ../../config.mk
include ../../common.mk

LIB_OBJS = btwam.o gimbals.o
LIB_TARG = libbtwam.a
SLIB_TARG = libbtwam.so
INCL_FILES = btwam.h gimbals.h
PATH_INSTALL_LIB = ../../lib
PATH_INSTALL_INC = ../../include

# Done with defines, now for the real work

default: lib

# Make *.o files rebuild on any change to *.h
$(LIB_OBJS) : $(INCL_FILES)

lib: ${LIB_OBJS}
	$(AR) rcs $(LIB_TARG) ${LIB_OBJS}
	$(RM) $*.o

slib: ${LIB_OBJS}
	${CC} -g ${LDFLAGS} -shared -o ${SLIB_TARG} ${LIB_OBJS}
   
clean: 
	rm *.o ${LIB_TARG} $(PATH_INSTALL_LIB)/${LIB_TARG}
   
install:
	cp $(LIB_TARG) $(PATH_INSTALL_LIB)
	cp $(INCL_FILES) $(PATH_INSTALL_INC)

