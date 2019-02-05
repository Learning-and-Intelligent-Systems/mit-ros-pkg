# This Makefile was tested with GNU Make

# Use pkg-config to lookup the proper compiler and linker flags for LCM
CFLAGS=`pkg-config --cflags lcm opencv`
LDFLAGS=`pkg-config --libs lcm glib-2.0 gthread-2.0 opencv` 

INCLUDE = -I/usr/include/ -I/usr/include/flycapture -I/usr/include/glib-2.0/ -I/usr/lib/glib-2.0/include

LCM_INCLUDE = -I${LCM_INC}
CARMEN_INCLUDE = -I${CARMEN_INC}
CARMEN3D_INCLUDE = -I${CARMEN3D_INC}
LIBBOT_INCLUDE = -I${LIBBOT_INC}
CARMEN_COMMON_INCLUDE = -I${CARMEN_COMMON_INC}
CARMEN_CAMERA_INCLUDE = -I${CARMEN_CAMERA_INC}

#-I/home/drl/carmen3D/src/lcmtypes/C_source -I/home/drl/carmen3D/carmen/include/ -I/home/drl/carmen3D/include/ -I/home/drl/carmen3D/externals/libbot/software/libs/bot/  -I/home/drl/carmen3D/src/common/ -I/home/drl/carmen3D/src/camera/ -I/usr/include/flycapture

LIBS = -L/usr/src/flycapture/lib -lcarmen3d_lcmtypes -llcm -lbot-core -lglobal -lglobal3d -lflycapture${D} -pthread -lcarmen3d_jpeg_codec

LCM_LIBRARY = -L${LCM_LIB}
CARMEN_LIBRARY = -L${CARMEN_LIB}
CARMEN3D_LIBRARY = -L${CARMEN3D_LIB}

#-L/home/drl/carmen3D/src/lcmtypes/ -L/home/drl/carmen3D/lib -L/home/drl/carmen3D/carmen/lib/ 

# list the LCM types here
lcmtypes= 

# derive the file names of the .c, .h, and .o files corresponding
# to the LCM types here
lcm_c_h_files=$(lcmtypes:%=%.c) $(lcmtypes:%=%.h)
lcm_obj_files=$(lcmtypes:%=%.o)

# compile the LCM types to a static library 
all: libtypes.a multi-histROS
multi-histROS: multi-histROS.o libtypes.a
	$(CC) -o $@ $^ $(LDFLAGS) $(LCM_LIBRARY) $(CARMEN_LIBRARY) $(CARMEN3D_LIBRARY) ${LIBS} 

# prevent auto-generated lcm .c/.h files from being deleted
.SECONDARY : $(lcm_c_h_files)

libtypes.a: $(lcm_obj_files)
	ar rc $@ $(lcm_obj_files)

%.o: %.c %.h
	gcc $(CFLAGS) -c $<  

%.o: %.cpp
	gcc $(CFLAGS) $(LCM_INCLUDE) $(CARMEN_INCLUDE) $(CARMEN3D_INCLUDE) $(LIBBOT_INCLUDE) $(CARMEN_COMMON_INCLUDE) $(CARMEN_CAMERA_INCLUDE) ${INCLUDE} -c $<  
# -I${OPENCV_LIB}
%.c %.h: %.lcm
	lcm-gen -c $<

clean:
	rm -f multi-histROS
	rm -f *.o
	rm -f libtypes.a
	rm -f $(lcm_c_h_files)

