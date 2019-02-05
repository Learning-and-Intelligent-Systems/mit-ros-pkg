CC = gcc
SRC = wamik.c WAMKinematics.c
TARGET = libwamik.a
OBJS = $(SRC:.c=.o)
CFLAGS = -g -I../include 

all: lib test solib

%.o : %.c
	@echo "[CC]    " $<
	@echo "making .c:" $(CC) $(CFLAGS) -c $< -o $@ 
	@$(CC) $(CFLAGS) -c $< -o $@ 

lib: ${OBJS}
	@echo "[AR]     $(TARGET)"
	@echo "making lib:" $(AR) rs $(TARGET) $^ 2> /dev/null
	@$(AR) rs $(TARGET) $^ 2> /dev/null
	rm *.o

test: testwamik.o
	@echo "making test:" $(CC) testwamik.o -o testwamik -L./ -lwamik -lm
	@$(CC) testwamik.o -o testwamik -L./ -lwamik -lm
	rm testwamik.o

solib: SOlib/wamik_interface.c
	cd SOlib; sh ./create_wamik_so.sh

clean:
	rm libwamik.a testwamik
