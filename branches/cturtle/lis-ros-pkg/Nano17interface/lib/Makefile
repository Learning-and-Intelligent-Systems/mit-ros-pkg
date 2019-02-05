CC = gcc
SRC = btserial.c nano17lib.c
TARGET = libnano17.a
OBJS = $(SRC:.c=.o)
CFLAGS = -g -I../include

all: lib

%.o : %.c
	@echo "[CC]    " $<
	@echo "making .c:" $(CC) $(CFLAGS) -c $< -o $@ 
	@$(CC) $(CFLAGS) -c $< -o $@ 

lib: ${OBJS}
	@echo "[AR]     $(TARGET)"
	@echo "making lib:" $(AR) rs $(TARGET) $^ 2> /dev/null
	@$(AR) rs $(TARGET) $^ 2> /dev/null

clean:
	rm libnano17.a btserial.o nano17lib.o
