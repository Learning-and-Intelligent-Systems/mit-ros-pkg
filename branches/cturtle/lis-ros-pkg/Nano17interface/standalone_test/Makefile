CC = gcc
all: testNano17

LDFLAGS = -L../lib -lnano17 -lm -pthread
CFLAGS = -g -Wall -I../include

testNano17.o: testNano17.c
	$(CC) $(CFLAGS) -c testNano17.c -o testNano17.o

testNano17: testNano17.o ../lib/nano17lib.c
	$(CC) testNano17.o $(LDFLAGS) -o testNano17

clean:
	rm testNano17.o testNano17
