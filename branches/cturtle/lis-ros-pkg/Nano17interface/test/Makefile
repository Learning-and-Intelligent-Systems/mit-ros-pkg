CC = gcc
all: testNano17

LDFLAGS = -L../lib -lnano17 -pthread -lm
CFLAGS = -g -Wall -I../include

testNano17.o: testNano17.c
	$(CC) $(CFLAGS) -c testNano17.c -o testNano17.o

testNano17: testNano17.o
	$(CC) testNano17.o $(LDFLAGS) -o testNano17

clean:
	rm testNano17.o testNano17
