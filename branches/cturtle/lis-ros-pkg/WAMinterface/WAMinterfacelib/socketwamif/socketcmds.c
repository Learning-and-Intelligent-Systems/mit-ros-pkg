//Socket utility functions

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <errno.h>

//establish a connection to the server
int socket_client_connect(char *server_ip, unsigned short int server_port){

  int sockid;
  int err;
  int flag = 1;
  char data;
  struct sockaddr_in ssock_addr;
  
  bzero((char *) &ssock_addr, sizeof(ssock_addr));
  ssock_addr.sin_family = AF_INET;
  ssock_addr.sin_addr.s_addr = inet_addr(server_ip);
  ssock_addr.sin_port = htons(server_port);
  
  //Create Socket    
  if ((sockid = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0){
		fprintf(stderr, "error creating client socket, error %d: %s\n", errno, strerror(errno));
		return(0);
  }
	
  if(setsockopt(sockid, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int)) < 0) {
    fprintf(stderr, "error changing client socket option, error %d: %s\n", errno, strerror(errno));
    return(0);
  }
  
  //Connect to Server
  if (connect(sockid,(struct sockaddr *)&ssock_addr,sizeof(ssock_addr))<0){
		fprintf(stderr, "error connecting to server, error %d: %s\n", errno, strerror(errno));
		return(0);
	}
  
  //Wait for connection token from controller (disabled)
	/*
  while ((err = read(sockid, &data, 1)) == 0){
		usleep(1000);
	}  
  if ((err != 1) || (data != 'c')){
		fprintf(stderr, "Connection refused by controller\n");
		close(sockid);
		return(0);
		} */

     
  return(sockid);
}


//close the connection to the server
int socket_client_close(int sockid){
  send(sockid, "q\0", 2, 0);
  usleep(1000);
  return(close(sockid));
}


//read a char (returns 0 if error, else 1)
int read_char(int sockid, char *data){
	int err;
	int tries = 0;
	while ((err = read(sockid, data, 1)) <= 0 && tries++<100){
		usleep(1000);
	}
	if(err > 0)
		return 1;
	else{
		printf("read char error, err:%d \n", err);
		return 0;
	}
}


//Read an integer from the socket (returns 0 if error, else 1)    
//the terminating separation character (' ' or '\0') is put in separator   
int read_int(int sockid, int *num, char *separator){
  int i = 0;
  int err;
  char anum[256];
  char data;
	int tries = 100;
  do{
		err = read_char(sockid, &data);
    if (err > 0)
      anum[i++] = data;
    else
      return(0);
  } while ((data != ' ') && (data != '\0') && (data != '\n') && tries-->0);
  anum[--i] = '\0';
  *separator = data;
	/*
  errno = 0;
  *num = (int)strtol(anum, (char **)NULL, 10);
  if (errno != 0)
    return(0);
  else
	return(1);*/
	int numscanned = 0;
	numscanned = sscanf(anum, "%d", num);
	//printf("num: %d, numscanned: %d\n", *num, numscanned);
	if(numscanned != 1)
		return(0);
	return(1);
}


//Read a double from the socket (returns 0 if error, else 1)    
//the terminating separation character (' ' or '\0' or '\n') is put in separator  
int read_double(int sockid, double *num, char *separator){
  int i = 0;
  int err;
  char anum[256];
  char data;

  do{
		err = read_char(sockid, &data);
			/*
    while ((err = read(sockid, &data, 1)) == 0){
			usleep(1000);
			}*/
    if (err > 0)
      anum[i++] = data;
    else
      return(0);
  } while ((data != ' ') && (data != '\0') && (data != '\n'));

  anum[--i] = '\0';
  *separator = data;
  errno = 0;
  *num = strtod(anum, (char **)NULL);
  if (errno != 0)
    return(0);
  else
    return(1);
}

//read a list of ints from the socket (returns 0 if error, else 1)
//the final terminating separation character (' ' or '\0' or '\n') is put in separator  
int read_int_array(int sockid, int *array, int count, char *separator){
  int err = 1;
  int i = 0;
  *separator = ' ';
  while ((err) && (i<count)){
		err = read_int(sockid, &(array[i++]), separator);
	}
	if(err == 0 || i!=count){
		printf("error in read_int_array!\n");
		return 0;
	}
  return 1;
}

//read a list of doubles from the socket (returns 0 if error, else 1)
//the final terminating separation character (' ' or '\0' or '\n') is put in separator  
int read_double_array(int sockid, double *array, int count, char *separator){
  int err = 1;
  int i = 0;
  *separator = ' ';
  while ((err) && (i<count)){
		err = read_double(sockid, &(array[i++]), separator);
	}
	if(err == 0 || i!=count){
		printf("error in read_double_array!\n");
		return 0;
	}
  return 1;
}

//read in a string terminated by '\n' or '\0' (returns -1 if error, else length of string)
int read_string(int sockid, char *buffer, char *separator){
  int i = 0;
  int err;
  char data;

  do{
		err = read_char(sockid, &data);
    if(err > 0)
      buffer[i++] = data;
    else
      return -1;
  } while ((data != '\0') && (data != '\n'));

  buffer[--i] = '\0';
  *separator = data;
	return i;
}


//write a list of doubles to the socket, ' ' separated and '\n' terminated 
//(returns 0 if error, else 1)
int write_double_array(int sockid, double *array, int count){
  int i, err = 0;
  char anum[256];

  for (i=0; i<count; i++){
		if (i==0)
			sprintf(anum, "%.16f", array[i]);
		else if(i==count-1)
			sprintf(anum, " %.16f\n", array[i]);
		else
			sprintf(anum, " %.16f", array[i]);
		int len = strlen(anum);
		err = (err || (write(sockid, anum, len) != len));
	}
  return !err;
}

//write a single int to the socket, terminated by terminationchar
//(returns 0 if error, else 1)
int write_int(int sockid, int data, char terminationchar){
	int err = 0;
	char anum[256];
	sprintf(anum, "%d%c", data, terminationchar);

	err = write(sockid, anum, strlen(anum)) != strlen(anum);

	return !err;
}

//write an array of ints to the socket, ' ' separated and '\n' terminated
//(returns 0 if error, else 1)
int write_int_array(int sockid, int *array, int count){
	int i, err = 0;
	char anum[256];
	for(i=0; i<count; i++){
		if(i==0)
			sprintf(anum, "%d", array[i]);
		else if(i==count-1)
			sprintf(anum, " %d\n", array[i]);
		else
			sprintf(anum, " %d", array[i]);
		err = (err || (write(sockid, anum, strlen(anum)) != strlen(anum)));
	}
	return !err;
}

//read a block of size blocklen from the serial port (1-second timeout)
//returns -1 if error, else length read
int read_block(int sockid, char *buffer, int blocklen){
	int err;
	int lengthRead = 0;
	int bytesRead = 0;
	int ms = 1000;

	while(lengthRead < blocklen) {
		err = serialRead(sockid, buffer, 1, &bytesRead);
		/*
		//error in reading
		if(err){
			printf("error in reading\n");
			return -1;
			}*/
		
		lengthRead += bytesRead;
		buffer += bytesRead;
		usleep(20000); // Sleep for 20ms
		ms -= 20;
		if(ms < 0){
			printf("timeout!\n");
			return -1;
		}
	}	
	
	buffer[blocklen] = '\0'; // Null terminate
	
	if(lengthRead != blocklen){
		printf("error, bytesRead = %d, string read: %s\n", bytesRead, buffer);
		return -1;
	}
	return lengthRead;
}
