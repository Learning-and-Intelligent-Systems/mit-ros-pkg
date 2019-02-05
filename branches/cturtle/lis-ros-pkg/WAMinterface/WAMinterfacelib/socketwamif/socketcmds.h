//establish a connection to the server
int socket_client_connect(char *server_ip, unsigned short int server_port);

//close the connection to the server
int socket_client_close(int sockid);

//read a char (returns 0 if error, else 1)
int read_char(int sockid, char *data);

//Read an integer from the socket (returns 0 if error, else 1)    
//the terminating separation character (' ' or '\0') is put in separator 
int read_int(int sockid, int *num, char *separator);

//Read a double from the socket (returns 0 if error, else 1)    
//the terminating separation character (' ' or '\0' or '\n') is put in separator    
int read_double(int sockid, double *num, char *separator);

//read a list of ints from the socket (returns 0 if error, else 1)
//the final terminating separation character (' ' or '\0' or '\n') is put in separator  
int read_int_array(int sockid, int *array, int count, char *separator);

//read a list of doubles from the socket
int read_double_array(int sockid, double *array, int count, char *separator);

//read in a string terminated by '\n' or '\0' (returns -1 if error, else length of string)
int read_string(int sockid, char *buffer, char *separator);

//write a list of doubles to the socket, ' ' separated and '\n' terminated 
//(returns 0 if error, else 1)
int write_double_array(int sockid, double *array, int count);

//write a single int to the socket, terminated by terminationchar
//(returns 0 if error, else 1)
int write_int(int sockid, int data, char terminationchar);

//write an array of ints to the socket, ' ' separated and '\n' terminated
//(returns 0 if error, else 1)
int write_int_array(int sockid, int *array, int count);

//read a block of size blocklen from the serial port (1-second timeout)
//returns -1 if error, else length read
int read_block(int sockid, char *buffer, int blocklen);

