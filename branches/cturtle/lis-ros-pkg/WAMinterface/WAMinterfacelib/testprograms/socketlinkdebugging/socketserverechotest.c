//tests using a socket server with Xenomai/btclient (SEGFAULTS!!)

#include "waminterface/wamif.h"
#include "log/log.h"

#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <netinet/in.h>

#define MAXPENDING 5    /* Max connection requests */
#define BUFFSIZE 32
void Die(char *mess) { perror(mess); exit(1); }

void HandleClient(int sock) {
	char buffer[BUFFSIZE];
	int received = -1;
	/* Receive message */
	if ((received = recv(sock, buffer, BUFFSIZE, 0)) < 0) {
		Die("Failed to receive initial bytes from client");
	}
	/* Send bytes and check for more incoming data in loop */
	while (received > 0) {
		/* Send back received data */
		if (send(sock, buffer, received, 0) != received) {
			Die("Failed to send bytes to client");
		}
		/* Check for more data */
		if ((received = recv(sock, buffer, BUFFSIZE, 0)) < 0) {
			Die("Failed to receive additional bytes from client");
		}
	}
	close(sock);
}
 
int main(int argc, char *argv[]) {
	/*
	log_open("WAMtest.log");
	WamInterface *wamif;
	wamif = wamif_create(0);
	wamif_activate(wamif);
	*/
	//btwamsq(2.0);

	int serversock, clientsock;
	struct sockaddr_in echoserver, echoclient;

	if (argc != 2) {
		fprintf(stderr, "USAGE: echoserver <port>\n");
		exit(1);
	}
	/* Create the TCP socket */
	if ((serversock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
		Die("Failed to create socket");
	}
	/* Construct the server sockaddr_in structure */
	memset(&echoserver, 0, sizeof(echoserver));       /* Clear struct */
	echoserver.sin_family = AF_INET;                  /* Internet/IP */
	echoserver.sin_addr.s_addr = htonl(INADDR_ANY);   /* Incoming addr */
	echoserver.sin_port = htons(atoi(argv[1]));       /* server port */

 
	/* Bind the server socket */
	if (bind(serversock, (struct sockaddr *) &echoserver,
					 sizeof(echoserver)) < 0) {
		Die("Failed to bind the server socket");
	}
	/* Listen on the server socket */
	if (listen(serversock, MAXPENDING) < 0) {
		Die("Failed to listen on server socket");
	}

 
	/* Run until cancelled */
	while (1) {
		unsigned int clientlen = sizeof(echoclient);
		/* Wait for client connection */
		if ((clientsock =
				 accept(serversock, (struct sockaddr *) &echoclient,
								&clientlen)) < 0) {
			Die("Failed to accept client connection");
		}
		fprintf(stdout, "Client connected: %s\n",
						inet_ntoa(echoclient.sin_addr));
		HandleClient(clientsock);
	}
}
