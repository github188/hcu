/*
 * init_socket.h
 *
 *  Created on: 2015年12月1日
 *      Author: test
 */

#ifndef WEBSER_INIT_SOCKET_H_
#define WEBSER_INIT_SOCKET_H_

#include <netinet/in.h>

#define BACKLOG    20        /* length of listening queue on socket */
#define PORT    8080    /* web server listening port */


/* initialize the socket on server, include below
   socket();
   bind();
   listen();
*/


/* listen_fd : the web server listen file decriptor
   server_addr: the web server ipv4 address
   RETURNS: success on 0, error on -1
*/
int init_socket(int *listen_fd, struct sockaddr_in *server_addr);



#endif /* WEBSER_INIT_SOCKET_H_ */
