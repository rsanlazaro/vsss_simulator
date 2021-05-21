#ifndef SERVER_H
#define SERVER_H

#include <stdio.h>
//TCP Communication
#undef UNICODE
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "27015"

class Server{
    
    private:
        WSADATA wsaData;
        int iResult;

        SOCKET ListenSocket;
        SOCKET ClientSocket;

        struct addrinfo *result;
        struct addrinfo hints;

        int iSendResult;
        char recvbuf[DEFAULT_BUFLEN];
        int recvbuflen;

        int initialize_winsock();
        int resolve_server_address_and_port();
        int create_socket();
        int bind_socket();
        int listen_socket();
        int accept_socket();

    public:
        Server();

        int get_send_result();
        void set_send_result(int result);
        char* get_message_data();

        int setup_server();
        int shutdown_server();
        void close_socket_with_error();
        int send_message(char *msg);
        void read_message();
};
#endif