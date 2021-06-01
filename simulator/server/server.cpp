#include "server.hpp"

int Server::initialize_winsock(){
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return 1;
    }

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;
    return 0;
}

int Server::resolve_server_address_and_port(){
    // Resolve the server address and port
    iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
    if ( iResult != 0 ) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return 1;
    }
    return 0;
}

int Server::create_socket(){
    // Create a SOCKET for connecting to server
    ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (ListenSocket == INVALID_SOCKET) {
        printf("socket failed with error: %ld\n", WSAGetLastError());
        freeaddrinfo(result);
        WSACleanup();
        return 1;
    }
    return 0;
}

int Server::bind_socket(){
    // Setup the TCP listening socket
    iResult = bind( ListenSocket, result->ai_addr, (int)result->ai_addrlen);
    if (iResult == SOCKET_ERROR) {
        printf("bind failed with error: %d\n", WSAGetLastError());
        freeaddrinfo(result);
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }
    freeaddrinfo(result);
    return 0;
}

int Server::listen_socket(){
    iResult = listen(ListenSocket, SOMAXCONN);
    if (iResult == SOCKET_ERROR) {
        printf("listen failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }
    return 0;
}

int Server::accept_socket(){
    // Accept a client socket
    ClientSocket = accept(ListenSocket, NULL, NULL);
    if (ClientSocket == INVALID_SOCKET) {
        printf("accept failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    // No longer need server socket
    closesocket(ListenSocket);
    return 0;
}

Server::Server(){
    ListenSocket = INVALID_SOCKET;
    ClientSocket = INVALID_SOCKET;
    result = NULL;
    recvbuflen = DEFAULT_BUFLEN;
}

int Server::get_send_result(){
    return iSendResult;
}

void Server::set_send_result(int result){
    iSendResult = result;
}

char* Server::get_message_data(){
    return recvbuf;
}
    
int Server::setup_server(){
    if(initialize_winsock()) return 1;
    if(resolve_server_address_and_port()) return 1;
    if(create_socket()) return 1;
    if(bind_socket()) return 1;
    if(listen_socket()) return 1;
    if(accept_socket()) return 1;
    return 0;
}

int Server::shutdown_server(){
    // shutdown the connection since we're done
    iResult = shutdown(ClientSocket, SD_SEND);
    if (iResult == SOCKET_ERROR) {
        printf("shutdown failed with error: %d\n", WSAGetLastError());
        closesocket(ClientSocket);
        WSACleanup();
        return 1;
    }

    // cleanup
    closesocket(ClientSocket);
    WSACleanup();
    return 0;
}

void Server::close_socket_with_error(){
    printf("recv failed with error: %d\n", WSAGetLastError());
    closesocket(ClientSocket);
    WSACleanup();
}

int Server::send_message(char *msg){
    iSendResult = send( ClientSocket, msg, (int)strlen(msg), 0 );
    if (iSendResult == SOCKET_ERROR) {
        printf("send failed with error: %d\n", WSAGetLastError());
        closesocket(ClientSocket);
        WSACleanup();
        return 1;
    }
    return 0;
}

void Server::read_message(){
    iSendResult = recv(ClientSocket, recvbuf, recvbuflen, 0);
}
