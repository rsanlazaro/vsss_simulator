
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

using namespace std;

//Auxiliary objects
#include "field/field.hpp"

//Box2D
#include "../box2d/include/box2d/box2d.h"

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
    public:
        WSADATA wsaData;
        int iResult;

        SOCKET ListenSocket;
        SOCKET ClientSocket;

        struct addrinfo *result;
        struct addrinfo hints;

        int iSendResult;
        char recvbuf[DEFAULT_BUFLEN];
        int recvbuflen;

        Server(){
            ListenSocket = INVALID_SOCKET;
            ClientSocket = INVALID_SOCKET;
            result = NULL;
            recvbuflen = DEFAULT_BUFLEN;
        }

        int initialize_winsock(){
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
        int resolve_server_address_and_port(){
            // Resolve the server address and port
            iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
            if ( iResult != 0 ) {
                printf("getaddrinfo failed with error: %d\n", iResult);
                WSACleanup();
                return 1;
            }
            return 0;
        }
        int create_socket(){
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
        int bind_socket(){
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
        int listen_socket(){
            iResult = listen(ListenSocket, SOMAXCONN);
            if (iResult == SOCKET_ERROR) {
                printf("listen failed with error: %d\n", WSAGetLastError());
                closesocket(ListenSocket);
                WSACleanup();
                return 1;
            }
            return 0;
        }
        int accept_socket(){
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
        int setup_server(){
            if(initialize_winsock()) return 1;
            if(resolve_server_address_and_port()) return 1;
            if(create_socket()) return 1;
            if(bind_socket()) return 1;
            if(listen_socket()) return 1;
            if(accept_socket()) return 1;
            return 0;
        }
        int shutdown_server(){
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
};


void main(){

    //Field declaration
    Field field = Field(3.0f, 2.5f, 0.33f, 0.1f, 0.05f);
    //Get field points
    vector <pair<float, float>> plist = field.get_point_list();
    
    b2Vec2 gravity(0.0f, 0.0f);
    b2World world(gravity);

    const int field_points = 16;
    assert(field_points == plist.size());
    b2Vec2 vs[field_points];

    for(int i = 0; i < field_points; ++i){
        vs[i].Set(plist[i].first, plist[i].second);
    }
    
    b2ChainShape chain;
    chain.CreateLoop(vs, field_points);

    b2BodyDef groundBodyDef;
    groundBodyDef.type = b2_staticBody;
    groundBodyDef.position.Set(0.0f, 0.0f);

    b2Body* borders = world.CreateBody(&groundBodyDef);

    b2FixtureDef groundFixtureDef;
    groundFixtureDef.shape = &chain;
    //groundFixtureDef.density = 10.0f;
    groundFixtureDef.friction = 0.0f;
    groundFixtureDef.restitution = 1.0f;
    borders->CreateFixture(&groundFixtureDef);

    //Ball Shape
    b2CircleShape circle;
    //circle.m_p.Set(0.0f, 0.0f);
    circle.m_radius = 0.1f;

    //Ball dynamic body
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;

    bodyDef.position.Set(0.0f, 0.0f); // the body's origin position.
    bodyDef.angle = 0.0f; // the body's angle in radians.
    bodyDef.fixedRotation = true;
    bodyDef.linearDamping = 0.1f;
    //bodyDef.angularDamping = 0.01f;

    b2Body* body = world.CreateBody(&bodyDef);
    b2FixtureDef fixtureDef;
    fixtureDef.shape = &circle;
    fixtureDef.density = 1.0f;
    fixtureDef.friction = 0.0f;
    fixtureDef.restitution = 1.0f;
    body->CreateFixture(&fixtureDef);

    b2Vec2 force(5.8f, 5.2f);
    b2Vec2 point(0.0f, 0.0f);
    body->ApplyForce(force, point, true);

    //Robot body
    b2BodyDef robotBodyDef;
    robotBodyDef.type = b2_dynamicBody;
    robotBodyDef.position.Set(0.0f, 4.0f);
    b2Body* robotBody = world.CreateBody(&robotBodyDef);


    b2PolygonShape dynamicBox;
    dynamicBox.SetAsBox(0.2f, 0.2f);

    b2FixtureDef robotFixtureDef;
    robotFixtureDef.shape = &dynamicBox;
    robotFixtureDef.density = 5.0f;
    robotFixtureDef.friction = 0.3f;

    robotBody->CreateFixture(&robotFixtureDef);




    /*
    b2BodyDef groundBodyDef;
    groundBodyDef.position.Set(0.0f, -10.0f);
    
    b2Body* groundBody = world.CreateBody(&groundBodyDef);
    b2PolygonShape groundBox;
    groundBox.SetAsBox(50.0f, 10.0f);

    groundBody->CreateFixture(&groundBox, 0.0f);

    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position.Set(0.0f, 4.0f);
    b2Body* body = world.CreateBody(&bodyDef);


    b2PolygonShape dynamicBox;
    dynamicBox.SetAsBox(1.0f, 1.0f);

    b2FixtureDef fixtureDef;
    fixtureDef.shape = &dynamicBox;
    fixtureDef.density = 1.0f;
    fixtureDef.friction = 0.3f;

    body->CreateFixture(&fixtureDef);
    */

    float timeStep = 1.0f / 60.0f;
    /*
    int32 velocityIterations = 6;
    int32 positionIterations = 2;
    */

    int32 velocityIterations = 10;
    int32 positionIterations = 8;




    /*
    for (int32 i = 0; i < 60; ++i)
    {
        world.Step(timeStep, velocityIterations, positionIterations);
        b2Vec2 position = body->GetPosition();
        float angle = body->GetAngle();
        printf("%4.2f %4.2f %4.2f\n", position.x, position.y, angle);
    }
    */

    char msg[DEFAULT_BUFLEN];
    char aux[DEFAULT_BUFLEN];
    Server server = Server();
    if(server.setup_server()){
        printf("Server setup failed...");
        return;
    }
    // Receive until the peer shuts down the connection
    do {


        server.iResult = recv(server.ClientSocket, server.recvbuf, server.recvbuflen, 0);
        if (server.iResult > 0) {
            printf("Bytes received: %d\n", server.iResult);

            if(server.recvbuf[0] == 's'){
                printf("Sending world state...\n");

                world.Step(timeStep, velocityIterations, positionIterations);
                b2Vec2 position = body->GetPosition();
                float angle = body->GetAngle();
                sprintf_s(msg,"%4.2f %4.2f %4.2f\n", position.x, position.y, angle);
                printf(msg);

                // Echo the buffer back to the sender
                server.iSendResult = send( server.ClientSocket, msg, (int)strlen(msg), 0 );
                if (server.iSendResult == SOCKET_ERROR) {
                    printf("send failed with error: %d\n", WSAGetLastError());
                    closesocket(server.ClientSocket);
                    WSACleanup();
                    return;
                }
                printf("Bytes sent: %d\n", server.iSendResult);
            } else if(server.recvbuf[0] == 'f'){
                printf("Sending field coordinates...\n");

                msg[0] = '\0';
                for(int i = 0; i < plist.size(); ++i){
                    sprintf_s(aux,"%4.2f %4.2f ", plist[i].first, plist[i].second);
                    strcat_s(msg, aux);
                } strcat_s(msg, "\n");
                printf(msg); 

                // Echo the buffer back to the sender
                server.iSendResult = send( server.ClientSocket, msg, (int)strlen(msg), 0 );
                if (server.iSendResult == SOCKET_ERROR) {
                    printf("send failed with error: %d\n", WSAGetLastError());
                    closesocket(server.ClientSocket);
                    WSACleanup();
                    return;
                }
                printf("Bytes sent: %d\n", server.iSendResult);
            } else if(server.recvbuf[0] == 'b'){
                printf("Sending ball definition...\n");

                b2Vec2 position = body->GetPosition();
                sprintf_s(msg,"%4.2f %4.2f %4.2f\n", circle.m_radius, position.x, position.y);
                printf(msg);

                // Echo the buffer back to the sender
                server.iSendResult = send( server.ClientSocket, msg, (int)strlen(msg), 0 );
                if (server.iSendResult == SOCKET_ERROR) {
                    printf("send failed with error: %d\n", WSAGetLastError());
                    closesocket(server.ClientSocket);
                    WSACleanup();
                    return;
                }
                printf("Bytes sent: %d\n", server.iSendResult);
            }
        }
        else if (server.iResult == 0)
            printf("Connection closing...\n");
        else  {
            printf("recv failed with error: %d\n", WSAGetLastError());
            closesocket(server.ClientSocket);
            WSACleanup();
            return;
        }

    } while (server.iResult > 0);

    if(server.shutdown_server()){
        printf("Server shutdown failed...");
    }

    return;
}