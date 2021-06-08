
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#define _USE_MATH_DEFINES

using namespace std;

//Auxiliary objects
#include "field/field.hpp"
#include "server/server.hpp"
#include "robot/robot.hpp"
#include "PID/PID.hpp"


//Box2D
#include "../box2d/include/box2d/box2d.h"


void main(){
   // PID();
    fstream file;
    auto start = std::chrono::steady_clock::now();
    PID control = PID();
   // ofstream file("data.txt", ios::app);
    //Scale factors for box2d objects to fit between 0.1 and 10 m
    float scale_factor = 5;

    //Create box2d world
    b2Vec2 gravity(0.0f, 0.0f);
    b2World world(gravity);

    //Field declaration
    Field field = Field(1.5f * scale_factor, 1.3f * scale_factor, 40.f/130.f, 10.f/150.f, 
                    7.f/130.f, 70.f/130.f, 15.f/150.f, 40.f/130.f, 20.f/130.f, 5.f/150.f);
    
    //Create chain of points that represent the field's borders
    const int field_points = 16;
    assert(field_points == field.box2D_borders.size());
    b2Vec2 vs[field_points];
    for(int i = 0; i < field.box2D_borders.size(); ++i){
        vs[i].Set(field.box2D_borders[i].first, field.box2D_borders[i].second);
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

    /*
    b2BodyDef groundBodyDef;
    groundBodyDef.position.Set(0.0f, -10.0f);
    
    b2Body* groundBody = world.CreateBody(&groundBodyDef);
    b2PolygonShape groundBox;
    groundBox.SetAsBox(50.0f, 10.0f);

    groundBody->CreateFixture(&groundBox, 0.0f);
    */

    //Ball Shape
    b2CircleShape circle;
    //circle.m_p.Set(0.0f, 0.0f);
    circle.m_radius = 0.02135f * scale_factor;

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
    fixtureDef.density = 0.25f;
    fixtureDef.friction = 0.0f;
    fixtureDef.restitution = 0.7f;
    fixtureDef.restitutionThreshold = 0.1f;  // Fixes magnetic behavior with slow velocity collisions
    body->CreateFixture(&fixtureDef);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //------------------------------------------------ Robots ---------------------------------------------------------------//
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    float side_length           = 0.075f * scale_factor;
    float density               = 5.0f;
    float friction              = 0.1f;
    float restitution           = 0.01f;
    float restitutionThreshold  = 0.1f;
    float linearDamping         = 20.0f;
    float angularDamping        = 20.0f;

    int number_of_robots = 6;

    vector <b2Vec2> position(number_of_robots);
    vector <float>  angle(number_of_robots);
    vector <Robot>  robots(number_of_robots);
    vector <pair<float, float>> forces(number_of_robots);

    position[0].Set(-0.8f, 0.5f);
    angle[0] = 0.0f;
    position[1].Set(0.8f, 0.5f);
    angle[1] = 0.0f;
    position[2].Set(-0.8f, -0.5f);
    angle[2] = 0.0f;
    position[3].Set(0.8f, -0.5f);
    angle[3] = 0.0f;
    position[4].Set(1.5f, 0.5f);
    angle[4] = 0.0f;
    position[5].Set(-1.5f, 0.5f);
    angle[5] = 0.0f;

    for(int i = 0; i < robots.size(); ++i){
        robots[i] = Robot(position[i], angle[i], side_length, density, friction, restitution, 
        restitutionThreshold, linearDamping, angularDamping, &world);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //------------------------------------------------ Server ---------------------------------------------------------------//
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    float timeStep = 1.0f / 60.0f;
    /*
    int32 velocityIterations = 6;
    int32 positionIterations = 2;
    */
    int32 velocityIterations = 12;
    int32 positionIterations = 10;

    char msg[DEFAULT_BUFLEN];
    char aux[DEFAULT_BUFLEN];
    Server server = Server();
    if(server.setup_server()){
        printf("Server setup failed...");
        return;
    }
    // Receive until the peer shuts down the connection
    do {
        
        server.read_message();
        if (server.get_send_result() > 0) {
            memset(aux, 0, DEFAULT_BUFLEN);
            //printf("Bytes received: %d\n", server.get_send_result());

            char *data = server.get_message_data();
            //printf("data received: %s\n", data);
            if(data[0] == 's'){
               // printf("Sending world state...\n");

                b2Vec2 position = body->GetPosition();
                float angle = body->GetAngle();

                msg[0] = '\0';
                //Ball position
                sprintf_s(aux,"%.2f %.2f ", position.x, position.y);
                strcat_s(msg, aux);

                //Robots positions and angles
                for(int i = 0; i < robots.size(); ++i){
                    b2Body* robotBody = robots[i].get_body_ptr();
                    b2Vec2 position   = robotBody->GetPosition();
                    float angle       = robotBody->GetAngle();
                    sprintf_s(aux, "%.2f %.2f %.2f ", position.x, position.y, angle);
                    cout << position.x << "\t" << position.y << "\n";
                    strcat_s(msg, aux);
                }
                printf("\n");
                strcat_s(msg, "\n");
                
          
                //printf(msg);
                server.send_message(msg);
                //printf("Bytes sent: %d\n", server.get_send_result());

                b2Body* robotBody;
                float robotAngle, motorAngle;

                //Apply forces
                for (int i = 0; i < robots.size(); ++i){
                    robotBody  = robots[i].get_body_ptr();
                    robotAngle = robotBody->GetAngle();
                    b2Vec2 l_speed = robotBody->GetLinearVelocity();
                    float w_speed = robotBody->GetAngularVelocity();

                    float RPS_L = (2*(sqrt(pow(l_speed.x, 2) + pow(l_speed.y, 2))) - w_speed * 0.4f)  / (2 * 0.1f * 2 * M_PI);
                    float RPS_R = (2 * (sqrt(pow(l_speed.x, 2) + pow(l_speed.y, 2))) + w_speed * 0.4f) / (2 * 0.1f * 2 * M_PI);
                    float Desired_RPS_L = -2.0f;
                    float Desired_RPS_R = 2.0f;
                    
                    float Y_PID_L = control.PID_UD(Desired_RPS_L, RPS_L);
                    float Y_PID_R = control.PID_UD(Desired_RPS_R, RPS_R);
                    cout <<"pid  L = "<<Y_PID_L << "     pid  R = " << Y_PID_R << "\n";
                    
                    motorAngle = robotAngle + (float)M_PI / 2.f;

                    b2Vec2 leftMotorPos  = robotBody->GetWorldPoint(robots[i].get_left_motor_position());  //Get left  motor's position in world coord
                    b2Vec2 rightMotorPos = robotBody->GetWorldPoint(robots[i].get_right_motor_position()); //Get right motor's position in world coord

                    b2Vec2 leftMotorForce (Y_PID_L  * cos(motorAngle), Y_PID_L * sin(motorAngle));
                    b2Vec2 rightMotorForce(Y_PID_R * cos(motorAngle), Y_PID_R * sin(motorAngle));

                    robotBody->ApplyForce(leftMotorForce,  leftMotorPos,  true);
                    robotBody->ApplyForce(rightMotorForce, rightMotorPos, true);
                }

                //Simulate 1 step
                world.Step(timeStep, velocityIterations, positionIterations);

                //Debug
                /*
                for(int i = 0; i < robots.size(); ++i){
                    b2Vec2 position = robots[i].get_position();
                    float angle     = robots[i].get_angle();
                    printf("\trobot %d:\n", i);
                    printf("\t\tposition: %.4f %.4f\n", position.x, position.y);
                    printf("\t\tangle: %.4f\n", angle);
                }
                */


            } else if(data[0] == 'f'){
                printf("Sending field description...\n");
                msg[0] = '\0';
                for(int i = 0; i < field.box2D_borders.size(); ++i){
                    sprintf_s(aux,"%.2f %.2f ", field.box2D_borders[i].first, field.box2D_borders[i].second);
                    strcat_s(msg, aux);
                } 
                for(int i = 0; i < field.main_area.size(); ++i){
                    sprintf_s(aux,"%.2f %.2f ", field.main_area[i].first, field.main_area[i].second);
                    strcat_s(msg, aux);
                } 
                for(int i = 0; i < field.small_area_left.size(); ++i){
                    sprintf_s(aux,"%.2f %.2f ", field.small_area_left[i].first, field.small_area_left[i].second);
                    strcat_s(msg, aux);
                } 
                for(int i = 0; i < field.small_area_right.size(); ++i){
                    sprintf_s(aux,"%.2f %.2f ", field.small_area_right[i].first, field.small_area_right[i].second);
                    strcat_s(msg, aux);
                } 
                for(int i = 0; i < field.mid_line.size(); ++i){
                    sprintf_s(aux,"%.2f %.2f ", field.mid_line[i].first, field.mid_line[i].second);
                    strcat_s(msg, aux);
                } 
                //Mid circle
                sprintf_s(aux,"%.2f %.2f %.2f ", get<0>(field.mid_circle), get<1>(field.mid_circle), get<2>(field.mid_circle));
                strcat_s(msg, aux);
                //Small circle left
                sprintf_s(aux,"%.2f %.2f %.2f ", get<0>(field.small_circle_left), get<1>(field.small_circle_left), get<2>(field.small_circle_left));
                strcat_s(msg, aux);
                //Small circle right
                sprintf_s(aux,"%.2f %.2f %.2f ", get<0>(field.small_circle_right), get<1>(field.small_circle_right), get<2>(field.small_circle_right));
                strcat_s(msg, aux);

                strcat_s(msg, "\n");
                printf(msg); 
                server.send_message(msg);
                printf("Bytes sent: %d\n", server.get_send_result());
            } else if(data[0] == 'b'){
                printf("Sending ball description...\n");
                sprintf_s(msg,"%.4f\n", circle.m_radius);
                printf(msg);
                server.send_message(msg);
                printf("Bytes sent: %d\n", server.get_send_result());
            } else if(data[0] == 'r'){
                printf("Sending robot definition...\n");
                sprintf_s(msg,"%.4f %zd\n", side_length, robots.size());
                printf(msg);
                server.send_message(msg);
                printf("Bytes sent: %d\n", server.get_send_result());


            } else if (data[0] == 'a') {
                printf("\n");
                printf("Recieving forces definition...\n");
                memcpy(aux, &data[2], strlen(data)-2);
                float force_m1;
                float force_m2;
                int idx;
                printf("aux: %s\n", aux);
                sscanf_s(aux, "%f %f %d", &force_m1, &force_m2, &idx);
                printf("\nForce 1: %.2f Force 2: %.2f idx: %d\n\n", force_m1, force_m2, idx);
                forces[idx] = {force_m1, force_m2};
                msg[0] = 'K';
                msg[1] = '\n';
                msg[2] = '\0';
                printf(msg);
                server.send_message(msg);
                printf("Bytes sent: %d\n", server.get_send_result());
            }
        }
        else if (server.get_send_result() == 0)
            printf("Connection closing...\n");
        else  {
            server.close_socket_with_error();
            return;
        }
    } while (server.get_send_result() > 0);

    if(server.shutdown_server()){
        printf("Server shutdown failed...");
    }
    return;
}