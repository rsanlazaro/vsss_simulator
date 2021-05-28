#include "robot.hpp"

Robot::Robot(){}
Robot::Robot(const b2Vec2 &position, const float &angle, const float &size, const float &density, const float &friction,
const float &restitution, const float &linearDamping, const float &angularDamping, b2World* world){

    bodyDef.type = b2_dynamicBody;

    //Initial position and angle
    bodyDef.position = position;
    bodyDef.angle    = angle;

    //Damping
    bodyDef.linearDamping  = linearDamping;
    bodyDef.angularDamping = angularDamping;

    //Add body to world
    body = world->CreateBody(&bodyDef);

    //Set shape as box
    dynamicBox.SetAsBox(size/2.f, size/2.f);

    //Fixture definition
    fixtureDef.shape       = &dynamicBox;
    fixtureDef.density     = density;
    fixtureDef.friction    = friction;
    fixtureDef.restitution = restitution;
    fixtureDef.restitutionThreshold = 0.1f;

    //Add fixture to body
    body->CreateFixture(&fixtureDef);
}
b2Body* Robot::get_body_ptr(){
    return body;
}
b2PolygonShape Robot::get_shape(){
    return dynamicBox;
}
