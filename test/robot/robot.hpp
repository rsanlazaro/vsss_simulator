#ifndef ROBOT_H
#define ROBOT_H

#include <utility>
#include "../../box2d/include/box2d/box2d.h"

class Robot{
    private:
        b2BodyDef bodyDef;
        b2Body*   body;
        b2PolygonShape dynamicBox;
        b2FixtureDef   fixtureDef;
    public:
        Robot(const b2Vec2 &positionn, const float &angle, const float &size, const float &density, const float &friction,
        const float &restitution, const float &linearDamping, const float &angularDamping, b2World* world);
        b2Vec2 get_position();
        float  get_angle();
        b2PolygonShape get_shape();
};

#endif