#ifndef FIELD_H
#define FIELD_H

#include <vector>

class Field{
    private:
        float width;
        float height;
        float goal_height_percentage;
        float goal_width_percentage;
        float corner_height_percentage; 

    public:
        Field(const float &_width, const float &_height, const float &goal_h_p, const float &goal_w_p, const float &corner_h_p);
        std::vector <std::pair<float, float>> get_point_list();
};

#endif