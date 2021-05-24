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
        std::vector <std::pair<float, float>> box2D_borders;
        std::vector <std::pair<float, float>> main_area;
        void calculate_points();

    public:
        Field(const float &_width, const float &_height, const float &goal_h_p, const float &goal_w_p, const float &corner_h_p);
        std::vector <std::pair<float, float>> get_box2D_borders();
        std::vector <std::pair<float, float>> get_main_area();
};

#endif