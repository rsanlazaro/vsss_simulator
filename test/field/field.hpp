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
        float small_area_height_percentage;
        float small_area_width_percentage;
        void calculate_points();
    public:
        std::vector <std::pair<float, float>> box2D_borders;
        std::vector <std::pair<float, float>> main_area;
        std::vector <std::pair<float, float>> small_area_left;
        std::vector <std::pair<float, float>> small_area_right;
        Field(const float &_width, const float &_height, const float &goal_h_p, const float &goal_w_p, const float &corner_h_p,
        const float &small_area_h_p, const float &small_area_w_p);
};

#endif