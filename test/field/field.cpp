#include "field.hpp"

Field::Field(const float &_width, const float &_height, const float &goal_h_p, const float &goal_w_p, const float &corner_h_p){
    width  = _width;  //main area's width
    height = _height; //main area's height
    goal_height_percentage = goal_h_p; //percentage of height that corresponds to goal area
    goal_width_percentage  = goal_w_p; //percentage of width that corresponds to goal area
    corner_height_percentage = corner_h_p; //percentage of height that corresponds to triangular corners
    calculate_points();
}
void Field::calculate_points(){
    float mid_width = width / 2.0f;
    float mid_height = height / 2.0f;
    float corner_offset = height * corner_height_percentage;
    float goal_height_offset = (height * goal_height_percentage) / 2.0f;
    float goal_width_offset = (width * goal_width_percentage);

    main_area = std::vector <std::pair<float, float>>(4);

    main_area[0] = {-mid_width, mid_height};
    main_area[1] = {mid_width, mid_height};
    main_area[2] = {mid_width, -mid_height};
    main_area[3] = {-mid_width, -mid_height};

    box2D_borders = std::vector <std::pair<float, float>>(16);

    box2D_borders[0] = {-mid_width, mid_height-corner_offset};
    box2D_borders[1] = {-mid_width+corner_offset, mid_height};
    box2D_borders[2] = {mid_width-corner_offset, mid_height};
    box2D_borders[3] = {mid_width, mid_height-corner_offset};
    box2D_borders[4] = {mid_width, goal_height_offset};
    box2D_borders[5] = {mid_width+goal_width_offset, goal_height_offset};
    box2D_borders[6] = {mid_width+goal_width_offset, -goal_height_offset};
    box2D_borders[7] = {mid_width, -goal_height_offset};
    box2D_borders[8] = {mid_width, -mid_height+corner_offset};
    box2D_borders[9] = {mid_width-corner_offset, -mid_height};
    box2D_borders[10] = {-mid_width+corner_offset, -mid_height};
    box2D_borders[11] = {-mid_width, -mid_height+corner_offset};
    box2D_borders[12] = {-mid_width, -goal_height_offset};
    box2D_borders[13] = {-mid_width-goal_width_offset, -goal_height_offset};
    box2D_borders[14] = {-mid_width-goal_width_offset, goal_height_offset};
    box2D_borders[15] = {-mid_width, goal_height_offset};
}
std::vector <std::pair<float, float>> Field::get_box2D_borders(){
    return box2D_borders;
}
std::vector <std::pair<float, float>> Field::get_main_area(){
    return main_area;
}