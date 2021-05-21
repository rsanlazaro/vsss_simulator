#include "field.hpp"

Field::Field(const float &_width, const float &_height, const float &goal_h_p, const float &goal_w_p, const float &corner_h_p){
    width  = _width;
    height = _height;
    goal_height_percentage = goal_h_p;
    goal_width_percentage  = goal_w_p;
    corner_height_percentage = corner_h_p;
}

std::vector <std::pair<float, float>> Field::get_point_list(){
    std::vector <std::pair<float, float>> plist(16); 
    float mid_width = width / 2.0f;
    float mid_height = height / 2.0f;
    float corner_offset = height * corner_height_percentage;
    float goal_height_offset = (height * goal_height_percentage) / 2.0f;
    float goal_width_offset = (width * goal_width_percentage);

    plist[0] = {-mid_width, mid_height-corner_offset};
    plist[1] = {-mid_width+corner_offset, mid_height};
    plist[2] = {mid_width-corner_offset, mid_height};
    plist[3] = {mid_width, mid_height-corner_offset};
    plist[4] = {mid_width, goal_height_offset};
    plist[5] = {mid_width+goal_width_offset, goal_height_offset};
    plist[6] = {mid_width+goal_width_offset, -goal_height_offset};
    plist[7] = {mid_width, -goal_height_offset};
    plist[8] = {mid_width, -mid_height+corner_offset};
    plist[9] = {mid_width-corner_offset, -mid_height};
    plist[10] = {-mid_width+corner_offset, -mid_height};
    plist[11] = {-mid_width, -mid_height+corner_offset};
    plist[12] = {-mid_width, -goal_height_offset};
    plist[13] = {-mid_width-goal_width_offset, -goal_height_offset};
    plist[14] = {-mid_width-goal_width_offset, goal_height_offset};
    plist[15] = {-mid_width, goal_height_offset};
    return plist;
}