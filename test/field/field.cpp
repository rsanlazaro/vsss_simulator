#include "field.hpp"

Field::Field(const float &_width, const float &_height, const float &goal_h_p, const float &goal_w_p, const float &corner_h_p,
const float &small_area_h_p, const float &small_area_w_p, const float &mid_circle_h_p, const float &small_circle_h_p, const float &small_circle_w_p){
    width  = _width;                                    //main area's width
    height = _height;                                   //main area's height
    goal_height_percentage   = goal_h_p;                //percentage of height that corresponds to goal area
    goal_width_percentage    = goal_w_p;                //percentage of width that corresponds to goal area
    corner_height_percentage = corner_h_p;              //percentage of height that corresponds to triangular corners
    small_area_height_percentage = small_area_h_p;      //percentage of height that corresponds to small area
    small_area_width_percentage  = small_area_w_p;      //percentage of width that corresponds to small area
    mid_circle_height_percentage = mid_circle_h_p;      //percentge of height that corresponds to middle circle's diameter
    small_circle_height_percentage = small_circle_h_p;  //percentage of height that corresponds to small circle's arc base
    small_circle_width_percentage = small_circle_w_p;   //percentage of height that corresponds to small circle's arc height
    calculate_points();
}
void Field::calculate_points(){
    float mid_width = width / 2.0f;
    float mid_height = height / 2.0f;
    float corner_offset = height * corner_height_percentage;
    float goal_height_offset = (height * goal_height_percentage) / 2.0f;
    float goal_width_offset = (width * goal_width_percentage);
    float small_area_height_offset = (height * small_area_height_percentage) / 2.0f;
    float small_area_width_offset = (width * small_area_width_percentage);

    //small circle radius and angle calculation
    float arc_base   = height * small_circle_height_percentage;
    float arc_height = width * small_circle_width_percentage;
    float diameter   = 2.f * ((4.f*arc_height*arc_height + arc_base*arc_base) / (8.f*arc_height));

    main_area = std::vector <std::pair<float, float>>(4);

    main_area[0] = {-mid_width, mid_height};
    main_area[1] = {mid_width, mid_height};
    main_area[2] = {mid_width, -mid_height};
    main_area[3] = {-mid_width, -mid_height};

    mid_line = std::vector <std::pair<float, float>>(2);

    mid_line[0] = {0.f, mid_height};
    mid_line[1] = {0.f, -mid_height};

    mid_circle = std::tuple <float, float, float>(0.f, 0.f, height * mid_circle_height_percentage);

    small_area_left = std::vector <std::pair<float, float>>(4);

    small_area_left[0] = {-mid_width, small_area_height_offset};
    small_area_left[1] = {-mid_width+small_area_width_offset, small_area_height_offset};
    small_area_left[2] = {-mid_width+small_area_width_offset, -small_area_height_offset};
    small_area_left[3] = {-mid_width, -small_area_height_offset};

    small_circle_left = std::tuple <float, float, float>(-mid_width+small_area_width_offset/2.f, 0.f, diameter);

    small_area_right = std::vector <std::pair<float, float>>(4);

    small_area_right[0] = {mid_width-small_area_width_offset, small_area_height_offset};
    small_area_right[1] = {mid_width, small_area_height_offset};
    small_area_right[2] = {mid_width, -small_area_height_offset};
    small_area_right[3] = {mid_width-small_area_width_offset, -small_area_height_offset};

    small_circle_right = std::tuple <float, float, float>(mid_width-small_area_width_offset/2.f, 0.f, diameter);

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

