class Field{
  Point[] box2D_borders;
  Point[] main_area;
  Point[] small_area_left;
  Point[] small_area_right;
  Point[] mid_line;
  Point mid_circle_pos, small_circle_l_pos, small_circle_r_pos;
  float mid_circle_d, small_circle_l_d, small_circle_r_d;
  Field(float data[], Box2DTransform box2dtransform){
    box2D_borders    = new Point[16];
    main_area        = new Point[4];
    small_area_left  = new Point[4];
    small_area_right = new Point[4];
    mid_line         = new Point[2];
    for(int i = 0; i < 16; ++i){
      box2D_borders[i] = box2dtransform.transform_point(new Point(data[i*2], data[i*2+1]));
    }
    for(int i = 16; i < 20; ++i){
      main_area[i-16] = box2dtransform.transform_point(new Point(data[i*2], data[i*2+1]));
    }
    for(int i = 20; i < 24; ++i){
      small_area_left[i-20] = box2dtransform.transform_point(new Point(data[i*2], data[i*2+1]));
    }
    for(int i = 24; i < 28; ++i){
      small_area_right[i-24] = box2dtransform.transform_point(new Point(data[i*2], data[i*2+1]));
    }
    for(int i = 28; i < 30; ++i){
      mid_line[i-28] = box2dtransform.transform_point(new Point(data[i*2], data[i*2+1]));
    }
    mid_circle_pos     = box2dtransform.transform_point(new Point(data[60], data[61]));
    mid_circle_d       = box2dtransform.transform_scalar(data[62]);
    small_circle_l_pos = box2dtransform.transform_point(new Point(data[63], data[64]));
    small_circle_l_d   = box2dtransform.transform_scalar(data[65]);
    small_circle_r_pos = box2dtransform.transform_point(new Point(data[66], data[67]));
    small_circle_r_d   = box2dtransform.transform_scalar(data[68]);
  }
  void draw_shape(Point[] shape){
    beginShape();
    for(int i = 0; i < shape.length; ++i){
      vertex(shape[i].x, shape[i].y);
    }
    endShape(CLOSE);
  }
  void _draw(int background_color){
    stroke(255);
    strokeWeight(3);
    noFill();
    draw_shape(box2D_borders);
    draw_shape(main_area);
    circle(mid_circle_pos.x, mid_circle_pos.y, mid_circle_d);
    arc(small_circle_l_pos.x, small_circle_l_pos.y, small_circle_l_d, small_circle_l_d, 3.*PI/2., 5.*PI/2.);
    arc(small_circle_r_pos.x, small_circle_r_pos.y, small_circle_r_d, small_circle_r_d, PI/2., 3.*PI/2.);
    fill(background_color);
    draw_shape(small_area_left);
    draw_shape(small_area_right);
    draw_shape(mid_line);
    noFill();
  }
};
