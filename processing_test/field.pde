class Field{
  Point[] box2D_borders;
  Point[] main_area;
  Point[] small_area_left;
  Point[] small_area_right;
  Point[] mid_line;
  Point mid_circle_pos;
  float mid_circle_r;
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
    mid_circle_pos = box2dtransform.transform_point(new Point(data[60], data[61]));
    mid_circle_r   = box2dtransform.transform_scalar(data[62]);
  }
  void draw_shape(Point[] shape){
    beginShape();
    for(int i = 0; i < shape.length; ++i){
      vertex(shape[i].x, shape[i].y);
    }
    endShape(CLOSE);
  }
  void _draw(){
    noFill();
    draw_shape(box2D_borders);
    draw_shape(main_area);
    draw_shape(small_area_left);
    draw_shape(small_area_right);
    draw_shape(mid_line);
    circle(mid_circle_pos.x, mid_circle_pos.y, mid_circle_r);
  }
};
