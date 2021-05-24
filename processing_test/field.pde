class Field{
  Point[] box2D_borders;
  Point[] main_area;
  Field(float data[], Box2DTransform box2dtransform){
    box2D_borders = new Point[16];
    main_area = new Point[4];
    for(int i = 0; i < 16; ++i){
      box2D_borders[i] = box2dtransform.transform_point(new Point(data[i*2], data[i*2+1]));
      box2D_borders[i]._print();
    }
    for(int i = 16; i < 20; ++i){
      main_area[i-16] = box2dtransform.transform_point(new Point(data[i*2], data[i*2+1]));
      main_area[i-16]._print();
    }
  }
  void _draw(){
    noFill();
    beginShape();
    for(int i = 0; i < 16; ++i){
      vertex(box2D_borders[i].x, box2D_borders[i].y);
    }
    endShape(CLOSE);
    beginShape();
    for(int i = 0; i < 4; ++i){
      vertex(main_area[i].x, main_area[i].y);
    }
    endShape(CLOSE);
  }
};
