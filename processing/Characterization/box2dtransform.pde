class Box2DTransform{

  float meterToPixel;
  Point origin;
  
  Box2DTransform(float meterToPixel, Point origin){
    this.meterToPixel = meterToPixel;
    this.origin = origin;
  }
  float transform_scalar(float scalar){
    return scalar * meterToPixel;
  }
  Point transform_point(Point p){
    return new Point(p.x * meterToPixel + origin.x, -p.y * meterToPixel + origin.y);
  }
};
