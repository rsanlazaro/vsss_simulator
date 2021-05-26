class Robot{
  Point position;
  float angle;
  float size;
  Robot(Point position, float angle, float size){
    this.position = position;
    this.angle    = angle;
    this.size     = size;
  }
  void _draw(){
    position._print();
    println(-angle);
    println(size);
    pushMatrix();
    translate(position.x, position.y);
    rotate(-angle);
    rect(-size/2., -size/2., size, size);
    popMatrix();
  }
};
