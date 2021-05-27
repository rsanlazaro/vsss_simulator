class Robot{
  Point position;
  float angle;
  float size;
  Robot(){}
  Robot(Point position, float angle, float size){
    this.position = position;
    this.angle    = angle;
    this.size     = size;
  }
  void _draw(){
    rectMode(CENTER);
    pushMatrix();
    translate(position.x, position.y);
    rotate(-angle);
    rect(0, 0, size, size);
    popMatrix();
  }
  void _print(){
    print("position: ");
    position._print();
    println("angle: " + angle);
  }
};
