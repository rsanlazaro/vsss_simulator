class Ball{
  Point position;
  float radius;
  color clr;
  Ball(){}
  Ball(float radius, color clr){
    this.radius   = radius;
    this.clr      = clr;
  }
  void set_position(Point position){
    this.position = position;
  }
  void _print(){
    print("position: ");
    position._print();
  }
  void _draw(){
    fill(clr);
    noStroke();
    circle(position.x, position.y, radius*2.);
  }
};
