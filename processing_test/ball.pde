class Ball{
  Point position;
  float radius;
  color clr;
  Ball(){}
  Ball(Point position, float radius){
    this.position = position;
    this.radius   = radius;
  }
  void set_position(Point position){
    this.position = position;
  }
  void set_color(color clr){
    this.clr = clr;
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
