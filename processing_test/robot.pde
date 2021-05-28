class Robot{
  Point position;
  float angle;
  float side_length;
  int team;
  Robot(){}
  Robot(float side_length, int team){
    this.side_length = side_length;
    this.team        = team;
  }
  void set_position(Point position){
    this.position = position;
  }
  void set_angle(float angle){
    this.angle = angle;
  }
  void _draw(){
    rectMode(CENTER);
    pushMatrix();
    translate(position.x, position.y);
    rotate(-angle);
    if(team == 1){
      fill(team_1_color);
    } else{
      fill(team_2_color);
    }
    rect(0, 0, side_length, side_length);
    popMatrix();
  }
  void _print(){
    print("position: ");
    position._print();
    println("angle: " + angle);
  }
};
