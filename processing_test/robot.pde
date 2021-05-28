class Robot{
  Point position;
  float angle;
  float side_length;
  int team;
  color team_clr;
  Robot(){}
  Robot(float side_length){
    this.side_length = side_length;
  }
  void set_position(Point position){
    this.position = position;
  }
  void set_angle(float angle){
    this.angle = angle;
  }
  void set_team(int team, color team_clr){
    this.team = team;
    this.team_clr = team_clr;
  }
  void _draw(){
    rectMode(CENTER);
    pushMatrix();
    translate(position.x, position.y);
    rotate(-angle);
    //fill(team_clr);
    rect(0, 0, side_length, side_length);
    popMatrix();
  }
  void _print(){
    print("position: ");
    position._print();
    println("angle: " + angle);
  }
};
