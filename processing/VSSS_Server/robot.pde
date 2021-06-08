class Robot{
  Point position;
  float angle;
  float side_length;
  int team, role;
  color team_color, role_color;
  Robot(){}
  Robot(float side_length, int team, int role){
    this.side_length = side_length;
    this.team        = team;
    this.role        = role;
    if(team == 1){
      team_color = team_1_color;
    } else{
      team_color = team_2_color;
    }
    if(role == 1){
      role_color = goalkeeper_color;
    } else if(role == 2){
      role_color = midfield_color;
    } else{
      role_color = striker_color;
    }
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
    
    // Create robot body
    fill(team_color);
    rect(0, 0, side_length, side_length);
    
    // Create role circle   
    fill(role_color);
    circle(side_length/4., side_length/4., side_length/6.);
    circle(-side_length/4., side_length/4., side_length/6.);
    circle(-side_length/4., -side_length/4., side_length/6.);
    
    popMatrix();
  }
  void _print(){
    print("position: ");
    position._print();
    println("angle: " + angle);
  }
};
