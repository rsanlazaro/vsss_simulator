class RobotKinematicModel{

  float wheel_radius;
  float lenght_between_wheels;
  float left_velocity, right_velocity;
  
  RobotKinematicModel(float radius, float lenght){
    this.wheel_radius          = radius;
    this.lenght_between_wheels = lenght;
  }
  void setSpeed(float linear_velocity, float angular_velocity){
    //Inverse kinematics
    left_velocity  = (2.0 * linear_velocity - angular_velocity * lenght_between_wheels) / (2.0 * wheel_radius); 
    right_velocity = (2.0 * linear_velocity + angular_velocity * lenght_between_wheels) / (2.0 * wheel_radius); 
  }
  void setDifferentialSpeed(float left_velocity, float right_velocity){
    this.left_velocity  = left_velocity;
    this.right_velocity = right_velocity;
  }
  void _print(){
    println("Wheel radius: " + wheel_radius);
    println("Length between wheels: " + lenght_between_wheels);
  }
}
