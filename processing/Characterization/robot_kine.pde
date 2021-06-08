class RobotKinematicModel{

  float wheel_radius;
  float lenght_between_wheels;
  float left_velocity, right_velocity;
  float linear_velocity, angular_velocity;
  
  RobotKinematicModel(float radius, float lenght){
    this.wheel_radius          = radius;
    this.lenght_between_wheels = lenght;
  }
  void setSpeed(float linear_velocity, float angular_velocity){
    this.linear_velocity  = linear_velocity;
    this.angular_velocity = angular_velocity;
    //Inverse kinematics
    float w_l  = (2.0 * linear_velocity - angular_velocity * lenght_between_wheels) / (2.0 * wheel_radius); 
    float w_r  = (2.0 * linear_velocity + angular_velocity * lenght_between_wheels)  / (2.0 * wheel_radius); 
    left_velocity  = w_l * wheel_radius; 
    right_velocity = w_r * wheel_radius; 
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
