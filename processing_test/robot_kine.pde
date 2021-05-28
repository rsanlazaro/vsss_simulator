class Robot_kine{

  float radius;
  float lenght;
  float VL,VR;

  
  Robot_kine(float radius, float lenght){
  
      this.radius = radius;
      this.lenght = lenght;
      println("Defining robot characteristics  " + radius + " " + lenght);
    }
  void setSpeed(float L_Speed, float A_Speed){
    //Making the inverse kinematics
    VR = (2.0 * L_Speed + A_Speed * lenght) / (2.0 * radius); 
    VL = (2.0 * L_Speed - A_Speed * lenght) / (2.0 * radius); 
  }
  void setDifferentialSpeed(float L_Speed, float R_Speed){
    //Declaring same variables
    VR = R_Speed;
    VL = L_Speed;
  }
}
