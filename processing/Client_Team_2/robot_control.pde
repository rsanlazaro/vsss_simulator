class Robot_control{
  int r_id;
  Robot_control(int r_id){
    this.r_id = r_id;
  }
  void set_vel_robot(float Vl, float Va){
    println("sending data... " + r_id + " " + Vl + " " + Va);
    c2.write(r_id + " " + Vl + " " + Va + "\n\0");
  }
};
