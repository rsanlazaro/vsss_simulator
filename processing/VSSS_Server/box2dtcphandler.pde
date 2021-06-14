class Box2DTCPHandler{
  
  Client c;
  String input;
  float[] data;
  Box2DTransform box2dtransform;
  
  Box2DTCPHandler(PApplet app, String host, int port){
    // Connect to the server's IP address and port
    c = new Client(app, host, port);
    Point center   = new Point(width / 2.0, height / 2.0);
    box2dtransform = new Box2DTransform(meterToPixel, center);
  }
  
  void request_field_description(){
    c.write("f\n\0");
    // Receive data from server
    while(c.available() == 0);
    input = c.readString();
    input = input.substring(0, input.indexOf("\n"));
    data = float(split(input, ' '));
    field = new Field(data, box2dtransform);
    if(paused){
      //println("Requesting field description...");
      //println(input);
    }
  }
  
  void request_ball_description(){
    c.write("b\n\0");
    // Receive data from server
    while(c.available() == 0);
    input = c.readString();
    input = input.substring(0, input.indexOf("\n"));
    data = float(split(input, ' '));
    ball = new Ball(box2dtransform.transform_scalar(data[0]), ball_color);
    if(paused){
      //println("Requesting ball description...");
      //println(input);
      //println("Radius: " + ball.radius);
    }
  }
  
  void request_robot_description(){
    c.write("r\n\0");
    // Receive data from server
    while(c.available() == 0);
    input = c.readString();
    input = input.substring(0, input.indexOf("\n"));
    data = float(split(input, ' '));
    float side_length = box2dtransform.transform_scalar(data[0]);
    int number_of_robots = (int)data[1];
    //Define robot kinematic model with side_length in meters and wheel radius
    model  = new RobotKinematicModel(wheel_radius, data[0]);
    //Initialize robots array
    robots = new Robot[number_of_robots];
    for(int i = 0; i < robots.length; ++i){
      robots[i] = new Robot(side_length, robot_teams[i], robot_roles[i]);
    }
    if(paused){
      //println("Requesting robot description...");
      //println(input);
      //println("Side length: " + side_length);
      model._print();
    }
  }

  void request_world_state(){
    c.write("s\n\0");
    // Receive data from server
    while(c.available() == 0);
    input = c.readString();
    input = input.substring(0, input.indexOf("\n"));
    //println(input);
    data = float(split(input, ' '));
    if(data.length < 20){
      return;
    }
    ball.set_position(box2dtransform.transform_point(new Point(data[0], data[1])));
    for(int i = 0; i < robots.length; ++i){
      robots[i].set_position(box2dtransform.transform_point(new Point(data[i*3+2], data[i*3+3])));
      robots[i].set_angle(data[i*3+4]);
    }
    if(paused){
      //println("Requesting world state...");
      //println(input);
      ball._print();
      for(int i = 0; i < robots.length; ++i){
        robots[i]._print();
      }
    }
  }
  
  void send_velocities(RobotKinematicModel model, int idx){
    if(paused){
      //println("Setting velocities... VL: " + nf(model.left_velocity,0,3) + " VR: " + nf(model.right_velocity,0,3) + " idx: " + idx);
    }
    c.write("a " + model.left_velocity + " " + model.right_velocity + " " + idx +"\n\0");
    // Receive data from server
    while(c.available() == 0);
    input = c.readString();
    input = input.substring(0, input.indexOf("\n"));
  }

};
