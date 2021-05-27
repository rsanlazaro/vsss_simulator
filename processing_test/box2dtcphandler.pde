class Box2DTCPHandler{
  
  Client c;
  String input;
  float[] data;
  Box2DTransform box2dtransform;
  
  Box2DTCPHandler(PApplet app, String host, int port, float meterToPixel){
    // Connect to the server's IP address and port
    c = new Client(app, host, port);
    Point center   = new Point(width / 2.0, height / 2.0);
    box2dtransform = new Box2DTransform(meterToPixel, center);
  }
  
  void request_field_description(boolean paused){
    c.write("f\n\0");
    // Receive data from server
    while(c.available() == 0);
    input = c.readString();
    input = input.substring(0, input.indexOf("\n"));
    data = float(split(input, ' '));
    field = new Field(data, box2dtransform);
    if(paused){
      println("Requesting field description...");
      println("Bytes received: " + c.available());
      println(input);
    }
  }
  
  void request_ball_description(boolean paused){
    c.write("b\n\0");
    // Receive data from server
    while(c.available() == 0);
    input = c.readString();
    input = input.substring(0, input.indexOf("\n"));
    data = float(split(input, ' '));
    ball = new Ball(box2dtransform.transform_point(new Point(data[1], data[2])), box2dtransform.transform_scalar(data[0]));
    if(paused){
      println("Requesting ball description...");
      println("Bytes received: " + c.available());
      println(input);
      ball._print();
      println("Radius: " + ball.radius);
    }
  }
  
  void request_robot_description(boolean paused){
    c.write("r\n\0");
    // Receive data from server
    while(c.available() == 0);
    input = c.readString();
    input = input.substring(0, input.indexOf("\n"));
    data = float(split(input, ' '));
    hx = box2dtransform.transform_scalar(data[0]);
    hy = box2dtransform.transform_scalar(data[1]);
    //r1.position = box2dtransform.transform_point(new Point(data[2], data[3]));
    if(paused){
      println("Requesting robot description...");
      println("Bytes received: " + c.available());
      println(input);
    }
  }

  void request_world_state(boolean paused){
    c.write("s\n\0");
    // Receive data from server
    while(c.available() == 0);
    input = c.readString();
    input = input.substring(0, input.indexOf("\n"));
    data = float(split(input, ' '));
    ball.set_position(box2dtransform.transform_point(new Point(data[0], data[1])));
    int number_of_robots = (int)data[2];
    robots = new Robot[number_of_robots];
    for(int i = 0; i < robots.length; ++i){
      robots[i] = new Robot(box2dtransform.transform_point(new Point(data[(i+1)*3], data[(i+1)*3+1])), data[(i+1)*3+2], hx*2.);
    }
    if(paused){
      println("Requesting world state...");
      println("Bytes received: " + c.available());
      println(input);
      for(int i = 0; i < robots.length; ++i){
        robots[i]._print();
      }
    }
  }

};
