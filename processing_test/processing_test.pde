/**
 * Shared Drawing Canvas (Client) 
 * by Alexander R. Galloway. 
 * 
 * The Processing Client class is instantiated by specifying a remote 
 * address and port number to which the socket connection should be made. 
 * Once the connection is made, the client may read (or write) data to the server.
 * Before running this program, start the Shared Drawing Canvas (Server) program.
 */


import processing.net.*;

Client c;
String input;
float data[];

float ball_radius;
Point ball_pos;

Point robot_pos;
float robot_angle;
float hx, hy;

Point center;
Box2DTransform box2dtransform;
Field field;

void setup() 
{
  size(1500, 1000);
  background(204);
  stroke(0);
  frameRate(60); // Slow it down a little
  
  center = new Point(width / 2.0, height / 2.0);
  box2dtransform = new Box2DTransform(150, center);
  
  // Connect to the server's IP address and port
  c = new Client(this, "127.0.0.1", 27015); // Replace with your server's IP and port
  
  //Get simulator field coordinates
  println("Requesting field coordinates...");
  c.write("f\n\0");
  // Receive data from server
  delay(100);
  if (c.available() > 0) {
    input = c.readString();
    input = input.substring(0, input.indexOf("\n"));
    data = float(split(input, ' '));
    println(data);
    field = new Field(data, box2dtransform);
  }
  
  //Get ball definition
  println("Requesting ball definition...");
  c.write("b\n\0");
  // Receive data from server
  delay(100);
  if (c.available() > 0) {
    input = c.readString();
    input = input.substring(0, input.indexOf("\n"));
    data = float(split(input, ' '));
    println(data);
    ball_radius = box2dtransform.transform_scalar(data[0]);
    ball_pos = box2dtransform.transform_point(new Point(data[1], data[2]));
  }
  
  //Get robot definition
  println("Requesting robot definition...");
  c.write("r\n\0");
  // Receive data from server
  delay(1000);
  if (c.available() > 0) {
    input = c.readString();
    input = input.substring(0, input.indexOf("\n"));
    data = float(split(input, ' '));
    println(data);
    hx = box2dtransform.transform_scalar(data[0]);
    hy = box2dtransform.transform_scalar(data[1]);
    robot_pos = box2dtransform.transform_point(new Point(data[2], data[3]));
  }
}

void draw() 
{
  
  background(204);
  c.write("s\n\0");
  // Receive data from server
  if (c.available() > 0) {
    input = c.readString();
    input = input.substring(0, input.indexOf("\n"));
    println(input);
    data = float(split(input, ' '));
    println(data);
    ball_pos = box2dtransform.transform_point(new Point(data[0], data[1]));
    robot_pos = box2dtransform.transform_point(new Point(data[3], data[4]));
    robot_pos._print();
    robot_angle = data[5];
  }
  
  fill(255,0,255);
  strokeWeight(3);
  

  field._draw();
  
  center._draw();
  
  stroke(0);
  strokeWeight(1);
  circle(ball_pos.x, ball_pos.y, ball_radius*2);
  pushMatrix();
  translate(robot_pos.x, robot_pos.y);
  rotate(robot_angle);
  rect(-hx, -hy, hx*2, hy*2);
  popMatrix();
  
}
