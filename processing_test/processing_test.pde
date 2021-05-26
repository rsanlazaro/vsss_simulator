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

Robot r1, r2;
float hx, hy;

Point center;
Box2DTransform box2dtransform;
Field field;

int background_color = 15;

boolean paused = false;
boolean frame_request = false;

void setup() 
{
  size(1500, 1000);
  background(background_color);
  stroke(255);
  frameRate(60);
  
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
    //r1.position = box2dtransform.transform_point(new Point(data[2], data[3]));
  }
}

void draw() 
{
  if(!paused || frame_request){
    background(background_color);
    
    c.write("s\n\0");
    println("Requesting world state...");
    delay(1);
    // Receive data from server
    println(c.available());
    if (c.available() > 0) {
      println("data");
      input = c.readString();
      input = input.substring(0, input.indexOf("\n"));
      println(input);
      data = float(split(input, ' '));
      println(data);
      ball_pos = box2dtransform.transform_point(new Point(data[0], data[1]));
      r1 = new Robot(box2dtransform.transform_point(new Point(data[3], data[4])), data[5], hx*2.);
      r2 = new Robot(box2dtransform.transform_point(new Point(data[6], data[7])), data[8], hx*2.);
    }
    
    
    strokeWeight(3);
    
  
    field._draw(background_color);
    fill(255,0,255);
    stroke(255);
    strokeWeight(1);
    circle(ball_pos.x, ball_pos.y, ball_radius*2);
    r1._draw();
    r2._draw();
    
    frame_request = false;
  }
  
  
}

void keyPressed(){
  if(key == 'p'){
    paused = !paused;
  }
  if(key == ' '){
    frame_request = true;
  }
}
