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
Robot_kine robot_kine;
Box2DTCPHandler handler;

//1 meter is equivalent to "meterToPixel" pixels
float meterToPixel = 150.;
String input;
float data[];

Field field;
Ball ball;
Robot[] robots;
Server s;
Client c1;
Client c2;


//robot teams (1 for team 1 and 2 for team 2)
int[] robot_teams = {1,1,1,2,2,2};
//robot roles (1 for goalkeeper, 2 for midfield and 3 for forward)
int[] robot_roles = {1,2,3,1,2,3};

int background_color = 15;
color ball_color = color(255, 128, 0);
color team_1_color = color(89, 0, 255);
color team_2_color = color(216, 255, 0);

boolean paused = true;
boolean frame_request = true;

void setup() 
{
  size(1500, 1000);
  background(background_color);
  stroke(255);
  frameRate(60);
  
  s = new Server(this, 12345); // Start a simple server on a port
  robot_kine=new Robot_kine(0.1, 0.4);
  //TCP Handler for Box2D Server
  handler = new Box2DTCPHandler(this, "127.0.0.2", 27015);
  //Field description
  handler.request_field_description();
  //Ball description
  handler.request_ball_description();
  //Robot description
  handler.request_robot_description();
  
  
  // Receive data from client

  
}

void draw() 
{
  if(!paused || frame_request){
    handler.request_world_state();
    
    //Draw state
    background(background_color);
    field._draw(background_color);
    ball._draw();
    for(int i = 0; i < robots.length; ++i){
      robots[i]._draw();
    }
    
    frame_request = false;
    

  c1 = s.available();
  if (c1 != null) {
    println("recieving data from client");
    input = c1.readString();

    input = input.substring(0, input.indexOf("\n")); // Only up to the newline
    data = float(split(input, ' ')); // Split values into an array
    robot_kine.setSpeed(data[1], data[2]);
    int id = int(data[0]);
    handler.send_velocities(robot_kine.VL, robot_kine.VR, id);
  }
/*  c2 = s.available();
  if (c2 != null) {
    println("recieving data from client");
    input = c1.readString();

    input = input.substring(0, input.indexOf("\n")); // Only up to the newline
    data = float(split(input, ' ')); // Split values into an array
    robot_kine.setSpeed(data[1], data[2]);
    int id = int(data[0]);
    handler.send_velocities(robot_kine.VL, robot_kine.VR, id);
  }*/
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
