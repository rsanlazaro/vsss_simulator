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

Box2DTCPHandler handler;

//1 meter is equivalent to "meterToPixel" pixels
float meterToPixel = 150.;

Field field;
Ball ball;
Robot[] robots;

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
  
  //TCP Handler for Box2D Server
  handler = new Box2DTCPHandler(this, "127.0.0.1", 27015);
  //Field description
  handler.request_field_description();
  //Ball description
  handler.request_ball_description();
  //Robot description
  handler.request_robot_description();
  
  
  /*
  //Set robots colors
  for(int i = 0; i < robots.length; ++i){
    if(i < 3){
      robots[i].set_team(1, team_1_color);
    } else{
      robots[i].set_team(2, team_2_color);
    }
  }*/
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
