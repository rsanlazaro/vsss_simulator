import processing.net.*;

//TCP communication with box2D server
Box2DTCPHandler handler;

//TCP communication with processing clients
Server s;
Client c1;
Client c2;
String input;
String commands[];
float data[];

//1 meter is equivalent to "meterToPixel" pixels
float meterToPixel = 50.;

Field field;
Ball ball;
Robot[] robots;
RobotKinematicModel model;
float wheel_radius = 0.1;

//robot teams (1 for team 1 and 2 for team 2)
int[] robot_teams = {1,1,1,2,2,2};
//robot roles (1 for goalkeeper, 2 for midfield and 3 for forward)
int[] robot_roles = {1,2,3,1,2,3};

int background_color = 15;
color ball_color       = color(255, 128, 0);   //Orange-ish
color robots_color     = color(113, 190, 202);  //turquoise-ish
color team_1_color     = color(209, 65, 248); //Purple-ish
color team_2_color     = color(248, 242, 57);  //Yellow-ish
color goalkeeper_color = color(255, 0, 0);     //Red-ish
color midfield_color   = color(0, 255, 0);     //Green-ish
color forward_color    = color(0, 0, 255);     //Blue-ish


boolean paused = false;
boolean frame_request = true;

void setup() 
{
  size(500, 350);
  background(background_color);
  stroke(255);
  frameRate(60);
  
  //TCP Handler for Box2D Server
  handler = new Box2DTCPHandler(this, "127.0.0.2", 27015);
  //Field description
  handler.request_field_description();
  //Ball description
  handler.request_ball_description();
  //Robot description
  handler.request_robot_description();
  //Thread that handles the communication with clients
  thread("ClientsTCP");
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

void ClientsTCP(){
  s = new Server(this, 12345); // Start a simple server on a port
  
  //Recieve data from clients
  while(true){
    c1 = s.available();
    if (c1 != null) {
      println("Recieving data from client");
      input = c1.readString();
      println(input);
      commands = split(input, '\n');
      for(int i = 0; i < commands.length; ++i){
        data = float(split(commands[i], ' '));
        if(data.length < 3){
          break;
        }
        println(commands[i]);
        println(data);
        int id = int(data[0]);
        model.setSpeed(data[1], data[2]);
        handler.send_velocities(model, id);
        println("Robot: " + id);
        println("Left  velocity: " + model.left_velocity);
        println("Right velocity: " + model.right_velocity);
      }
    }
    delay(5);
  }
}
