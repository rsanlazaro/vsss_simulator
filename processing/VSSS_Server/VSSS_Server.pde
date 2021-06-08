
import processing.net.*;
import gab.opencv.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Mat;

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
float meterToPixel = 150.;

Field field;
Ball ball;
Robot[] robots;
RobotKinematicModel model;
float wheel_radius = 0.1;

//robot teams (1 for team 1 and 2 for team 2)
int[] robot_teams = {1,1,1,2,2,2};
//robot roles (1 for goalkeeper, 2 for midfield and 3 for forward)
int[] robot_roles = {1,2,3,1,2,3};

// Vision Settings
int background_color = 15;
color ball_color       = color(255, 128, 0);    // Orange-ish
color team_1_color     = color(0, 0, 255);      // Blue
color team_2_color     = color(255, 255, 0);    // Yellow
color goalkeeper_color = color(0, 255, 0);      // Green
color midfield_color   = color(26, 238, 255);   // Cyan
color striker_color    = color(230, 25, 255);   // Purple

boolean startC   = false; // Variable to start frameCount
boolean displayC = false; // Variable to display the coordinates
boolean coord    = true; // If true, it shows the values for x, y and angle
int offset       = 120; // Value used to display the coordinates
int rate         = 1; // The rate at which the coordinates are computed
Vision robotClass1, robotClass2, robotClass3, robotClass4, robotClass5, robotClass6, ballClass; // Class for vision

boolean paused = true;
boolean frame_request = true;

void setup() 
{
  size(1500, 1000);
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
  // Computer Vision
  robotClass1 = new Vision(this, team_1_color, goalkeeper_color);
  robotClass2 = new Vision(this, team_1_color, midfield_color);
  robotClass3 = new Vision(this, team_1_color, striker_color);
  robotClass4 = new Vision(this, team_2_color, goalkeeper_color);
  robotClass5 = new Vision(this, team_2_color, midfield_color);
  robotClass6 = new Vision(this, team_2_color, striker_color);
  ballClass   = new Vision(this, ball_color);
}

void draw() 
{
  if(!paused || frame_request){
    println(int(frameRate));
    handler.request_world_state();
    //Draw state
    background(background_color);
    field._draw(background_color);
    ball._draw();
        for(int i = 0; i < robots.length; ++i){
      robots[i]._draw();
    }

    if (!startC || frameCount%rate == 0){
      robotClass1.update(1);
      robotClass2.update(1);
      robotClass3.update(1);
      robotClass4.update(1);
      robotClass5.update(1);
      robotClass6.update(1);
      ballClass.update(2);
      startC = true;
      displayC = true;
    }else{
      displayC = false;
    }
    
    if (paused){
      
      fill(255);
      textSize(12);
      strokeWeight(3);
      stroke(255,0,0);
      
      offset = 60;
      if(robotClass1.centroidG.x>width/2){
        offset = -120;
      }
      text("x: " + nf(robotClass1.centroidG.x,0,2),robotClass1.centroidG.x+offset,robotClass1.centroidG.y);
      text("y: " + nf(robotClass1.centroidG.y,0,2),robotClass1.centroidG.x+offset,robotClass1.centroidG.y+15);
      text("a: " + nf(robotClass1.angle,0,2),robotClass1.centroidG.x+offset,robotClass1.centroidG.y+30);
      line(robotClass1.centroidG.x,robotClass1.centroidG.y,robotClass1.centroidG.x+(cos(radians(robotClass1.angle))*35),robotClass1.centroidG.y-(sin(radians(robotClass1.angle))*35));
      
      offset = 60;
      if(robotClass2.centroidG.x>width/2){
        offset = -120;
      }
      text("x: " + nf(robotClass2.centroidG.x,0,2),robotClass2.centroidG.x+offset,robotClass2.centroidG.y);
      text("y: " + nf(robotClass2.centroidG.y,0,2),robotClass2.centroidG.x+offset,robotClass2.centroidG.y+15);
      text("a: " + nf(robotClass2.angle,0,2),robotClass2.centroidG.x+offset,robotClass2.centroidG.y+30);
      line(robotClass2.centroidG.x,robotClass2.centroidG.y,robotClass2.centroidG.x+(cos(radians(robotClass2.angle))*35),robotClass2.centroidG.y-(sin(radians(robotClass2.angle))*35));
      
      offset = 60;
      if(robotClass3.centroidG.x>width/2){
        offset = -120;
      }
      text("x: " + nf(robotClass3.centroidG.x,0,2),robotClass3.centroidG.x+offset,robotClass3.centroidG.y);
      text("y: " + nf(robotClass3.centroidG.y,0,2),robotClass3.centroidG.x+offset,robotClass3.centroidG.y+15);
      text("a: " + nf(robotClass3.angle,0,2),robotClass3.centroidG.x+offset,robotClass3.centroidG.y+30);
      line(robotClass3.centroidG.x,robotClass3.centroidG.y,robotClass3.centroidG.x+(cos(radians(robotClass3.angle))*35),robotClass3.centroidG.y-(sin(radians(robotClass3.angle))*35));
      
      offset = 60;
      if(robotClass4.centroidG.x>width/2){
        offset = -120;
      }        
      text("x: " + nf(robotClass4.centroidG.x,0,2),robotClass4.centroidG.x+offset,robotClass4.centroidG.y);
      text("y: " + nf(robotClass4.centroidG.y,0,2),robotClass4.centroidG.x+offset,robotClass4.centroidG.y+15);
      text("a: " + nf(robotClass4.angle,0,2),robotClass4.centroidG.x+offset,robotClass4.centroidG.y+30);
      line(robotClass4.centroidG.x,robotClass4.centroidG.y,robotClass4.centroidG.x+(cos(radians(robotClass4.angle))*35),robotClass4.centroidG.y-(sin(radians(robotClass4.angle))*35));
      
      offset = 60;
      if(robotClass5.centroidG.x>width/2){
        offset = -120;
      }
      text("x: " + nf(robotClass5.centroidG.x,0,2),robotClass5.centroidG.x+offset,robotClass5.centroidG.y);
      text("y: " + nf(robotClass5.centroidG.y,0,2),robotClass5.centroidG.x+offset,robotClass5.centroidG.y+15);
      text("a: " + nf(robotClass5.angle,0,2),robotClass5.centroidG.x+offset,robotClass5.centroidG.y+30);
      line(robotClass5.centroidG.x,robotClass5.centroidG.y,robotClass5.centroidG.x+(cos(radians(robotClass5.angle))*35),robotClass5.centroidG.y-(sin(radians(robotClass5.angle))*35));
      
      offset = 60;
      if(robotClass6.centroidG.x>width/2){
        offset = -120;
      }
      text("x: " + nf(robotClass6.centroidG.x,0,2),robotClass6.centroidG.x+offset,robotClass6.centroidG.y);
      text("y: " + nf(robotClass6.centroidG.y,0,2),robotClass6.centroidG.x+offset,robotClass6.centroidG.y+15);
      text("a: " + nf(robotClass6.angle,0,2),robotClass6.centroidG.x+offset,robotClass6.centroidG.y+30);
      line(robotClass6.centroidG.x,robotClass6.centroidG.y,robotClass6.centroidG.x+(cos(radians(robotClass6.angle))*35),robotClass6.centroidG.y-(sin(radians(robotClass6.angle))*35));
      
      offset = 30;
      if(ballClass.centroidG.x>width/2){
        offset = -90;
      }
      text("x: " + nf(ballClass.centroidG.x,0,2),ballClass.centroidG.x+offset,ballClass.centroidG.y);
      text("y: " + nf(ballClass.centroidG.y,0,2),ballClass.centroidG.x+offset,ballClass.centroidG.y+15);
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
