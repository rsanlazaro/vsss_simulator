import processing.net.*;

Client c1;
String input;
int data[];
float Vl=-1.0;
float Va=1.0;
float idx=0;
int aux=0;

Robot_control r1;
Robot_control r2;
Robot_control r3;

void setup() 
{
  c1 = new Client(this, "127.0.0.2", 12345); // Replace with your server's IP and port
  r1 = new Robot_control(0);
  r2 = new Robot_control(1);
  r3 = new Robot_control(2);
  r1.set_vel_robot(0,10); 
  r2.set_vel_robot(0,0);
  r3.set_vel_robot(0,0);
}

void draw() 
{  
  r1.set_vel_robot(0,10); 
  r2.set_vel_robot(0,0);
  r3.set_vel_robot(0,0);
  if (c1.available() > 0){
    input = c1.readString();
    println(input);
  }
  //if (mousePressed == true) {
   // Vl=1.0;
    //Va=1.0;
    
    //c1.write(Vl + " " + Va + " " + idx + "\n");
    
    
 // }
}
