import processing.net.*;

Client c2;


Robot_control r1;
Robot_control r2;
Robot_control r3;

void setup() 
{
  c2 = new Client(this, "127.0.0.2", 12345); // Replace with your server's IP and port
  r1 = new Robot_control(3);
  r2 = new Robot_control(4);
  r3 = new Robot_control(5);

  r1.set_vel_robot(4,-5); 
  r2.set_vel_robot(4,-5);
  r3.set_vel_robot(4,5);

}

void draw() 
{
  
  r1.set_vel_robot(10,-5); 
  r2.set_vel_robot(-1,5);
  r3.set_vel_robot(5,0);
  delay(2000);
   r1.set_vel_robot(-10,-5); 
  r2.set_vel_robot(1,5);
  r3.set_vel_robot(-5,0);
  delay(2000);
   r1.set_vel_robot(0,-5); 
  r2.set_vel_robot(0,5);
  r3.set_vel_robot(0,0);
  delay(2000);
   r1.set_vel_robot(0,5); 
  r2.set_vel_robot(1,5);
  r3.set_vel_robot(-5,0);
  delay(2000);
}
