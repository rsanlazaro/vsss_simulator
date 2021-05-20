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
Point field_p[] = new Point[16];
Point center;

boolean firstTime = true;

void setup() 
{
  size(800, 800);
  background(204);
  stroke(0);
  frameRate(5); // Slow it down a little
  // Connect to the server's IP address and port
  c = new Client(this, "127.0.0.1", 27015); // Replace with your server's IP and port
  
  center = new Point(width / 2.0, height / 2.0);
  //Get simulator field coordinates
  println("Requesting field coordinates...");
  c.write("f\n");
  // Receive data from server
  delay(100);
  if (c.available() > 0) {
    input = c.readString();
    input = input.substring(0, input.indexOf("\n"));
    data = float(split(input, ' '));
    println(data);
    for(int i = 0; i < 16; ++i){
      field_p[i] = new Point(data[i*2]*200+center.x,-data[i*2+1]*200+center.y);
      field_p[i]._print();
    }
  }
  
  
}

void draw() 
{
  if(firstTime){
    
    
    
    firstTime = false;
  }
  
  /*
  c.write("s\n");
  // Receive data from server
  if (c.available() > 0) {
    input = c.readString();
    input = input.substring(0, input.indexOf("\n"));
    print(input);
    data = float(split(input, ' '));
    //println(data);
  }
  */
  fill(255,0,255);
  strokeWeight(3);
  

  noFill();
  beginShape();
  
  for(int i = 0; i < 16; ++i){
    vertex(field_p[i].x, field_p[i].y);
  }
  
  endShape(CLOSE);
  
  for(int i = 0; i < 16; ++i){
    //if(i <= 4){
    field_p[i]._draw();
    //}
  }
  center._draw();
}
