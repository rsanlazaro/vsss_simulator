class Point{
  float x,y;
  Point(){
    this.x = this.y = 0.0;
  }
  Point(float x, float y){
    this.x = x;
    this.y = y;
  }
  void _print(){
    //println("x: " + x + " y: " + y);
  }
  void _draw(){
    fill(255,0,255);
    noStroke();
    circle(x, y, 5);
  }
}
