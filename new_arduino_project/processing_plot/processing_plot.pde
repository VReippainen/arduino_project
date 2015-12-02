import processing.serial.*;
Serial myPort;
int xPos = 1;
String val;
String oldVal;
void setup(){
String portName = Serial.list()[0];
myPort = new Serial(this, portName, 57600);
size(1600,1200);
background(0);

}
void draw(){
  
    val = myPort.readStringUntil('\n');
    oldVal = val;

  if(val != null){
    val = trim(val);
  
  float inByte = float(val);
  inByte = map(inByte, 745,976,0,height);
  stroke(127,34,255);
  line(xPos, height, xPos, height - inByte);
  if(xPos >= width){
    xPos = 0;
    background(0);
  }
  else{
    xPos++;
  }
  println(val);}
  String oldVal = val;
}