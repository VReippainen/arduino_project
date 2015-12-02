import processing.serial.*;
Serial mySerial;
Table table;
String filename = "data22.txt";
String val;
int starttime;
int duration = 10000;
int endtime;
void setup() {
  table = new Table();
  table.addColumn("id");
  mySerial = new Serial( this, Serial.list()[0], 57600 );
  table.addColumn("millis");
  table.addColumn("sensor");
  starttime = millis();
  delay(3000);
}
void draw() {
  starttime = millis();
  endtime = starttime;
  while(endtime < (starttime+duration)){
  val = mySerial.readStringUntil('\n');
  if(val!=null && (millis()-endtime) > 5){
    val = trim(val);
    int sensorVals[] = int(split(val, ','));
    TableRow newRow = table.addRow();
    newRow.setInt("id", table.lastRowIndex());
    newRow.setInt("millis", millis()-starttime);
    newRow.setInt("sensor", (sensorVals[0]));
    endtime = millis();
  }
  
  
  }
  saveTable(table, filename, "csv");
  println("Ready");
  
  
  
  
}