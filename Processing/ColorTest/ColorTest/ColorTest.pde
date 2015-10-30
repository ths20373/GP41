import processing.serial.*;
 
Serial myPort;
String str_get_data = null;
String buf[];
 
void setup() 
{
  size(500, 500);
  myPort = new Serial(this, "COM4", 38400);
  delay(3000);
}
 
void draw() 
{
  str_get_data = myPort.readStringUntil(10);
   
  println(str_get_data);
        
  if (str_get_data != null){
    str_get_data = trim(str_get_data);    //改行コード取り除き
    buf = split(str_get_data,",");
     
    //色を設定
    fill(int(buf[1])*10, int(buf[2])*10, int(buf[3])*10);
    rect(100 ,100, 300, 300);
  }
   
  delay(50);
   
}