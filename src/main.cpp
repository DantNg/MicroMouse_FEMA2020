//motor A connected between A01 and A02
//motor B connected between B01 and B02
#include <Encoder.h>
#include<Arduino.h>
#include<Dispatch.h>
#include<Wire.h>
#include<L3G4200D.h>
#include<StackList.h>
#include<QueueList.h>
////
#include<coord.h>
#include<entry.h>
#include<instruction.h>
//encoder A and B
const int L_encoder_A = 2;
const int L_encoder_B = 11;
const int R_encoder_A = 3;
const int R_encoder_B  = 12;
//Motor A
const int L_motor_forward = 9;
const int L_motor_backward = 5;

//Motor B
const int R_motor_forward = 10;
const int R_motor_backward = 6;

QueueList<instruction> instructions;

//setup encoder
Encoder rightEnc(R_encoder_A, R_encoder_B);
Encoder leftEnc(L_encoder_A, L_encoder_B);

long positionRight = -999;
long positionLeft = -999;
unsigned long oneTurn = 220L;
//IR Variable Setup
const int ir_side_out = 4;
const int ir_front_out = 8;
int ir_diag_out = 7;

const int front_r_in = A0;//confirmed
const int front_l_in = A1;//confirmed
const int side_r_in = A2;
const int side_l_in = A3;//confirmed
const int diag_r_in = A6;//confirmed
const int diag_l_in = A7;//confirmed
  //Sensor reading
  int sensorState = 0;

byte globalHeading = 4;
coord globalCoord = {0,0};
coord globalEnd = {0,0};

Dispatch dispatch(ir_diag_out, ir_side_out, ir_front_out, front_r_in, front_l_in, side_r_in, side_l_in, diag_r_in, diag_l_in);
//N,S,E,W
  byte headings[] = {1,2,4,8};
//Define some global constants
  #define X 16
  #define Y 16
  entry maze[Y][X];

//Setup Gyro
  L3G4200D gyro;
  // Can fine-tune this if you need to *****
  float DPS_MULT = 0.00001;  ///giảm thì đọ nhạy giảm 
  // Delta angles (raw input from gyro)
  float y = 0;
  // Actual angles
  float angY = 0;
  // Final angles (readable value for user)
  float real_angY = 0;
  // Previous angles for calculation
  float p_angY = 0;

  // Calibration values
  int gyroLowY = 0;
  int gyroHighY = 0;
  // Used for calculating delta time
  unsigned long pastMicros = 0;


void calibrateGyro()
{
  Serial.println("Calibrating gyro, don't move!");
  for (int i = 0; i <200; i++)
  {
     y = gyro.getY();
    if (y > gyroHighY)
      gyroHighY = y;
    else if (y < gyroLowY)
      gyroLowY = y;
    delay(10);
  }
  Serial.println("Calibration complete.");
}
ISR(TIMER2_OVF_vect) {
  // Interrupt routine.
  if(sensorState==0){
    //Power down the side sensors and power up the front LEDs
    dispatch.powerDown(dispatch.irArray[2][0]);
    dispatch.powerUp(dispatch.irArray[0][0]);
    sensorState++;
  }else if(sensorState==1){
    //Read both front sensors
    dispatch.readSensor(0,1);
    dispatch.readSensor(0,2);
    sensorState++;
  }else if(sensorState==2){
    //Power down the front sensors and power up the diag LEDs
    dispatch.powerDown(dispatch.irArray[0][0]);
    dispatch.powerUp(dispatch.irArray[1][0]);
    sensorState++;
  }else if(sensorState==3){
    //Read both diag sensors
    dispatch.readSensor(1,1);
    dispatch.readSensor(1,2);
    sensorState++;    
  }else if(sensorState==4){
    //Power down the diag sensors and power up the side LEDs
    dispatch.powerDown(dispatch.irArray[1][0]);
    dispatch.powerUp(dispatch.irArray[2][0]);
    sensorState++;
  }else if(sensorState==5){
    //Read both side sensors
    dispatch.readSensor(2,1);
    dispatch.readSensor(2,2);
    sensorState=0;
  }
}
void motorMove(float perSpeed, int pin1, int pin2) {
 
   if(perSpeed>0||perSpeed==0)
   {
     analogWrite(pin1, perSpeed);
      analogWrite(pin2, 0);
   }
   else if(perSpeed<0)
   {
     perSpeed*=-1;
     analogWrite(pin1, 0);
    analogWrite(pin2, perSpeed);
   }
}
void gyroRead()
{
  y = gyro.getY();
  // Calculate delta time
    float dt;
    if (micros() > pastMicros) //micros() overflows every ~70 minutes
      dt = (float) (micros() - pastMicros) / 1000000.0;
    else
      dt = (float) ((4294967295 - pastMicros) + micros()) / 1000000.0;

  // Calculate angles
    if (y >= gyroHighY || y <= gyroLowY)
    {
      angY += ((p_angY + (y * DPS_MULT)) / 2) * dt;
      p_angY = y * DPS_MULT;
    } else
    {
      p_angY = 0;
    }

    pastMicros = micros();
    real_angY = (int)(angY * 1000);
    
    if (real_angY < 0)
    {
      real_angY += 360;
    }
    else if (real_angY >= 360)
    {
      real_angY -= 360;
    }
    
  dispatch.gyroVal=real_angY;
}
void turnError(float error){
   //If the desiredPosition is greater than the currAngle then we need to rotate clockwise
   if(error>0.0){
     motorMove(-error, R_motor_backward,R_motor_forward);
     motorMove(error, L_motor_forward,L_motor_backward);
   //if the currAngle is greater than the desired position we need to rotate counter clockwise
   }else if(error < 0 ){
     motorMove(-error, R_motor_forward,R_motor_backward);
     motorMove(error, L_motor_backward,L_motor_forward);
   }
}

void moveDist(float dist)
{
  double 
        Kp = 5,
        Ki = 0.0,
        Kd = 0.1; 
  double 
        P = 0,
        I = 0,
        D = 0;
  float circ = 106.8;
  long encoder_count = (long)((dist/circ)*oneTurn);
  //float motorSpeed = .1;
  long currCountLeft  = leftEnc.read();
  //long currCountLeft  = 0;
  //long currCountLeft = leftEnc.read();
  long desiredCount = currCountLeft+encoder_count;
  //tính toán sai số
  float lastError=0;
  float error=(desiredCount-positionLeft);
  //GIÁ TRỊ GÓC BAN ĐẦU
  int passAngle = dispatch.gyroVal;
    
  while(error >20 || error <-20)
  {
   // ĐẶT GIÁ TRỊ PID
    P = error*Kp;

    I += error*Ki;
    
    D = ( lastError - error)*Kd;

    double PID = P+I+D;
    long speed = constrain(PID,-255,255);
    //bánh phải chậm hơi bánh trái 1 giá trị DENTA  
    /*
      chúng ta tính sai số góc của xe so với góc ban đầu của nó sau mỗi lần lặp
      dựa vào sai số nhân với tỉ lệ tự chọn , lấy giá trị đó làm DENTA giảm tốc độ
      động cơ bên trái 
    */
    gyroRead();
    int currAngl  = dispatch.gyroVal;
    int denta =  currAngl - passAngle;
    denta *= 45;  //tỉ lệ này tự chọn
    denta = constrain(denta,0,60); //giới hạn giá trị denta 0 -> 100
    Serial.print("Denta : ");
    Serial.println(denta);
    motorMove(speed-denta,L_motor_forward,L_motor_backward);
    motorMove(-1*speed,R_motor_forward,R_motor_backward); // *-1 để làm bánh phải quay đúng chiều , cái này do lỗi in mạch :b


    //TÍnh GiÁ TRỊ ENNCODER mới
    lastError = error;
    long newLeft = leftEnc.read();
    positionLeft = newLeft;
    //Serial.print("Position \t\t\t\t");
    //Serial.print(positionLeft);
    error=(desiredCount-abs(positionLeft));
   
  } 
    motorMove(0,L_motor_forward,L_motor_backward);
    motorMove(0,R_motor_backward,R_motor_forward);
 
 
}
//In MAZE
void printMaze(){
  for(int j=0; j<Y; j++){
    for(int i=0; i<X; i++){
      Serial.print(maze[j][i].walls);
      Serial.print(" ");
    }
    Serial.println();
  }
}
//Xác định ví trí hiện tại của tường
byte readCurrent(){
  byte wallReading = 15;
  byte north = 0;
  byte south = 0;
  byte east = 0;
  byte west = 0;
  switch(globalHeading){
    case 1:
      //if the forward sensor is tripped
      if(dispatch.irValues[0]>100){
        //set north to 1
        north = 1;
      }
      //if the right sensor is tripped
      if(dispatch.irValues[4]>100){
        //set east to 4
        east = 4;
      }
      //if the left sensor is tripped
      if(dispatch.irValues[5]>100){
        //set west to 9
        west = 8;
      }
      //Subtract the sum of north east and west from the value of wall reading
      wallReading -= (north+east+west);
      break;
    case 2:
      //if the forward sensor is tripped
      if(dispatch.irValues[0]>100){
        //set south to 2
        south = 2;
      }
      //if the right sensor is tripped
      if(dispatch.irValues[4]>100){
        //set west to 8
        west = 8;
      }
      //if the left sensor is tripped
      if(dispatch.irValues[5]>100){
        //set east to 4       
        east = 4;
      }
      //subtract the sum from 15
      wallReading-=(south+east+west);
      break;
    case 4:
      //if the forward sensor is tripped
      if(dispatch.irValues[0]>100){
        //set east to 4
        east = 4;
      }
      //if the right sensor is tripped
      if(dispatch.irValues[4]>100){
        //set south to 2
        south = 2;
      }
      //if the left sensor is tripped
      if(dispatch.irValues[5]>100){
        //set north to 1
        north = 1;
      }
      //subtract the sum from 15
      wallReading-=(north+south+east);
      break;
    case 8:
      //if the forward sensor is tripped
      if(dispatch.irValues[0]>100){
        //set east to 8
        west = 8;
      }
      //if the right sensor is tripped
      if(dispatch.irValues[4]>100){
        //set north to 1
        north = 1;
      }
      //if the left sensor is tripped
      if(dispatch.irValues[5]>100){
        //set south to 1
        south = 2;
      }
      //subtract the sum from 15
      wallReading-=(west+north+south);
      break;
    }
    return wallReading;
  }
int calcDist(byte posx, byte posy, byte desireX, byte desireY){
  int dist = (int) (abs(desireY-posy)+abs(desireX-posx));
  return dist;
}
int calcCenter(byte posx, byte posy, byte dim){
  byte center = dim/2;
  int dist = 0;
  
  if(posy<center){
    if(posx<center){
      //You're in the top left of the maze
      dist=calcDist(posx, posy, (center-1), (center-1));
    }else{
      //You're in the top right of the maze
      dist=calcDist(posx,posy,center,(center-1));
    }
  }else{
    if(posx>=center){
      //You're in the bottom right of the maze
      dist=calcDist(posx,posy,center,center);
    }else{
      //You're in the bottom left of the maze
      dist=calcDist(posx,posy, (center-1),center);
    }
  }
return dist;
}
//Khởi tạo mê cung ban đầu
void instantiate(){
  for(int j = 0; j<Y; j++){
    for(int i = 0; i<X; i++){
      maze[j][i].distance = calcCenter(i, j, X);
      maze[j][i].walls = 15;
      //If this is the left column (0,x)
      if(i==0){
        maze[j][i].walls = 7;
      }
      //if this is the top row
      if(j==0){
        maze[j][i].walls = 14;
      }
      //if this is the bottom row
      if(j==(Y-1)){
        maze[j][i].walls = 13;
      }
      //If this is the righ column
      if(i==(X-1)){
        maze[j][i].walls = 11;
      }
      maze[0][0].walls = 6;
      maze[Y-1][0].walls = 5;
      maze[0][X-1].walls = 10;
      maze[X-1][Y-1].walls = 9;
    }
  }
}
//Khởi tạo đường đi tối ưu ban đầu 
void instantiateReflood(){
  for(int j = 0; j<Y; j++){
    for(int i = 0; i<X; i++){
      maze[j][i].distance = calcCenter(i, j, X);
    }
  }
}

void resetToCoord(coord desiredCoord){
  for(int j = 0; j<Y; j++){
    for(int i = 0; i<X; i++){
      maze[j][i].distance = calcDist(i, j, desiredCoord.x, desiredCoord.y);
    }
  }
}
//Xác định vị trí tiếp theo cần đi 
coord bearingCoord(coord currCoord, byte heading){
  coord nextCoord = {0,0};
  switch (heading){
    case 1:
      //code
      nextCoord.x=currCoord.x;
      nextCoord.y=currCoord.y-1;
      break;
    case 2:
      nextCoord.x=currCoord.x;
      nextCoord.y=currCoord.y+1;
      break;
    case 4:
      nextCoord.x=currCoord.x+1;
      nextCoord.y=currCoord.y;
      break;
    case 8:
      nextCoord.x=currCoord.x-1;
      nextCoord.y=currCoord.y;
      break;
  }
  return nextCoord;
}
//Lấy 1 tọa độ kiểm tra xem nó có trong giới hạn mê cung k
boolean checkBounds(coord Coord){
  if((Coord.x >= X) || (Coord.y >= Y) || (Coord.x < 0) || (Coord.y < 0)){return false;}else{return true;}
}
//Xác định tọa độ mới tối ưu nơi cách xa tọa độ cũ nhất mà xe có thể di chuyển 
byte orient(coord currCoord, byte heading){
  
  coord leastNext = {0,0};
  //This is the absolute largest value possible (dimension of maze squared)
  int leastNextVal = sizeof(maze)*sizeof(maze);
  byte leastDir = heading;
  
  //If there is a bitwise equivalence between the current heading and the cell's value, then the next cell is accessible
  if((maze[currCoord.x][currCoord.y].walls & heading) != 0){
    //Define a coordinate for the next cell based onthis heading and set the leastNextVal t its value
    coord leastnextTemp = bearingCoord(currCoord, heading);
    
    if(checkBounds(leastnextTemp)){
      leastNext = leastnextTemp;
      leastNextVal = maze[leastNext.y][leastNext.x].distance;
    }
  }
  
  for(int i=0; i < sizeof(headings); i++){
    byte dir = headings[i];
    //if this dir is accessible
    if((maze[currCoord.y][currCoord.x].walls & dir) != 0){
      //define the coordiante for this dir
      coord dirCoord = bearingCoord(currCoord,dir);
      
      if(checkBounds(dirCoord)){
        //if this dir is more optimal than continuing straight
        if(maze[dirCoord.y][dirCoord.x].distance < leastNextVal){
          //update teh value of leastNextVal
          leastNextVal = maze[dirCoord.y][dirCoord.x].distance;
          //update the value of leastnext to this dir
          leastNext = dirCoord;
          leastDir = dir;
        }
      }
    }
  }
  return leastDir;
}
//Kiểm tra ô gần nhất
int checkNeighs(coord Coord){
  int minVal =  sizeof(maze)*sizeof(maze);
  for(int i=0; i<sizeof(headings); i++){
    byte dir = headings[i];
    //if this dir is accessible
    if((maze[Coord.y][Coord.x].walls & dir) != 0){
      //Get the coordinate of the accessible neighbor
      coord neighCoord = bearingCoord(Coord, dir);
      //Check the value of the accessible neighbor
      if (checkBounds(neighCoord)){
        //if the neighbore is less than the current recording minimum value, update the minimum value
        //If minVal is null, set it right away, otherwise test
        if(maze[neighCoord.y][neighCoord.x].distance < minVal){minVal = maze[neighCoord.y][neighCoord.x].distance;}
      }
    }
  }
  return minVal;
}

//Xác định ngõ cụt
boolean isDead(coord coord){
  boolean deadEnd = false;
  if(checkBounds(coord)){
    byte bounds = maze[coord.y][coord.x].walls;
    //bounds is the integer from the exploratory maze that represents the known walls of the coordinate
    if((bounds == 1)||(bounds == 2)||(bounds == 4) || (bounds == 8)){deadEnd=true;}
  }
  return deadEnd;
}
boolean isEnd(coord Coord, coord DesiredArray[]){
  boolean End = false;
  for(int i=0; i<sizeof(DesiredArray);i++){
    coord Desired = DesiredArray[i];
    if(checkBounds(Coord)){
      if((Coord.x == Desired.x)&&(Coord.y==Desired.y)){
        End = true;
      }
    }
  }
  return End;
}

boolean isHeading(byte heading){
  Serial.println(dispatch.gyroVal);
  boolean headingBool = false;
  switch(heading){
    //NORTH
    case 1:
      //if the robot's orientation is between 315 deg and 225 deg (pointing north)
      if((dispatch.gyroVal<315.0)&&(dispatch.gyroVal>225.0)){
        headingBool = true;
      }
      break;
    case 2:
      if((dispatch.gyroVal<135.0)&&(dispatch.gyroVal>45.0)){
        headingBool = true;
      }
      break;
    case 4:
      if((dispatch.gyroVal<=360 && dispatch.gyroVal>315.0)||(dispatch.gyroVal>=0 && dispatch.gyroVal<45.0)){
        headingBool = true;
      }
      break;
    case 8:
      if((dispatch.gyroVal>135.0)&&(dispatch.gyroVal<225.0)){
        headingBool = true;
      }
      break;
  }
  //Serial.println(headingBool);
  return headingBool;
}
void turn(float desiredPosition)
{
  float KP = 13.5 ;
  float KI = 0;
  float KD = 0;
  if(desiredPosition==0.0){
   // gyroRead();
    if(isHeading(1)){
      //this is ~270
      float currAngle = dispatch.gyroVal;
      //so make it ~-90
      currAngle-=360.0;
      //So the error will be ~90 because the desired position is greater than the current angle
      float error = (desiredPosition - currAngle);
      while((error>1.0)||(error<-1.0)){
        float P = 0;
        P = error* KP;
        long speed = constrain(P,-220,220);
        turnError(speed);
        gyroRead();
        if(dispatch.gyroVal>260.0){
          currAngle = dispatch.gyroVal-360.0;
        }
        error = (desiredPosition - currAngle);
      }
    }
    else if(isHeading(2)){
      //this is ~90
      float currAngle = dispatch.gyroVal;
      //So the error will be ~-90 because the desired position is less than the current angle
      float error = (desiredPosition - currAngle);
      while((error>1.0)||(error<-1.0)){
         float P = 0;
        P = error* KP;
        long speed = constrain(P,-220,220);
        turnError(speed);
        gyroRead();
        if(dispatch.gyroVal>260.0){
          currAngle = dispatch.gyroVal-360;
        }else{
          currAngle = dispatch.gyroVal;
        }
        error = (desiredPosition - currAngle);
      }
    }
    else if(isHeading(8)){
      //this is ~180
      float currAngle = dispatch.gyroVal;
      //So the error will be ~-180
      float error = (desiredPosition-currAngle);
      while((error>1.0)||(error<-1.0)){
        float P = 0;
        P = error* KP;
        long speed = constrain(P,-220,220);
        turnError(speed);
        gyroRead();
        if(dispatch.gyroVal>350.0){
          currAngle = dispatch.gyroVal-360;
        }else{
          currAngle = dispatch.gyroVal;
        }
        error = (desiredPosition - currAngle);
      }
    }
  }
  else if(desiredPosition == 90.0){
    //gyroRead();
   // Serial.println("90");
    if(isHeading(1)||isHeading(8)){
      //this is ~270 or ~180
      
      float currAngle = dispatch.gyroVal;
      //We want the robot to rotate counter clockwise which means a negative error
      //So subtract the currangle from the desired angle
      float error = (desiredPosition - currAngle);
      while((error>1.0)||(error<-1.0)){
        float P = 0;
        P = error* KP;
        long speed = constrain(P,-220,220);
        turnError(speed);
        gyroRead();
        currAngle = dispatch.gyroVal;
        Serial.println(dispatch.gyroVal);
        error = (desiredPosition - currAngle);
      }
    }
    else if(isHeading(4)){
      //this is ~0.0
    //  Serial.println("this is 0");
      float currAngle = dispatch.gyroVal;
      //if it is over shot by a bit, subtrack 360
      if(currAngle<350.0){
        currAngle-=360.0;
      }
      //We want the robot to rotate clockwise which means a positive error
      //So subtract the currangle from the desired angle
      float error = (desiredPosition - currAngle);
      while((error>1.0)||(error<-1.0)){
        
        float P = 0;
        P = error* KP;
        long speed = constrain(P,-220,220);
        turnError(speed);
        gyroRead();
        currAngle = dispatch.gyroVal;
        Serial.println(dispatch.gyroVal);
        error = (desiredPosition - currAngle);
      }
    }
  }
  else if(desiredPosition == 180.0){
  //  gyroRead();
    if(isHeading(1)){
      //this is ~270
      float currAngle = dispatch.gyroVal;
      //so we want to turn counter clockwise with a negative error value
      float error = (desiredPosition - currAngle);
      while((error>1.0)||(error<-1.0)){
        float P = 0;
        P = error* KP;
        long speed = constrain(P,-220,220);
        turnError(speed);
        gyroRead();
        currAngle = dispatch.gyroVal;
        Serial.println(dispatch.gyroVal);
        error = (desiredPosition - currAngle);
      }
    }
    else if(isHeading(2)||isHeading(4)){
      
      //this is ~90 or ~0
      float currAngle = dispatch.gyroVal;
      //if it is over shot by a bit, subtrack 360
      if(currAngle<350.0){
        currAngle-=360.0;
      }
       //We want the robot to rotate clockwise which means a positive error
      //So subtract the currangle from the desired angle
      float error = (desiredPosition - currAngle);
      while((error>1.0)||(error<-1.0)){
        float P = 0;
        P = error* KP;
        long speed = constrain(P,-220,220);
        turnError(speed);
        gyroRead();
        currAngle = dispatch.gyroVal;
        Serial.println(dispatch.gyroVal);
        error = (desiredPosition - currAngle);
      }
    }
  }
  else if(desiredPosition == 270.0){
   // gyroRead();
    if(isHeading(4)){
      
      //this is ~0
      float currAngle = dispatch.gyroVal;
      //if it is undershot
      if(currAngle<5.0){
        currAngle+=360.0;
      }
      //We want the robot to rotate counter clockwise which means a negative error
      //So subtract the currangle from the desired angle
      float error = (desiredPosition - currAngle);
      while((error>1.0)||(error<-1.0)){
        float P = 0;
        P = error* KP;
        long speed = constrain(P,-220,220);
        turnError(speed);
        gyroRead();
        currAngle = dispatch.gyroVal;
       // Serial.println(dispatch.gyroVal);
        error = (desiredPosition - currAngle);
        //if it is still undershot
        if(currAngle<5.0){
          currAngle+=360.0;
        }
        error = (desiredPosition - currAngle);
      }
    }
    else if(isHeading(2)||isHeading(8)){
      
      //this is ~90 or ~180
      float currAngle = dispatch.gyroVal;

      //We want the robot to rotate clockwise which means a positive error
      //So subtract the currangle from the desired angle
      float error = (desiredPosition - currAngle);
      while((error>1.0)||(error<-1.0)){
        float P = 0;
        P = error* KP;
        long speed = constrain(P,-220,220);
        turnError(speed);
        gyroRead();
        currAngle = dispatch.gyroVal;
       // Serial.println(dispatch.gyroVal);
        error = (desiredPosition - currAngle);
      }
    }
  }
}
/*
void turnAngle(float desiredAngle)
{
  //gyroRead();
  float currAng = dispatch.gyroVal; 
  float lastError = currAng;
  float error_angle = desiredAngle- currAng;
  while(error_angle >1 || error_angle <-1 )
  {
  float
        P = 0,
       // I = 0,
        D = 0;
    P = error_angle* 20;
    //I += error_angle* 0.05;//0.05
    D = ( lastError - error_angle)*0.0005;
    double PID = P+D;
    long speed = constrain(PID,-200,200);
    turnError(speed);
    lastError = currAng;
     gyroRead();
     currAng = dispatch.gyroVal;
    Serial.println(error_angle);
    error_angle=(desiredAngle-currAng);   
  } 
    motorMove(0,L_motor_forward,L_motor_backward);
    motorMove(0,R_motor_backward,R_motor_forward);
   
}
*/


void setup(){
//setup motor pin
    pinMode(L_motor_forward, OUTPUT);
    pinMode(L_motor_backward, OUTPUT);

    pinMode(R_motor_forward, OUTPUT);
    pinMode(R_motor_backward, OUTPUT);
//setup pin encoder
    pinMode(R_encoder_A, INPUT);
    digitalWrite(R_encoder_A, LOW);
    pinMode(R_encoder_B, INPUT);
    digitalWrite(R_encoder_B, LOW);
    pinMode(L_encoder_A, INPUT);
    digitalWrite(L_encoder_A, LOW);
    pinMode(L_encoder_B, INPUT);
    digitalWrite(L_encoder_B, LOW); 
//
digitalWrite(7, LOW);
digitalWrite(8, LOW);
digitalWrite(12, LOW);
dispatch.calibrateSensors();
   
  Serial.println(dispatch.calibratedArray[0]);
  Serial.println(dispatch.calibratedArray[1]);
  Serial.println(dispatch.calibratedArray[2]);
  Serial.println(dispatch.calibratedArray[3]);
  Serial.println(dispatch.calibratedArray[4]);
  Serial.println(dispatch.calibratedArray[5]);
  Serial.println();
///
  Serial.begin(9600);
   Wire.begin();
  gyro.initialize(250);
  Serial.begin(9600);
  Serial.println("Starting up L3G4200D");
  delay(1000); // wait for the sensor to be ready
  calibrateGyro();
 // attachInterrupt(0, updateAngle, RISING);
  pastMicros = micros();
   gyroRead();
  // Enable Timer2 interrupt.
 TIMSK2 = (0<<OCIE2A) | (1<<TOIE2);
 //moveDist(200);
}

void loop()
{
    
moveDist(200);
delay(500);
isHeading(4);
turn(90);
delay(400);
moveDist(200);
isHeading(8);
delay(500);
turn(180);
moveDist(200);
delay(500);
turn(270);
moveDist(200);


}

