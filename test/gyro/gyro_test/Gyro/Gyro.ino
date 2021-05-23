/*
 author : Alexander Hadik
 editer : Dat Nguyen
 this version using L3G4200D and library L3G
*/

#include <Wire.h>
#include <L3G.h>

L3G gyro;
int sampleNum=750;
   unsigned long time; 
  int sampleTime=10; 
  int rate; 
  long dc_offset=0; 
  double noise=0;
  int prev_rate=0; 
  double angle=0; 
  
void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }

  gyro.enableDefault();
   //Calculate initial DC offset and noise level
  for(int n=0;n<sampleNum;n++){ 
   gyro.read(); 
   dc_offset+=(long)gyro.g.y; 
  } 
  dc_offset=dc_offset/sampleNum; 
  
  for(int n=0;n<sampleNum;n++){
   gyro.read(); 

   if((int)gyro.g.z-dc_offset>noise) 
     noise=(int)gyro.g.y-dc_offset; 
    else if((int)gyro.g.y-dc_offset<-noise) 
     noise=-(int)gyro.g.y-dc_offset; 
  } 
  noise=noise/100; //gyro returns hundredths of degrees/sec 
}

void loop() {
  
  gyroRead();
  delay(100);
  /*
  Serial.print("X: ");
  Serial.print(gyro.g.x);
  Serial.print(" Y: ");
  Serial.print(gyro.g.y);
  Serial.print(" Z: ");
  Serial.println(gyro.g.z);
 */
}

void gyroRead(){
  
  // Every 10 ms take a sample from the gyro 
 //if(millis() - time > sampleTime) {
   gyro.read(); 
   rate=((int)gyro.g.y-dc_offset)/100; 
    
    angle += ((double)(prev_rate + rate) * sampleTime) / 200; 
    // remember the current speed for the next loop rate integration. 
    prev_rate = rate; 
    // Keep our angle between 0-359 degrees 
    
    
   if (angle < 0)
     angle += 360; 
   else if (angle >= 360)
     angle -= 360; 


    Serial.print("Angle: ");
    Serial.print(angle);    
    Serial.print("\trate: "); 
    Serial.println(rate);
   
    
  
}
