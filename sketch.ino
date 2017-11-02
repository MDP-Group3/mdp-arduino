#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

/*-------------------------------Combined Code-------------------------------------*/
#include <SharpIR.h>
#include <PID_v1.h>
#include <DualVNH5019MotorShield.h>
DualVNH5019MotorShield md;
#include <math.h>
#include <RunningMedian.h>

/*-----------------------------Pins and Ports-----------------------------------*/


#define R_ENCODER 5 //wheel 107
#define L_ENCODER 13 //wheel 123
#define LL_SENSOR A0 //
#define L_SENSOR A1 //
#define FL_SENSOR A2 //
#define FM_SENSOR A3 //
#define FR_SENSOR A4 //
#define R_SENSOR A5//

#define sen GP2Y0A21YK0F
#define sen_1 GP2Y0A02YK0F

boolean finish = false;    // for acknowledgement  

/*#define s1 A0 //front_middle
#define s2 A1 //front_right(from robot's view)
#define s3 A2 //front_left (from robot's view)
#define s4 A3 //left_side short
#define s5 A4 //right_side
#define s6 A5 //left_long range*/


//SharpIR S1 (LL_SENSOR, 35, 0.97, 0.0, 20150);
//SharpIR S2 (L_SENSOR, 35, 0.97, 0.0, 1080);
//SharpIR S3 (FL_SENSOR, 35, 0.97, 0.0, 1080);
//SharpIR S4 (FM_SENSOR, 35, 0.97, 0.0, 1080);
//SharpIR S5 (FR_SENSOR, 35, 0.97, 0.0, 1080);
//SharpIR S6 (R_SENSOR, 35, 0.97, 0.0, 1080);

SharpIR S1 (sen_1,LL_SENSOR);
SharpIR S2 (sen,L_SENSOR);
SharpIR S3 (sen,FL_SENSOR);
SharpIR S4 (sen,FM_SENSOR);
SharpIR S5 (sen,FR_SENSOR);
SharpIR S6 (sen,R_SENSOR);

SharpIR sensor[] = {S1, S2, S3, S4, S5, S6};



volatile double rencoder = 0;
volatile double lencoder = 0;

/*-------------------------------PID Variables--------------------------------*/
double volatile Rpwm;
double volatile Lpwm;
double volatile cur_Rrpm;
double volatile cur_Lrpm;
double volatile Rrpm = 80;//setpoint
double volatile Lrpm = 80;//setpoint
double volatile new_Rrpm;
double volatile new_Lrpm;
double volatile Lspeed;
double volatile Rspeed;
double volatile FRrpm = 110;//setpoint //120 
double volatile FLrpm = 110;//setpoint //120
double volatile output;

float flkp = 0.8;//9.9,8
float flki = 0;// might have to change this.4.8 //5.9
float flkd = 0;//0.16,0.2

float frkp = 0.512;//8; 0.015
float frki = 0;//4.95 - 7.5//4.85;7
float frkd = 0;//0.17;

float lkp = 8.9;//9.9,8
float lki = 5.9;// might have to change this.4.8 //5.9
float lkd = 0.5;//0.16,0.2

float rkp = 0.020;//8; 0.015 - OLD//0.04
float rki = 0;//4.95 - 7.5//4.85;7
float rkd = 0;//0.17;

PID FrightPID(&cur_Rrpm, &new_Rrpm, &FRrpm, frkp, frki, frkd, DIRECT);
PID FleftPID(&cur_Lrpm, &new_Lrpm, &FLrpm, flkp, flki, flkd, DIRECT);

PID rightPID(&cur_Rrpm, &new_Rrpm, &Rrpm, rkp, rki, rkd, DIRECT);
PID leftPID(&cur_Lrpm, &new_Lrpm, &Lrpm, lkp, lki, lkd, DIRECT);

PID turnPID(&lencoder, &output,&rencoder, 0.5,0,0, DIRECT);

/*---------------------------Flags----------------------------------------------*/
boolean s = true;
boolean fp = false;
boolean go ; // for forward motion
boolean r; // for turning right
boolean l; //for turning left

/*-----------------------------ticks--------------------------------------------*/
int tick = 0;
int done;
volatile int grid;//amount of blocks to move forward(distance = grid x 10)

int fcount = 0;

char command; // for 

void start(boolean *s) // use flag and must use start after calibration
{
  if (s == true)
  {
   for(int i=0;i<250;i++)
       md.setSpeeds(i,i);
    s = false;  
  }
}

void moveBackward(int grid)
{  
   tick = 0;
   Lspeed = -250;
   Rspeed = -250;
   done = 0;

   while(done != 1)
   {
   // in a loop
   md.setSpeeds(-Lspeed, -Rspeed);
   Lpwm = pulseIn(13,HIGH); //wheel 123
   Rpwm = pulseIn(5,HIGH);  //wheel 107
   cur_Lrpm = 60.000/(562.250*(Lpwm/1000000)*2.00);
   cur_Rrpm = 60.00/(562.250*(Rpwm/1000000)*2.00);
   leftPID.Compute();
   rightPID.Compute();
   /*----------------for tick count--------------------------------*/
   if(Lpwm > 720)
   {
     tick++;
   }
   
  if(tick == 127 ) // 10 - 8  // 20 - 15// 30 - 21 //40 - 28//50 - 35(--) //60 -42 //70 - 47  // 80 - 55 //90 - 63 // 100 - 70(76) // 110 - (76) //120 - 82 // 130 - 88 // 140 - 94 // 150 - 100 // 160-106 // 170-112 // 180-120`   2 - ticks - 10 degress for left turn,5 - ticks 30 6-ticks - 40,7-50 
   {
     done++; // 45 - 5//8 - 60 , 9 - 70,10 -80, 11 - 90, 12 - 100, 13- 110, 14-120, 21 -> 180// 37 - 360,//720 ->75// 42 -> 270, 1080 -> 112
     tick = 0;
     //rtick = 0;
   } 
   if(done == 1) 
   {
     finish = true;
     md.setBrakes(-350,-350); // 400 for going forward
     delay(100);
     fcount ++;
   }
   /*------------------------end of tick count---------------------------*/
   
    Rspeed = (3.125*new_Lrpm) + 9.12000;
    Lspeed = (2.78000*new_Rrpm)+13.33000;
   }
    
}

/* FASTEST PATH - */


void FastForward(int grid = 1)
{  
 /* if(fcount == 5)
  {
    fcount = 0;
    forward(0.4);
  }*/
   int a = 0;
   tick = 0;
   Lspeed = 350;
   Rspeed = 350;
   done = 0;
   while(done != 1)
   {
   // in a loop
   md.setSpeeds(Lspeed , Rspeed);
   Lpwm = pulseIn(13,HIGH); //wheel 123
   Rpwm = pulseIn(5,HIGH);  //wheel 107
   cur_Lrpm = 60.000/(562.250*(Lpwm/1000000)*2.00);
   cur_Rrpm = 60.00/(562.250*(Rpwm/1000000)*2.00);

  // Serial.print(cur_Lrpm);
  // Serial.print("              ");
  // Serial.print(cur_Rrpm);
 // Serial.print(Lpwm);
  // Serial.println();
 
   FleftPID.Compute();
   FrightPID.Compute();
/*-----------------------------*/
   if (Lpwm > 500)
   {
     tick++;
   }// 105 for one block
   //105 //127
   
   if(!fp) {a = 128;}
   else {a = 135;}; 
   
  if(tick == grid*a) //128  // 10 - 8  // 20 - 15// 30 - 21 //40 - 28//50 - 35(--) //60 -42 //70 - 47  // 80 - 55 //90 - 63 // 100 - 70(76) // 110 - (76) //120 - 82 // 130 - 88 // 140 - 94 // 150 - 100 // 160-106 // 170-112 // 180-120`   2 - ticks - 10 degress for left turn,5 - ticks 30 6-ticks - 40,7-50 
   {
     done++; // 45 - 5//8 - 60 , 9 - 70,10 -80, 11 - 90, 12 - 100, 13- 110, 14-120, 21 -> 180// 37 - 360,//720 ->75// 42 -> 270, 1080 -> 112
    tick = 0;
    
   } 
   if(done == 1) 
   {
     md.setBrakes(360, 360); // 400 for going forward
     //Serial.println("done moving forward");
     delay(100);
     finish = true;
   }
   /*------------------------end of tick count---------------------------*/
   
   Lspeed = (3.125*new_Rrpm) + 9.12000;
   Rspeed = (2.78000*new_Lrpm)+13.33000;
   }

}


void moveForward(int grid)
{
  double right = rencoder;
  double left = lencoder;
  output = 0;
  lencoder = 0; 
  rencoder = 0;
  double target = 0;
  target = grid *550;//need to calculate   

  while(lencoder<target)
  {
    turnPID.Compute();
    md.setSpeeds((390+output),(400+output));
  }

  md.setBrakes(390,400);
  finish = true;
}





void sensor_distance() 
{
   
  Serial.print("P");
  int med = 15;
//  RunningMedian samples = RunningMedian(10) ;
//  for(int i =0; i<6;i++)
//  {
//    if(i=0)
//    {
//    for(int a=0;a<10;a++)
//    {
//    float dist = sensor[0].getDistance(false);
//    samples.add(dist);
//    }
//    float h = samples.getMedian();
//    Serial.print(h);
//   
//    }
//    else 
//    Serial.print(sensor[i].getDistance(false));
//    Serial.print("-");
//  }
//
//  Serial.println();

RunningMedian samples0 = RunningMedian(15) ;
RunningMedian samples1 = RunningMedian(15) ;
RunningMedian samples2 = RunningMedian(6) ;
RunningMedian samples3 = RunningMedian(6) ;
RunningMedian samples4 = RunningMedian(6) ;
RunningMedian samples5 = RunningMedian(15) ;

  for(int i=0; i<15;i++)
  {   
      samples2.add(sensor[2].getDistance(false));
      samples3.add(sensor[3].getDistance(false));
      samples4.add(sensor[4].getDistance(false));
      samples0.add(sensor[0].getDistance(false));
      samples1.add(sensor[1].getDistance(false));
      samples5.add(sensor[5].getDistance(false));
  }
 
    float h0 = samples0.getMedian();
    float h1 = samples1.getMedian();
    float h2 = samples2.getMedian();
    float h3 = samples3.getMedian();
    float h4 = samples4.getMedian();
    float h5 = samples5.getMedian(); 
          //if(h<29)
                   
         // if((h>=20.00) && (h<29))
         // h = 20;

         
          if ((h0>=29) && (h0<=37))
          h0 = 30;
          else if ((h0>=37) && (h0<=46))
          h0 = 40;
          else if((h0>46) && (h0<=66))
          h0 = 50;
          else if(h0>66)
          h0 = 80;

        Serial.print(h0);
        Serial.print("-");
     

         /*if(i==1)---old
        {
          if((h>10) && (h<=17))
          h = 10;
          else if ((h>15) && (h<=24))
          h = 20;
          else if ((h>32) && (h<3))
          h = 30;
          else if(h>=32) 
          h = 80;
        }*/
        
          if(h1<=15)
          h1 = 9;
          else if((h1>15) && (h1<22))
          h1 = 10;
          else if ((h1>=22) && (h1<=31))
          h1 = 20;
          else if((h1>31) && (h1<34))
          h1 = 30;
          else if(h1>=34)
          h1 = 80;
        
       Serial.print(h1);
       Serial.print("-");
   
          if(h2<18)
          h2 = 9;
          else if((h2>=18) && (h2<25))
          h2 = 10;
          else if ((h2>=25) && (h2<37))
          h2 = 20;
          else if (h2>=37) 
           h2 = 80;

        Serial.print(h2);
        Serial.print("-");        

          if(h3<12)
           h3 = 9;
           else if((h3>=12) && (h3<18))
            h3 = 10;
             else if ((h3>=18) && (h3<31))
              h3 = 20;
                else if ((h3>=31) && (h3<43))
                h3 = 30;
                  else if(h3>43)
                  h3 = 80;
                  
         Serial.print(h3);
         Serial.print("-");      
         
          if(h4<17)
          h4 = 9;
          else if((h4>=17) && (h4<25))
          h4 = 10;
          else if ((h4>=25) && (h4<36))
          h4 = 20;
          else if (h4>=36) 
           h4 = 80;

        Serial.print(h4);
        Serial.print("-");

       /* if(i==4)
        {
          if((h>=10) && (h<21))
          h = 10;
          else if ((h>=21) && (h<31))
          h = 20;
          else if ((h>=31) && (h<=43))
          h = 30;
          
          else if(h>43)
          h = 80;
        } */

          if(h5<=15)
          h5 = 9;
          else if((h5>15) && (h5<22))
          h5 = 10;
          else if ((h5>=22) && (h5<=32))
          h5 = 20;
          else if((h5>=32) && (h5<40))
          h5 = 30;
          else if(h5>=40)
          h5 = 80;
        
     Serial.print(h5);
     Serial.print("-");
     Serial.println();    
                                                                                                                             
  finish =true;

}


/*void turnLeft(int angle)
{  // good for 90 degrees
  md.setSpeeds(250,-250);
  delay(7.9*angle); //8.35 //7.9 - OLD
  md.setBrakes(400,400);
  delay(1000);
  finish = true;
}*/

/*void turnRight(int angle)
{
  md.setSpeeds(-250,250);
  delay(7.9*angle);//8.2//7.9 - OLDX
  md.setBrakes(400,400);
  delay(1000);  
  finish = true;
}*/

void turnLeft(int angle)
{
  double right = rencoder;
  double left = lencoder;
  output = 0;
  lencoder = 0; 
  rencoder = 0;
  double target = 0;
  if(angle <90) 
    target = angle*5;//8.86 //NEW -9.1//8.8775//8.86//9
  else if(angle == 90)
     target = angle*8.9;
  else if (angle <=180) 
    target = angle *9.275;   

  while(lencoder<target)
  {
    turnPID.Compute();
    md.setSpeeds((390+output),-(400+output));
  }

  md.setBrakes(390,-400);
  finish = true;
  
}


void turnRight(int angle)
{
  double right = rencoder;
  double left = lencoder;
  output= 0;
  lencoder = 0; 
  rencoder = 0;
  double target = 0;
   if(angle<90)
     target = angle*5;
   else if(angle==90) 
    target = angle*8.9; //8.86//9.1-NEW//9//8.82 

  while(lencoder<target)
  {
    turnPID.Compute();
    md.setSpeeds(-(390+output),(400+output));
  }

  md.setBrakes(-390,400);
  finish = true;
  
}

void stopwheels()
{
  md.setSpeeds(0,0);
} 


void cal_right(int angle)
{
  double right = rencoder;
  double left = lencoder;
  lencoder = 0; 
  rencoder = 0;
  double target = 0;
  target = angle*5.3;   
  while(lencoder<target)
  {
    md.setSpeeds(-250,250);
  }

  md.setBrakes(-400,400);
  finish = true;
  
  
}

void cal_left(int angle)
{
  double right = rencoder;
  double left = lencoder;
  lencoder = 0; 
  rencoder = 0;
  double target = 0;
  target = angle *5.8;   
  while(lencoder<target)
  {
    md.setSpeeds(250,-250);
  }

  md.setBrakes(400,-400);
  finish = true;
  
}


//boolean acc = false;

void dist_calibrate()
{  
   double distance = (S2.getDistance(false) +S4.getDistance(false)) / 2;
  //   double distance = S3.getDistance(false);
  // Serial.println(S2.getDistance(false));
   Serial.println(S2.getDistance(false));
   
   Serial.println(S4.getDistance(false));
//   double distance = S3.getDistance(false);
  double diff = distance - 12.00; 

  //Serial.println(distance);

 // diff = fmod(diff,10); // to calibrate against any wall.
  
  //Serial.println(diff);
 
   if (abs(diff) > 0.5)
  {
    if(diff >0)
    {      
     forward(diff);
    }

    else if(diff<0)
    {
      backward(abs(diff));
    }
  }
  finish = true;

}

void forward(double dist)
{
  double right = rencoder;
  double left = lencoder;
  lencoder = 0; 
  rencoder = 0;
  double target = dist*45;

  while(lencoder<target)
  {
    md.setSpeeds(200,200);
  }

  md.setBrakes(400,400);
  finish = true;
}


void backward(double dist)
{
   double right = rencoder;
  double left = lencoder;
  lencoder = 0; 
  rencoder = 0;
  double target = dist*37;

  while(lencoder<target)
  {
    md.setSpeeds(-200,-200);
  }

  md.setBrakes(-400,-400);
  finish = true;

}

void calibrate()
{ 
   RunningMedian samples1 = RunningMedian(5) ;
   samples1.add(S3.getDistance(false));
   float h0 = samples1.getMedian();

   RunningMedian samples2 = RunningMedian(5) ;
   samples2.add(S5.getDistance(false));
   float h1 = samples2.getMedian();
   
  double frontLeft =  h0 -0.75;
  double frontRight = h1;
  
 // Serial.println(S3.getDistance(false));
 // Serial.println(S5.getDistance(false));
 // Serial.println();

  if((frontLeft >= 5 )&&( frontLeft <=25) && (frontRight >= 5) && (frontRight<=25))
  {

  //Serial.print("Left:");
  Serial.println(frontLeft);
  //Serial.print("Right");
  Serial.println(frontRight);
    
    double theta = 0.0;
    double rad2deg = 180/3.14159;
    theta = asin((abs(frontLeft -frontRight))/15.75) * rad2deg;//15.75

   // Serial.print("theta");
   Serial.println(theta);
      if(frontLeft - frontRight > 0.1)
      {   
        cal_right(theta);
      }

      if(frontRight - frontLeft >0.1)
      {
        cal_left(theta);
      }

  //////////////////
 
  frontLeft = h0-0.75;
  frontRight = h1;
  


//     Serial.println(S3.getDistance(false));
//  Serial.println(S5.getDistance(false));
//  Serial.println();


  /*Serial.print("Left:");
  Serial.println(frontLeft);
  Serial.print("Right");
  Serial.println(frontRight);*/
    
     theta = 0.0;  
     theta = asin((abs(frontLeft -frontRight))/15.75) * rad2deg;

 // Serial.print("theta");
 // Serial.println(theta);
  
  
  if(frontLeft - frontRight > 0.1)
  {   
     cal_right(theta);
  }

  if(frontRight - frontLeft >0.1)
  {
    cal_left(theta);
  }

  

// ----------------------distance---------------------
   backward(2.5);
   delayMicroseconds(100);
   md.setSpeeds(0,0);
   //Serial.println(S2.getDistance(false));
   //Serial.println(S3.getDistance(false));
   //Serial.println(S4.getDistance(false));
  double diff1 = S3.getDistance(false) - 11.4;//12.8 
  double diff2 = S5.getDistance(false) - 12.4;//13.2
  double diff = (diff1+diff2)/2;
   //  Serial.println(diff1);
   //  Serial.println(diff2);
   //  Serial.println(diff);
   if (abs(diff) > 0.5)
  {
    if(diff >0)
    {      
     forward(diff);
    }
    else if(diff<0)
    {
      backward(abs(diff));
    }
  }

  delayMicroseconds(100);
/// -------------second angle calibration-----------------------  


  double frontLeft = h0 -0.75;
  double frontRight = h1;
//  Serial.println(S3.getDistance(false));
//  Serial.println(S5.getDistance(false));
//  Serial.println();

  theta = 0.0;  
  theta = asin((abs(frontLeft -frontRight))/15.75) * rad2deg;  //16.5 for modified Sharp  //15.75
  if(frontLeft - frontRight > 0.1)
  {   
     cal_right(theta);
  }

  if(frontRight - frontLeft >0.1)
  {
    cal_left(theta);
 
  }
    
 delayMicroseconds(100);

//  Serial.println(S3.getDistance(false));
//  Serial.println(S5.getDistance(false));
//  Serial.println();


 
  frontLeft = h0-0.75;
  frontRight = h1;
  theta = 0.0;  
  theta = asin((abs(frontLeft -frontRight))/15.75) * rad2deg;  
  if(frontLeft - frontRight > 0.1)
  {   
     cal_right(theta);
  }

  if(frontRight - frontLeft >0.1)
  {
    cal_left(theta);
  }
  

  } 
  

  finish = true;
}

 void rencoderInc()
 {
  rencoder++;
 }

 void lencoderInc()
 {
  lencoder++;
 }


void setup() 
{
  Serial.begin(115200);
  md.init();
  pinMode(R_ENCODER, INPUT);
  pinMode(L_ENCODER, INPUT);
  rightPID.SetMode(AUTOMATIC);
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetOutputLimits(67.68,80);
  leftPID.SetOutputLimits(73.07,80.27);
  leftPID.SetSampleTime(5);
  rightPID.SetSampleTime(5);

  FrightPID.SetMode(AUTOMATIC);
  FleftPID.SetMode(AUTOMATIC);
  FrightPID.SetOutputLimits(67.68,110);
  FleftPID.SetOutputLimits(73.07,110);
  FleftPID.SetSampleTime(5);
  FrightPID.SetSampleTime(5);
  turnPID.SetOutputLimits(-50,50);
  turnPID.SetMode(AUTOMATIC);

 attachPCINT(digitalPinToPCINT(R_ENCODER),rencoderInc,HIGH);
 attachPCINT(digitalPinToPCINT(L_ENCODER),lencoderInc,HIGH);
   
  go = true;
  finish = false;
 fp = false;
// fp = true;
  grid = 1;
}

String Buff;
boolean readCommand = false;
int i = 0;
void loop() 
{ 
//  
//   Serial.print(S3.getDistance(false));
//  Serial.print("                      ");
//  Serial.print(S5.getDistance(false));
//  Serial.println();
//
// delay(100);
//  
//FastForward(4*1.23);
//delay(500); 
//turnRight(90);
//delay(100);
//FastForward(4*1.23);
cal_right(4);
// Buff = "F5RF6LF2RF3LF5RF3LF5";
   if(fp)
  {
    if(Serial.available()>0)
   {
 Buff = Serial.readString();
     // Serial.print("Buff is ");
     // Serial.println(Buff);
     
   }
    command = Buff.charAt(i);
   // Serial.println("Current Command is: ");
   // Serial.println(command);
    if(command == 'F')
    {
      i++;
      grid = (int(Buff.charAt(i))) -48;
      
     // Serial.println(Buff.charAt(++i));
     // Serial.print("Grid  ");
     // Serial.println(grid);
    }
   }
   
   else
   if(Serial.available()>0)
   {
     command = Serial.read();
   }
 
    switch(command)
    {
    //  case 'F': moveForward(22); break;
      case 'F': if(!fp)moveForward(1);else FastForward(grid*1.07); break;
      case 'B': turnLeft(180);moveForward(22);turnLeft(180); break;
      case 'L': if(fp)turnLeft(93);else turnLeft(90); break; //82
      case 'R': turnRight(90);break; //87
      case 'U': turnLeft(180);break;
      case 'C': calibrate();break;
      case 'c': dist_calibrate(); break;
      case 'S': stopwheels();break;
      case 'd': sensor_distance(); finish = false;command = '\0'; break;
      case 'X': fp = true;finish = false; command = '\0'; break; 
      default: break;
    }


 if(finish == true)
  {
    if(!fp)
    sensor_distance();
    else
    {
   //   Serial.println(Buff);
      md.setBrakes(400,400);
      delay(500);
    i++;
    delayMicroseconds(1);
   // if (i == buffsize)
   // i = 0;
     }
    finish = false;
    command = '\0'; 
  }
  
}
