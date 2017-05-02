#include "Leg.h"
#include "Ultrasonic.h"

int del = 20;             //Delay length after each step in legs moving cycles
int pace = 20;            //Nummer of steps in legs moving cycles
int HexHeight = 30;       //Hex body (legs origins) height from ground 
int LegsLiftHeight = 25;  //Max height on which legs lifts 
int LegsOffset = 60;      //Offset legs end from legs origin

boolean automove = false; //Automatic movement flag

static Leg Legs[6];       //Array of leg: 0,1,2 - left legs; 3,4,5 - right legs

Ultrasonic ultrasonic(38, 37); //Setting pins for HR-SR04

void printPoint(String S, int i, Point P) //Debugging function
{
  Serial.print(S);
  Serial.print("["); 
  Serial.print(i); 
  Serial.print("] x="); 
  if (P.x>=0) {Serial.print(" ");}
  Serial.print(P.x); 
  Serial.print(" y="); 
  if (P.y>=0) {Serial.print(" ");}
  Serial.print(P.y); 
  Serial.print(" z="); 
  if (P.z>=0) {Serial.print(" ");}
  Serial.println(P.z);
}

float lift(int i, int pace) //Calculate current leg lift height by parabolic function
{
  return ((-2.0 / pace) * sqr(i - pace / 2) + (pace / 2)) * (2.0 / pace);
}

void config() //Legs configuration
{
  Serial.println("config: start");

  Legs[0].attach(27,28,29);
  Legs[1].attach(14,15,16);
  Legs[2].attach(7,6,5);
  
  Legs[3].attach(48,47,46);
  Legs[4].attach(61,62,63);
  Legs[5].attach(54,55,56);

  Legs[0].config(0,90,60,187+5,45); 
  Legs[1].config(1,90,64,191+3,90);
  Legs[2].config(2,115,53,198,135);
  
  Legs[3].config(3,85,118,-7,45); 
  Legs[4].config(4,79,122+4,-4+5,90);
  Legs[5].config(5,86,134,-3,135); 

  Legs[0].configTurn(-72.12,-15.65);
  Legs[1].configTurn(-62,0);
  Legs[2].configTurn(-72.12,15.65);
  
  Legs[3].configTurn(-72.12,-15.65); 
  Legs[4].configTurn(-62,0);
  Legs[5].configTurn(-72.12,15.65);

  Serial.println("config: end");
}
 
//Turn around Hex origin (number of steps, turn direction (1 == right or -1 == left), turn angle on each step (deg), not return legs in initial positon);
void turn (int Steps, int direction, float turnAngleD, boolean Unend) 
{
  Serial.println("turn: start");

  float TurnAngleD;                                         //Turn angle on each step
  if (turnAngleD == 0 || turnAngleD > 30) TurnAngleD = 25;  //Turn angle limits
  else TurnAngleD = turnAngleD;

  int Direction;                      //Turn direction
  if (direction != -1) Direction = 1;
  else Direction = -1;

  Point P0[6];  //Array of initial legs positions
  Point P1[6];  //Array of positions of legs turned at half TurnAngle
  Point P2[6];  //Array of positions of legs turned at half -TurnAngle
  Point A;      //Current point 
  float angle;  //Current point
  int end = 0;  //Type of turn end    
   
  for(int i=0; i<=5; i++) //Filling P0, P1, P2 arrays
  {    
    P0[i] = Legs[i].getCurrentPos();
    angle = -Direction * deg2rad(TurnAngleD / 2);
    Legs[i].turnPoint(LegsOffset,angle,P1[i].x,P1[i].y);  
    Legs[i].turnPoint(LegsOffset,-angle,P2[i].x,P2[i].y); 
    P1[i].z = -HexHeight;
    P2[i].z = -HexHeight;
  } 
  
  if (!(P0[1] == P1[1] && P0[3] == P1[3] && P0[5] == P1[5]) || !Unend) //Check if first 3 legs are already in turn start position
  {
    for(int i=1; i<=pace; i++) //Moving first 3 legs for turn start position 
    {
      A.z = -HexHeight + LegsLiftHeight * lift(i,pace);

      A.x = P0[1].x + (P2[1].x - P0[1].x) * i / pace;
      A.y = P0[1].y + (P2[1].y - P0[1].y) * i / pace;
      Legs[1].reach(A);

      A.x = P0[3].x + (P1[3].x - P0[3].x) * i / pace;
      A.y = P0[3].y + (P1[3].y - P0[3].y) * i / pace;
      Legs[3].reach(A);

      A.x = P0[5].x + (P1[5].x - P0[5].x) * i / pace;
      A.y = P0[5].y + (P1[5].y - P0[5].y) * i / pace;
      Legs[5].reach(A);

      delay(del);     
    }
  } 
 
  for(int i=1; i<=pace; i++) 
  {
    //Turn Hex body 1 (first 3 legs)
    angle = -Direction * deg2rad(-TurnAngleD / 2 + (TurnAngleD) * i / pace);
    A.z = -HexHeight;  
    
    Legs[1].turnPoint(LegsOffset,angle,A.x,A.y);
    Legs[1].reach(A);

    Legs[3].turnPoint(LegsOffset,-angle,A.x,A.y);
    Legs[3].reach(A);

    Legs[5].turnPoint(LegsOffset,-angle,A.x,A.y);
    Legs[5].reach(A);
    
    delay(del);

    A.z = -HexHeight + LegsLiftHeight * lift(i,pace);

    if(Steps == 1 && !Unend) //If steps only 1 just lift second 3 legs
    {
      A.x = P0[0].x;
      A.y = P0[0].y;
      Legs[0].reach(A);

      A.x = P0[2].x;
      A.y = P0[2].y;
      Legs[2].reach(A);

      A.x = P0[4].x;
      A.y = P0[4].y;
      Legs[4].reach(A);
      
      end = 2;
    } 
    else //Steps more than 1 move second 3 legs in turn start position
    {    
      A.x = P0[0].x + (P2[0].x - P0[0].x) * i / pace;
      A.y = P0[0].y + (P2[0].y - P0[0].y) * i / pace;
      Legs[0].reach(A);

      A.x = P0[2].x + (P2[2].x - P0[2].x) * i / pace;
      A.y = P0[2].y + (P2[2].y - P0[2].y) * i / pace;
      Legs[2].reach(A);

      A.x = P0[4].x + (P1[4].x - P0[4].x) * i / pace;
      A.y = P0[4].y + (P1[4].y - P0[4].y) * i / pace;
      Legs[4].reach(A);
    } 
  }
 
  for(int j=2; j<=Steps;)
  {  
    for (int i=1; i<=pace; i++)
    {
      //Turn Hex body 2 (second 3 legs)
      A.z = -HexHeight;
      angle = -Direction * deg2rad(TurnAngleD / 2 - (TurnAngleD) * i / pace);
      
      Legs[0].turnPoint(LegsOffset,-angle,A.x,A.y);
      Legs[0].reach(A);

      Legs[2].turnPoint(LegsOffset,-angle,A.x,A.y);
      Legs[2].reach(A);

      Legs[4].turnPoint(LegsOffset,angle,A.x,A.y);
      Legs[4].reach(A);

      A.z = -HexHeight + LegsLiftHeight * lift(i,pace);

      if ((Steps - j) > 0 || Unend) //If there are more steps move first 3 legs in turn start position
      {
        Legs[1].turnPoint(LegsOffset,angle,A.x,A.y);
        Legs[1].reach(A);

        Legs[3].turnPoint(LegsOffset,-angle,A.x,A.y);
        Legs[3].reach(A);

        Legs[5].turnPoint(LegsOffset,-angle,A.x,A.y);
        Legs[5].reach(A);
        
        delay(del);
      }
      else //There are no more steps move first 3 legs in initial position
      {
        A.x = P1[1].x + (P0[1].x - P1[1].x) * i / pace;
        A.y = P1[1].y + (P0[1].y - P1[1].y) * i / pace;
        Legs[1].reach(A);

        A.x = P2[3].x + (P0[3].x - P2[3].x) * i / pace;
        A.y = P2[3].y + (P0[3].y - P2[3].y) * i / pace;
        Legs[3].reach(A);

        A.x = P2[5].x + (P0[5].x - P2[5].x) * i / pace;
        A.y = P2[5].y + (P0[5].y - P2[5].y) * i / pace;
        Legs[5].reach(A);

        delay(del);
        end = 1;
      }
    }
    
    if (end == 1) break;

    for (int i=1; i<=pace; i++)
    {
      //Turn Hex body 1 (first 3 legs)
      A.z = -HexHeight;
      angle = - Direction * deg2rad(TurnAngleD / 2 - (TurnAngleD) * i / pace);
      
      Legs[1].turnPoint(LegsOffset,-angle,A.x,A.y);
      Legs[1].reach(A);

      Legs[3].turnPoint(LegsOffset,angle,A.x,A.y);
      Legs[3].reach(A);

      Legs[5].turnPoint(LegsOffset,angle,A.x,A.y);
      Legs[5].reach(A);
      
      A.z = -HexHeight + LegsLiftHeight * lift(i,pace);

      if ((Steps - (j + 1)) > 0 || Unend) //If there are more steps move second 3 legs in turn start position
      {
        Legs[0].turnPoint(LegsOffset,angle,A.x,A.y);
        Legs[0].reach(A);

        Legs[2].turnPoint(LegsOffset,angle,A.x,A.y);
        Legs[2].reach(A);

        Legs[4].turnPoint(LegsOffset,-angle,A.x,A.y);
        Legs[4].reach(A);

        delay(del);
      }
      else //There are no more steps move second 3 legs in initial position
      {
        A.x = P1[0].x + (P0[0].x - P1[0].x) * i / pace;
        A.y = P1[0].y + (P0[0].y - P1[0].y) * i / pace;
        Legs[0].reach(A);

        A.x = P1[2].x + (P0[2].x - P1[2].x) * i / pace;
        A.y = P1[2].y + (P0[2].y - P1[2].y) * i / pace;
        Legs[2].reach(A);

        A.x = P2[4].x + (P0[4].x - P2[4].x) * i / pace;
        A.y = P2[4].y + (P0[4].y - P2[4].y) * i / pace;
        Legs[4].reach(A); 

        delay(del);
        end = 2;
      }
    } 
    j = j + 2;
  }

  if (end == 1)
  {
    for(int i=1; i<=pace; i++) //Move second 3 legs in base position
    {
      A.z = -HexHeight + LegsLiftHeight * lift(i,pace);

      A.x = P1[0].x + (LegsOffset - P1[0].x) * i / pace;
      A.y = P1[0].y + (0 - P1[0].y) * i / pace;
      Legs[0].reach(A);

      A.x = P1[2].x + (LegsOffset - P1[2].x) * i / pace;
      A.y = P1[2].y + (0 - P1[2].y) * i / pace;
      Legs[2].reach(A);

      A.x = P2[4].x + (LegsOffset - P2[4].x) * i / pace;
      A.y = P2[4].y + (0 - P2[4].y) * i / pace;
      Legs[4].reach(A);

      delay(del);  
    }  
  }

  if (end == 2)
  {
    for(int i=1; i<=pace; i++) //Move first 3 legs in base position
    {
      A.z = -HexHeight + LegsLiftHeight * lift(i,pace);

      A.x = P1[1].x + (LegsOffset - P1[1].x) * i / pace;
      A.y = P1[1].y + (0 - P1[1].y) * i / pace;
      Legs[1].reach(A);

      A.x = P2[3].x + (LegsOffset - P2[3].x) * i / pace;
      A.y = P2[3].y + (0 - P2[3].y) * i / pace;
      Legs[3].reach(A);

      A.x = P2[5].x + (LegsOffset - P2[5].x) * i / pace;
      A.y = P2[5].y + (0 - P2[5].y) * i / pace;
      Legs[5].reach(A); 

      delay(del);
    } 
  }
  Serial.println("turn: end");
}

//Move (number of steps, moving direction angle (0 == forward, 180 == backward), step length, not return legs in initial positon);
void move (int Steps, int DirectionAngleD, int steplength, boolean Unend)
{
  Serial.println("move: start");

  int StepLength;
  if (steplength <= 0) StepLength = 40;
  else StepLength = steplength;
  
  Point P0[6];  //Array of initial legs positions
  Point P1[6];  //Array of positions of legs turned at half StepLength
  Point P2[6];  //Array of positions of legs turned at half -StepLength
  Point A;      //Current point 
  int end = 0;  //Type of move end  

  for(int i=0; i<=5; i++) //Filling P0, P1, P2 arrays
  {
    Legs[i].transfer(LegsOffset,StepLength,DirectionAngleD,P1[i].x,P1[i].y,P2[i].x,P2[i].y);
    P1[i].z = -HexHeight;
    P2[i].z = -HexHeight;
    P0[i] = Legs[i].getCurrentPos();
  }

  if (!(P0[1] == P1[1] && P0[3] == P1[3] && P0[5] == P1[5]) || !Unend) //Check if first 3 legs are already in move start position
  {
    for(int i=1; i<=pace; i++) //Moving first 3 legs for move start position
    {
      A.z = -HexHeight + LegsLiftHeight * lift(i,pace);
      
      A.x = P0[1].x + (P1[1].x - P0[1].x) * i / pace;
      A.y = P0[1].y + (P1[1].y - P0[1].y) * i / pace;
      Legs[1].reach(A);

      A.x = P0[3].x + (P1[3].x - P0[3].x) * i / pace;
      A.y = P0[3].y + (P1[3].y - P0[3].y) * i / pace;
      Legs[3].reach(A);

      A.x = P0[5].x + (P1[5].x - P0[5].x) * i / pace;
      A.y = P0[5].y + (P1[5].y - P0[5].y) * i / pace;
      Legs[5].reach(A); 

      delay(del);     
    }
  }

  for(int i=1; i<=pace; i++) 
  {
    //Move Hex body 1 (first 3 legs)
    A.z = -HexHeight;

    A.x = P1[1].x + (P2[1].x - P1[1].x) * i / pace;
    A.y = P1[1].y + (P2[1].y - P1[1].y) * i / pace;
    Legs[1].reach(A);

    A.x = P1[3].x + (P2[3].x - P1[3].x) * i / pace;
    A.y = P1[3].y + (P2[3].y - P1[3].y) * i / pace;
    Legs[3].reach(A);

    A.x = P1[5].x + (P2[5].x - P1[5].x) * i / pace;
    A.y = P1[5].y + (P2[5].y - P1[5].y) * i / pace;
    Legs[5].reach(A);

    A.z = -HexHeight + LegsLiftHeight * lift(i,pace);

    if(Steps == 1 && !Unend) //If steps only 1 just lift second 3 legs
    {
      A.x = P0[0].x;
      A.y = P0[0].y;
      Legs[0].reach(A);

      A.x = P0[2].x;
      A.y = P0[2].y;
      Legs[2].reach(A);

      A.x = P0[4].x;
      A.y = P0[4].y;
      Legs[4].reach(A);
      
      delay(del);
      end = 2;   
    }
    else //Steps more than 1 move second 3 legs in turn move position
    {
      A.x = P0[0].x + (P1[0].x - P0[0].x) * i / pace;
      A.y = P0[0].y + (P1[0].y - P0[0].y) * i / pace;
      Legs[0].reach(A);

      A.x = P0[2].x + (P1[2].x - P0[2].x) * i / pace;
      A.y = P0[2].y + (P1[2].y - P0[2].y) * i / pace;
      Legs[2].reach(A);

      A.x = P0[4].x + (P1[4].x - P0[4].x) * i / pace;
      A.y = P0[4].y + (P1[4].y - P0[4].y) * i / pace;
      Legs[4].reach(A); 

      delay(del);  
    }
  }

  for(int j=2; j<=Steps;)
  {
    for(int i=1; i<=pace; i++)
    {
      //Move Hex body 2 (second 3 legs)
      A.z = -HexHeight;

      A.x = P1[0].x + (P2[0].x - P1[0].x) * i / pace;
      A.y = P1[0].y + (P2[0].y - P1[0].y) * i / pace;
      Legs[0].reach(A);

      A.x = P1[2].x + (P2[2].x - P1[2].x) * i / pace;
      A.y = P1[2].y + (P2[2].y - P1[2].y) * i / pace;
      Legs[2].reach(A);

      A.x = P1[4].x + (P2[4].x - P1[4].x) * i / pace;
      A.y = P1[4].y + (P2[4].y - P1[4].y) * i / pace;;
      Legs[4].reach(A); 

      A.z = -HexHeight + LegsLiftHeight * lift(i,pace);

      if ((Steps - j) > 0 || Unend) //If there are more steps move first 3 legs in move start position
      {
        A.x = P2[1].x + (P1[1].x - P2[1].x) * i / pace;
        A.y = P2[1].y + (P1[1].y - P2[1].y) * i / pace;
        Legs[1].reach(A);

        A.x = P2[3].x + (P1[3].x - P2[3].x) * i / pace;
        A.y = P2[3].y + (P1[3].y - P2[3].y) * i / pace;
        Legs[3].reach(A);

        A.x = P2[5].x + (P1[5].x - P2[5].x) * i / pace;
        A.y = P2[5].y + (P1[5].y - P2[5].y) * i / pace;
        Legs[5].reach(A);  
        delay(del);    
      }
      else //There are no more steps move second 3 legs in initial position
      {
        A.x = P2[1].x + (P0[1].x - P2[1].x) * i / pace;
        A.y = P2[1].y + (P0[1].y - P2[1].y) * i / pace;
        Legs[1].reach(A);

        A.x = P2[3].x + (P0[3].x - P2[3].x) * i / pace;
        A.y = P2[3].y + (P0[3].y - P2[3].y) * i / pace;
        Legs[3].reach(A);

        A.x = P2[5].x + (P0[5].x - P2[5].x) * i / pace;
        A.y = P2[5].y + (P0[5].y - P2[5].y) * i / pace;
        Legs[5].reach(A);  
        
        delay(del); 
        end = 1;
      }
    }

    if (j == Steps) break;

    for(int i=1; i<=pace; i++)
    {
      //Move Hex body 1 (first 3 legs)
      A.z = -HexHeight;

      A.x = P1[1].x + (P2[1].x - P1[1].x) * i / pace;
      A.y = P1[1].y + (P2[1].y - P1[1].y) * i / pace;
      Legs[1].reach(A);

      A.x = P1[3].x + (P2[3].x - P1[3].x) * i / pace;
      A.y = P1[3].y + (P2[3].y - P1[3].y) * i / pace;
      Legs[3].reach(A);

      A.x = P1[5].x + (P2[5].x - P1[5].x) * i / pace;
      A.y = P1[5].y + (P2[5].y - P1[5].y) * i / pace;;
      Legs[5].reach(A);    

      A.z = -HexHeight + LegsLiftHeight * lift(i,pace);

      if ((Steps - (j+1)) > 0 || Unend) //If there are more steps move second 3 legs in move start position
      {
        A.x = P2[0].x + (P1[0].x - P2[0].x) * i / pace;
        A.y = P2[0].y + (P1[0].y - P2[0].y) * i / pace;
        Legs[0].reach(A);

        A.x = P2[2].x + (P1[2].x - P2[2].x) * i / pace;
        A.y = P2[2].y + (P1[2].y - P2[2].y) * i / pace;
        Legs[2].reach(A);

        A.x = P2[4].x + (P1[4].x - P2[4].x) * i / pace;
        A.y = P2[4].y + (P1[4].y - P2[4].y) * i / pace;
        Legs[4].reach(A);
        
        delay(del);
      }
      else //There are no more steps move second 3 legs in initial position
      {
        A.x = P2[0].x + (P0[0].x - P2[0].x) * i / pace;
        A.y = P2[0].y + (P0[0].y - P2[0].y) * i / pace;
        Legs[0].reach(A);

        A.x = P2[2].x + (P0[2].x - P2[2].x) * i / pace;
        A.y = P2[2].y + (P0[2].y - P2[2].y) * i / pace;
        Legs[2].reach(A);

        A.x = P2[4].x + (P0[4].x - P2[4].x) * i / pace;
        A.y = P2[4].y + (P0[4].y - P2[4].y) * i / pace;
        Legs[4].reach(A);
        
        delay(del);
        end = 2;        
      }
    }     
    j = j + 2;
  }

  if (Unend) end = 0;

  if (end == 1)
  {
    for(int i=1; i<=pace; i++) //Move second 3 legs in base position
    {   
      A.z = -HexHeight + LegsLiftHeight * lift(i,pace);
      
      A.x = P2[0].x + (LegsOffset - P2[0].x) * i / pace;
      A.y = P2[0].y + (0 - P2[0].y) * i / pace;
      Legs[0].reach(A);

      A.x = P2[2].x + (LegsOffset - P2[2].x) * i / pace;
      A.y = P2[2].y + (0 - P2[2].y) * i / pace;
      Legs[2].reach(A);

      A.x = P2[4].x + (LegsOffset - P2[4].x) * i / pace;
      A.y = P2[4].y + (0 - P2[4].y) * i / pace;
      Legs[4].reach(A);  
      
      delay(del);
    }      
  }

  if (end == 2)
  {
    for(int i=1; i<=pace; i++) //Move first 3 legs in base position
    {   
      A.z = -HexHeight + LegsLiftHeight * lift(i,pace);
      
      A.x = P2[1].x + (LegsOffset - P2[1].x) * i / pace;
      A.y = P2[1].y + (0 - P2[1].y) * i / pace;
      Legs[1].reach(A);

      A.x = P2[3].x + (LegsOffset - P2[3].x) * i / pace;
      A.y = P2[3].y + (0 - P2[3].y) * i / pace;
      Legs[3].reach(A);

      A.x = P2[5].x + (LegsOffset - P2[5].x) * i / pace;
      A.y = P2[5].y + (0 - P2[5].y) * i / pace;
      Legs[5].reach(A);  
      
      delay(del); 
    }
  }
  
  Serial.println("move: end");
}

void getdown()  //Get down on ground and stetch oun legs
{
  Serial.println("gedown: start");

  Point A;      //Current point
  Point P0[6];  //Array of initial legs positions
  
  for(int i=0; i<=5; i++) P0[i] = Legs[i].getCurrentPos(); //Filling array P0 Point A;
  
  for(int j=1; j<=pace; j++) //Get down (lift legs to 5 mm height)
  {
    for(int i=0; i<=5; i++)
    {
      A = P0[i];
      A.z = P0[i].z + (0 - P0[i].z) * j / pace;
      Legs[i].reach(A);
      delay(3);
    }
  }

  for(int j=1; j<=pace; j++) //Stretch out legs
  {
    for(int i=0; i<=5; i++)
    {
      A = P0[i];
      A.x = P0[i].x + (180 - P0[i].x) * j / pace;
      A.z = 0;
      Legs[i].reach(A);
      delay(3);
    }
  }

  Serial.println("gedown: end");
}

void getup() //Get up from ground
{
  Serial.println("getup: start");

  HexHeight = 30; //Write getup height

  Point A;      //Current point
  Point P0[6];  //Array of initial legs positions
  
  for(int i=0; i<=5; i++) P0[i] = Legs[i].getCurrentPos(); //Filling array P0
  
  if (!((P0[0].x == 60) && (P0[1].x == 60) && (P0[2].x == 60) && (P0[3].x == 60) && (P0[4].x == 60) && (P0[5].x == 60))) //If legs aren't in getup position
  {
    for(int j=1; j<=pace; j++) //Lift legs to 5 mm
    {
      for(int i=0; i<=5; i++)
      {
        A = P0[i];
        A.z = P0[i].z + (5 - P0[i].z) * j / pace;
        Legs[i].reach(A);
        delay(1);
      }
    }
    
    for(int j=1; j<=pace; j++) //Move legs to getup position
    {
      for(int i=0; i<=5; i++)
      {
        A.x = P0[i].x + (60 - P0[i].x) * j / pace;
        A.y = P0[i].y + (0 - P0[i].y) * j / pace;
        A.z = 5;
        Legs[i].reach(A);
        delay(3);
      }
    }
  }

  A.x = 60;
  A.y = 0;

  for(int j=1; j<=pace; j++) //Getup (lower legs to - HexHeight)
  {
    for(int i=0; i<=5; i++)
    {
      A.z = 5 - (5+HexHeight) * j / pace;
      Legs[i].reach(A);
      delay(1);
    }
  }

  Serial.println("getup: end");
}

void changeLegsOffset(int legsOffset) //Change legs offset (new offset)
{
  Serial.println("changeLegsOffset: start");

  LegsOffset = legsOffset; //Wright new legs offset
  
  Point A;      //Current point
  Point P0[6];  //Array of initial legs positions

  int legsLiftHeight; //Local legs lift height
  if (HexHeight <= 10) legsLiftHeight = 5;
  else legsLiftHeight = LegsLiftHeight;
  
  for(int i=0; i<=5; i++) P0[i] = Legs[i].getCurrentPos(); //Filling array P0

  for(int i=1; i<=pace; i++) //Move first 3 legs to new legs offset
  {      
    A.z = -HexHeight + legsLiftHeight * lift(i,pace);
    
    A.x = P0[1].x + (LegsOffset - P0[1].x) * i / pace;
    A.y = P0[1].y + (0 - P0[1].y) * i / pace;
    Legs[1].reach(A);

    A.x = P0[3].x + (LegsOffset - P0[3].x) * i / pace;
    A.y = P0[3].y + (0 - P0[3].y) * i / pace;
    Legs[3].reach(A);

    A.x = P0[5].x + (LegsOffset - P0[5].x) * i / pace;
    A.y = P0[5].y + (0 - P0[5].y) * i / pace;
    Legs[5].reach(A); 
    delay(del);     
  }

  for(int i=1; i<=pace; i++) //Move second 3 legs to new LegsOffset
  {      
    A.z = -HexHeight + legsLiftHeight * lift(i,pace);
    
    A.x = P0[0].x + (LegsOffset - P0[0].x) * i / pace;
    A.y = P0[0].y + (0 - P0[0].y) * i / pace;
    Legs[0].reach(A);

    A.x = P0[2].x + (LegsOffset - P0[2].x) * i / pace;
    A.y = P0[2].y + (0 - P0[2].y) * i / pace;
    Legs[2].reach(A);

    A.x = P0[4].x + (LegsOffset - P0[4].x) * i / pace;
    A.y = P0[4].y + (0 - P0[4].y) * i / pace;
    Legs[4].reach(A); 
    delay(del);     
  }

  Serial.println("changeLegsOffset: end");
}

void changeHeight(int hexheight)  //Change Hex height (new height)
{
  Serial.println("changeHeight: start");

  HexHeight = hexheight; //Wright new HexHeight

  Point A;      //Current point
  Point P0[6];  //Array of initial legs positions
  
  for(int i=0; i<=5; i++) P0[i] = Legs[i].getCurrentPos(); //Filling array P0

  for(int j=1; j<=pace; j++) //Change height
  {
    for(int i=0; i<=5; i++)
    {
      A.x = P0[i].x;
      A.y = P0[i].y;
      A.z = P0[i].z + (-HexHeight - P0[i].z) * j / pace;
      Legs[i].reach(A);

      delay(5);
    }
  }

  if (HexHeight >= 15) changeLegsOffset(Legs[0].getCalcLegOffset()); //Change legs offset depending on HexHeight
  else changeLegsOffset(55);

  Serial.println("changeHeight: end");
}

void hello()
{
  Serial.println("hello: start");

  Point A;      //Current point
  Point P0[6];  //Array of initial legs positions
  
  for(int i=0; i<=5; i++) P0[i] = Legs[i].getCurrentPos(); //Filling array P0

  for (int i=1; i<=pace; i++) // Move middle legs forward
  {     
    A.z = -HexHeight + LegsLiftHeight * lift(i,pace);
    
    A.x = P0[1].x + (50 - P0[1].x) * i / pace;
    A.y = P0[1].y + (30 - P0[1].y) * i / pace;
    Legs[1].reach(A);

    A.x = P0[4].x + (50 - P0[4].x) * i / pace;
    A.y = P0[4].y + (30 - P0[4].y) * i / pace;
    Legs[4].reach(A);

    delay(del);
  }

  for (int i=1; i<=pace; i++) //Lower rear legs
  { 
    A.x = P0[2].x;
    A.y = P0[2].y;
    A.z = P0[2].z + (-5 - P0[2].z) * i / pace;
    Legs[2].reach(A);

    A.x = P0[5].x;
    A.y = P0[5].y;
    A.z = P0[5].z + (-5 - P0[5].z) * i / pace;
    Legs[5].reach(A);

    delay(del);
  }

  //Lift, wave, lower right front leg
  for(int i=25; i<=140; i++) {Legs[3].moveTiba(i); delay(5);}
  for(int i=140; i>=110; i--) {Legs[3].moveTiba(i); delay(10);}
  for(int i=110; i<=140; i++) {Legs[3].moveTiba(i); delay(10);}
  for(int i=140; i>=25; i--) {Legs[3].moveTiba(i); delay(5);}
  Legs[3].reach(P0[3]);  

  for (int i=1; i<=pace; i++) //Return rear legs in initial position
  { 
    A.x = P0[2].x;
    A.y = P0[2].y;
    A.z = -5 + (P0[2].z + 5) * i / pace;
    Legs[2].reach(A);

    A.x = P0[5].x;
    A.y = P0[5].y;
    A.z = -5 + (P0[5].z + 5) * i / pace;
    Legs[5].reach(A);

    delay(del);
  }

  for (int i=1; i<=pace; i++) //Return middle legs in initial positon
  {     
    A.z = -HexHeight + LegsLiftHeight * lift(i,pace); 
    
    A.x = 50 + (P0[1].x - 50) * i / pace;
    A.y = 30 + (P0[1].y - 30) * i / pace;
    Legs[1].reach(A);

    A.x = 50 + (P0[4].x - 50) * i / pace;
    A.y = 30 + (P0[4].y - 30) * i / pace;
    Legs[4].reach(A);

    delay(del);
  }

  Serial.println("hello: end");
}

void swaylift (int i, int SwayLength, float R, float& h0, float& h1, float& h2) //Calculate h0,h1,h2
{
  int L = 62;                                             //Distance between front/rear legs origins and middle legs orinings by Y
  h0 = (R - sqrt(sqr(R) - sqr(SwayLength / pace * i)));   //Height of middle legs origins
  h1 = (h0 + L * SwayLength / pace * i / R);              //Height of higher front/rear legs origins 
  h2 = (h0 - L * SwayLength / pace * i / R);              //Height of lower  front/rear legs origins
}

void sway()
{
  Serial.println("sway: start");

  float h0,h1,h2;       //h0 - height of middle legs origin; h1,h2 - heights of front and rear legs origins depending on sway direction
  int SwayHeight = 15;  //Max height of middle legs origin 
  int SwayLength = 60;  //Max displacement of of middle legs origin
  float R = (sqr(SwayHeight) + sqr(SwayLength)) / (2 * SwayHeight); //Radius of curcle on which middle legs origins travel

  Point P0[6];  //Array of initial legs positions
  Point P1[6];  //Array of positions of legs turned at half StepLength
  Point P2[6];  //Array of positions of legs turned at half -StepLength
  Point A;      //Current point 
 
  for(int i=0; i<=5; i++) //Filling P0, P1, P2 arrays
  {
    Legs[i].transfer(LegsOffset,2*SwayLength,0,P1[i].x,P1[i].y,P2[i].x,P2[i].y);
    P1[i].z = -HexHeight;
    P2[i].z = -HexHeight;
    P0[i] = Legs[i].getCurrentPos();
  }

  for(int i=1; i<=pace; i++) //Sway back
  {
    swaylift(i,SwayLength,R,h0,h1,h2);

    A.x = P0[0].x + (P1[0].x - P0[0].x) * i / pace;
    A.y = P0[0].y + (P1[0].y - P0[0].y) * i / pace;
    A.z = P0[0].z - h2;
    Legs[0].reach(A);

    A.x = P0[3].x + (P1[3].x - P0[3].x) * i / pace;
    A.y = P0[3].y + (P1[3].y - P0[3].y) * i / pace;
    A.z = P0[3].z - h2;
    Legs[3].reach(A);

    A.x = P0[1].x + (P1[1].x - P0[1].x) * i / pace;
    A.y = P0[1].y + (P1[1].y - P0[1].y) * i / pace;
    A.z = P0[1].z - h0;
    Legs[1].reach(A);

    A.x = P0[4].x + (P1[4].x - P0[4].x) * i / pace;
    A.y = P0[4].y + (P1[4].y - P0[4].y) * i / pace;
    A.z = P0[4].z - h0;
    Legs[4].reach(A);

    A.x = P0[2].x + (P1[2].x - P0[2].x) * i / pace;
    A.y = P0[2].y + (P1[2].y - P0[2].y) * i / pace;
    A.z = P0[2].z - h1;
    Legs[2].reach(A);

    A.x = P0[5].x + (P1[5].x - P0[5].x) * i / pace;
    A.y = P0[5].y + (P1[5].y - P0[5].y) * i / pace;
    A.z = P0[5].z - h1;
    Legs[5].reach(A);
    
    delay(del);     
  }

  for(int i=pace-1; i>=0; i--) //Return form sway back
  {
    swaylift(i,SwayLength,R,h0,h1,h2);;

    A.x = P0[0].x + (P1[0].x - P0[0].x) * i / pace;
    A.y = P0[0].y + (P1[0].y - P0[0].y) * i / pace;
    A.z = P0[0].z - h2;
    Legs[0].reach(A);

    A.x = P0[3].x + (P1[3].x - P0[3].x) * i / pace;
    A.y = P0[3].y + (P1[3].y - P0[3].y) * i / pace;
    A.z = P0[3].z - h2;
    Legs[3].reach(A);

    A.x = P0[1].x + (P1[1].x - P0[1].x) * i / pace;
    A.y = P0[1].y + (P1[1].y - P0[1].y) * i / pace;
    A.z = P0[1].z - h0;
    Legs[1].reach(A);

    A.x = P0[4].x + (P1[4].x - P0[4].x) * i / pace;
    A.y = P0[4].y + (P1[4].y - P0[4].y) * i / pace;
    A.z = P0[4].z - h0;
    Legs[4].reach(A);

    A.x = P0[2].x + (P1[2].x - P0[2].x) * i / pace;
    A.y = P0[2].y + (P1[2].y - P0[2].y) * i / pace;
    A.z = P0[2].z - h1;
    Legs[2].reach(A);

    A.x = P0[5].x + (P1[5].x - P0[5].x) * i / pace;
    A.y = P0[5].y + (P1[5].y - P0[5].y) * i / pace;
    A.z = P0[5].z - h1;
    Legs[5].reach(A);
    
    delay(del);     
  }

  for(int i=1; i<=pace; i++) //Sway forward 
  {
    swaylift(i,SwayLength,R,h0,h1,h2);

    A.x = P0[0].x + (P2[0].x - P0[0].x) * i / pace;
    A.y = P0[0].y + (P2[0].y - P0[0].y) * i / pace;
    A.z = P0[0].z - h1;
    Legs[0].reach(A);

    A.x = P0[3].x + (P2[3].x - P0[3].x) * i / pace;
    A.y = P0[3].y + (P2[3].y - P0[3].y) * i / pace;
    A.z = P0[3].z - h1;
    Legs[3].reach(A);

    A.x = P0[1].x + (P2[1].x - P0[1].x) * i / pace;
    A.y = P0[1].y + (P2[1].y - P0[1].y) * i / pace;
    A.z = P0[1].z - h0;
    Legs[1].reach(A);

    A.x = P0[4].x + (P2[4].x - P0[4].x) * i / pace;
    A.y = P0[4].y + (P2[4].y - P0[4].y) * i / pace;
    A.z = P0[4].z - h0;;
    Legs[4].reach(A);

    A.x = P0[2].x + (P2[2].x - P0[2].x) * i / pace;
    A.y = P0[2].y + (P2[2].y - P0[2].y) * i / pace;
    A.z = P0[2].z - h2;
    Legs[2].reach(A);

    A.x = P0[5].x + (P2[5].x - P0[5].x) * i / pace;
    A.y = P0[5].y + (P2[5].y - P0[5].y) * i / pace;
    A.z = P0[5].z - h2;
    Legs[5].reach(A);
    
    delay(del);     
  }

  for(int i=pace-1; i>=0; i--)  //Return form sway forward   
  {
    swaylift(i,SwayLength,R,h0,h1,h2);

    A.x = P0[0].x + (P2[0].x - P0[0].x) * i / pace;
    A.y = P0[0].y + (P2[0].y - P0[0].y) * i / pace;
    A.z = P0[0].z - h1;
    Legs[0].reach(A);

    A.x = P0[3].x + (P2[3].x - P0[3].x) * i / pace;
    A.y = P0[3].y + (P2[3].y - P0[3].y) * i / pace;
    A.z = P0[3].z - h1;
    Legs[3].reach(A);

    A.x = P0[1].x + (P2[1].x - P0[1].x) * i / pace;
    A.y = P0[1].y + (P2[1].y - P0[1].y) * i / pace;
    A.z = P0[1].z - h0;
    Legs[1].reach(A);

    A.x = P0[4].x + (P2[4].x - P0[4].x) * i / pace;
    A.y = P0[4].y + (P2[4].y - P0[4].y) * i / pace;
    A.z = P0[4].z - h0;;
    Legs[4].reach(A);

    A.x = P0[2].x + (P2[2].x - P0[2].x) * i / pace;
    A.y = P0[2].y + (P2[2].y - P0[2].y) * i / pace;
    A.z = P0[2].z - h2;
    Legs[2].reach(A);

    A.x = P0[5].x + (P2[5].x - P0[5].x) * i / pace;
    A.y = P0[5].y + (P2[5].y - P0[5].y) * i / pace;
    A.z = P0[5].z  - h2;
    Legs[5].reach(A);
    
    delay(del);    
  }

  Serial.println("sway: end");
}

void autoMove() //Auto move avoiding obstacles
{
  int dist = 0; //Distance to obstacle
  for(int i=0; i<5; i++) dist += ultrasonic.distanceRead(CM); //Add measured distance
  dist = int(dist / 5); //Average value of the distance  
      
  Serial.println(dist);

  if (dist >= 25) move(2,0,0,1);   //Two steps forward
  if (dist < 15) move(2,180,0,1);  //Two steps backward
  if (dist < 25 && dist >= 15)
  {
    if (random(2)) turn(4,1,0,1); //Turn right
    else turn(4,-1,0,1);          //or left
  }
} 

void setup() 
{ 
  Serial.begin(115200); //Start serial port at 115200 bps:
  config();             //Legs configuration

  Point A(60,0,0);                            //Legs base position 
  for (int i=0; i<=5; i++) Legs[i].reach(A);  //Moving legs for base position

  pinMode(39, OUTPUT);     //Pin for +5V HR-SR04
  digitalWrite(39, HIGH);  //Set pin +5V
} 

void loop() 
{
  char buffer[100];   //Array for reading form serial port
  char str[10];       //Array for comand name
  String Str;         //String for comand name
  int n = 0;          //Buffer counter

  int a = 0;          //Comand parameters
  int b = 0;
  int c = 0;
  int d = 0;

  if(Serial.available())  //Reading comand from serial port
  {
    delay(50);
    while( Serial.available() && n< 99) buffer[n++] = Serial.read();
    buffer[n++]='\0';
  }

  if (n>0) 
  {    
    sscanf(buffer, "%s %d %d %d %d", str, &a, &b, &c, &d); //Analysing readed array
    Str = str;                                             //Convert comand array for string
    
    if (Str == "turn") turn (a, b, c, d);                  //turn (int Steps, int direction, float turnangle, boolean Unend)
    if (Str == "move") move(a, b, c, d);                   //move (int Steps, int DirectionAngle, int LegsOffset, boolean Unend)
    if (Str == "getup") getup();
    if (Str == "hello") hello();       
    if (Str == "sway") sway();    
    if (Str == "getdown") getdown();
    if (Str == "changeh") changeHeight(a);
    if (Str == "changelo") changeLegsOffset(a);
 
    if (Str == "reach")
    {
      Point A(a,b,c); 
      for (int i=0; i<=5; i++) Legs[i].reach(A);           //Move legs to point A
    }

    if (Str == "setdel") if (a >= 0) del = a;              //Write new del
    else Serial.println("del lower then 0");
     
    if (Str == "chanepace") if (a >= 4) pace = a;          //Write new pace
    else Serial.println("Pace lower then 4");

    if (Str == "program")                                  // Perform a predetermined program                 
    {
      getup();
      hello();
      sway();
      del = 8;
      move(8,0,0,0);
      del = 25;
      changeHeight(130);
      delay(100);
      move(4,90,40,0);
      changeHeight(60);
      del = 18;
      turn(8,1,30,0);
      changeHeight(30);
      move(10,60,0,0);
      turn(4,-1,24,0);
      del = 4;
      move(8,0,25,0);
      del = 15;
      turn(4,-1,26,0);
      del = 20;
      hello();
      getdown();
    }

    if (Str == "auto") automove = true;   //Start automatic movement
    if (Str == "stop") 
    {
      automove = false;                   //Stop automatic movement
      changeLegsOffset(LegsOffset);       //Return legs in base position
    }
  }

  if (automove) autoMove();
}


