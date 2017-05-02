#include <Servo.h>
#include <math.h>
#include "Arduino.h"

#define PI 3.141592653589793238
#define sqr(x) ((x)*(x))                  //Square of number
#define rad2deg(x) ((180.0 / PI) * (x))   //Convert radians to degrees
#define deg2rad(x) ((PI / 180.0) * (x))   //Convert degrees to radians

const int FemurLenght = 70;               //Lenght of first (from body) part of leg
const int TibaLenght = 102;               //Lenght of second (from body) part of leg
const int FemurOffsetX = 12;              //Offset from the origin by the coordinate x
const int FemurOffsetY = 10;              //Offset from the origin by the coordinate y
const int FemurOffsetZ = 23;              //Offset from the origin by the coordinate z


struct Point //Point in 3d space with x,y,z coordinates
{
  float x, y, z;

  Point()
  {}

  Point(float a, float b, float c)
  {
    x = a;
    y = b;
    z = c;
  }
 
  bool operator==(Point & that) //Operator == overloading
  {
    return ((x == that.x) && (y == that.y) && (y == that.y));
  }

  Point operator+(Point & that) //Operator + overloading
  {
    return Point(x + that.x, y + that.y, z + that.z);
  }

  Point operator-(Point & that) //Operator - overloading
  {
    return Point(x - that.x, y - that.y, z - that.z);
  }
};

class Leg
{
private: 
  Servo CoxaServo;      //Servo fastended to Hex body (leg turn)
  Servo FemurServo;     //Servo controling first(from body) part of leg
  Servo TibaServo;      //Servo controling first(from body) part of leg 

  int CoxaBaseAngleD;   //CoxaServo base angle depends from servo and its mouinting (deg)
  int FemurBaseAngleD;  //FemurServo base angle depends from servo and its mouinting (deg)
  int TibaBaseAngleD;   //TibaServo base angle depends from servo and its mouinting (deg)

  int Mirror;           //-1 == left lengs, 1 -- right legs

  Point CurrentPos;     //Current leg position

  float TurnOffsetX;    //Offset between leg origin and Hex origin by local leg coordinate x
  float TurnOffsetY;    //Offset between leg origin and Hex origin by local leg coordinate y
  int LegAngleD;        //Angle bewteen leg axis x and Hex axis Y

  int LegNumber;        //0,1,2 - left legs; 3,4,5 - right legs

  int CalcLegOffset;    //Calculated leg offset from leg origin

  void move(float CoxaAngle, float FemurAngle, float TibaAngle)       //Move leg according to input angles (rad)
  {
    int CoxaAngleD = CoxaBaseAngleD - Mirror * rad2deg(CoxaAngle);    //CoxaAngle correction for base angle and mirroring
    int FemurAngleD = FemurBaseAngleD - Mirror * rad2deg(FemurAngle); //FemurAngle correction for base angle and mirroring
    int TibaAngleD = TibaBaseAngleD + Mirror * rad2deg(TibaAngle);    //TibaAngle correction for base angle and mirroring 

    CoxaServo.write(CoxaAngleD);
    FemurServo.write(FemurAngleD);
    TibaServo.write(TibaAngleD);    
  }

  boolean checkServoAngles(float CoxaAngle, float FemurAngle, float TibaAngle) //Check if imput angles (rad) are within acceptable limits
  {
    boolean check = true; //Result flag

    if (rad2deg(CoxaAngle) < -90 || rad2deg(CoxaAngle) > 65)      //Check if CoxaAngle is within acceptable limits
    {
      Serial.print("Incorrect CoxaAngle: ");
      Serial.println(rad2deg(CoxaAngle));
      check = false;
    }

    if (rad2deg(FemurAngle) < -50 || rad2deg(FemurAngle) > 100)   //Check if FemurAngle is within acceptable limits
    {
      Serial.print("Incorrect FemurAngle: ");
      Serial.println(rad2deg(FemurAngle));
      check = false;
    }

    if (rad2deg(TibaAngle) < 20 || rad2deg(FemurAngle) > 200)     //Check if FemurAngle is within acceptable limits
    {
      Serial.print("Incorrect TibaAngle: ");
      Serial.println(rad2deg(FemurAngle));
      check = false;
    }

    if (check) return true;
    else 
    {
      Serial.print(" LegNumber: ");
      Serial.println(LegNumber);
      return false;
    }
  }

 
public:
  Point getCurrentPos() {return CurrentPos;}

  int getCalcLegOffset() {return CalcLegOffset;}

  void moveTiba(int angle) {TibaServo.write(angle);} // Forced Tiba movement

  void config(int legNumber, int coxaBaseAngleD, int femurBaseAngleD, int tibaBaseAngleD, int legAngleD) //Parameters configuration
  {
    LegNumber = legNumber;
    CoxaBaseAngleD = coxaBaseAngleD;
    FemurBaseAngleD = femurBaseAngleD;
    TibaBaseAngleD = tibaBaseAngleD;

    LegAngleD = legAngleD;
    if (LegNumber < 3) Mirror = -1;
    else Mirror = 1;
  }

  void configTurn(float turnOffsetX, float turnOffsetY) //Turn parameters configuration
  {
    TurnOffsetX = turnOffsetX;
    TurnOffsetY = turnOffsetY;
  }

  void attach(int CoxaPin, int FemurPin, int TibaPin) //Attach servos to respective pins
  {
    CoxaServo.attach(CoxaPin);
    FemurServo.attach(FemurPin);
    TibaServo.attach(TibaPin);
  }

  void reach(Point dest) //Reach imput destination point
  {
    float CoxaAngle;  //Coxa angle without correction
    float FemurAngle; //Femur angle without correction
    float TibaAngle;  //Tiba angle without correciton

    float xyDist = sqrt(sqr(dest.x) + sqr(dest.y)); //Distance between leg origin and destination point in XY plane

    if (xyDist < 10)  //Protection from incorrect reach point
    {
      Serial.print("Incorrect reach point (xyDist) ");
      Serial.print("LegNumber: ");
      Serial.println(LegNumber);
      return; 
    } 
    
    float addCoxaAngle = asin(FemurOffsetY / xyDist); //Addition coxa angle is required because leg plane isn't pass through leg origin
    float rawCoxaAngle = asin(dest.y / xyDist);       //Coxa angle without correction
    
    CoxaAngle = rawCoxaAngle - addCoxaAngle; 
  
    float x = xyDist*cos(addCoxaAngle); //xyDist projection on leg plane
    
    //algolist.manual.ru/maths/geom/intersect/circlecircle2d.php aproach 2

    float d = sqrt(sqr(x - FemurOffsetX) + sqr(dest.z - FemurOffsetZ));    //Distance between two circles centers (1- femur start; 2 - tiba end) 

    if ((d > FemurLenght + TibaLenght) || (d < TibaLenght - FemurLenght))  //Protection from incorrect reach point
    {
      Serial.print("Incorrect reach point d: ");
      Serial.print(d);
      Serial.print(" LegNumber: ");
      Serial.print(LegNumber);
      Serial.print(" x: ");
      Serial.print(x);
      Serial.print(" z: ");
      Serial.println(dest.z);
      return; 
    } 

    float a = (sqr(FemurLenght) - sqr(TibaLenght) + sqr(d)) / (2*d);
    float h = sqrt(sqr(FemurLenght)-sqr(a)); 
     
    float x2 = FemurOffsetX + (a/d) * (x - FemurOffsetX); 
    float z2 = FemurOffsetZ + (a/d) * (dest.z - FemurOffsetZ); 
   
    float KneeX = x2 - (h/d) * (dest.z - FemurOffsetZ); //x coordinate of femur end/tiba start (knee)
    float KneeZ = z2 + (h/d) * (x - FemurOffsetX);      //z coordinate of femur end/tiba start (knee)

    CalcLegOffset = int(KneeX); //Writing knee x coordinate

    if ((KneeX - FemurOffsetX) >= 0) FemurAngle = asin((KneeZ-FemurOffsetZ)/FemurLenght);     //Calculation FemurAngle depending on knee location
                                else FemurAngle = PI - asin((KneeZ-FemurOffsetZ)/FemurLenght); 

    if (a >= 0) TibaAngle = acos(h/FemurLenght) + acos(h/TibaLenght);   //Calculation TibaAngle depending on a location
           else TibaAngle = - acos(h/FemurLenght) + acos(h/TibaLenght); 

    if (checkServoAngles(CoxaAngle, FemurAngle, TibaAngle)) //Checking angle correctness
    {  
      move(CoxaAngle, FemurAngle, TibaAngle);
      CurrentPos = dest;
    }
  }

  //Transfer from XY to leg local cooridnates xy for move (Leg offset, step length, step direction angle form Y axis, 
  //x1,y1, x2,x2 - points coordinates of start and end point of step)
  void transfer(int LegOffset, int StepLength, int DirectionAngle, float& x1, float& y1, float& x2, float& y2)
  {
    float x =  0.5 * StepLength * cos(deg2rad(LegAngleD - Mirror * DirectionAngle));  //Distance between leg base posinion and point 1/2 by x coordinate
    x1 = LegOffset + x;
    x2 = LegOffset - x;

    y1 = 0.5 * StepLength * sin(deg2rad(LegAngleD - Mirror * DirectionAngle));        //Distance between leg base posinion and point 1 by y coordinate
    y2 = -y1;
  }

  void turnPoint(int LegOffset, float TurnAngle, float& x, float& y) //Calculate turn point (Leg offset, angle from leg base position, x,y turn point coordinates)
  {
    int R; //Radius of circle on which turn point is lay
    if (LegNumber == 1 || LegNumber == 4) R = 62 + LegOffset; // For middle legs; 62 - distance between Hex body origin and middle legs origins by X coordinate
    else R = sqrt(sqr(40 + LegOffset * cos(deg2rad(45))) + sqr(62 + LegOffset * sin(deg2rad(45)))); // For front/rear legs;
    //40 - distance between Hex body origin and front/rear legs origins by X coordinate; 62 - by coordinate; 45 - base angle on which front/rear legs are directed; 

    float BaseAngle = asin((-1) * TurnOffsetY/R);       //Angle wich top is Hex body origin and it is between radius in leg  base position and leg local x axis 
    x = TurnOffsetX + R * cos(TurnAngle + BaseAngle); //Calculation x turn point coordinate
    y = TurnOffsetY + R * sin(TurnAngle + BaseAngle); //Calculation y turn point coordinate
  } 
};