/*MR Dir- 7
   MR PWM- 9
   ML Dir- 8
   ML PWM- 10
   RGB- 6,5,3

   Line sensor on A0,A1,A2,A3,A4
   A0-left & A4 - right

 600;
 */
//int threshold = (blackValue + whiteValue) * 0.5;

int P, D, I, previousError, PIDvalue, error;


int lsp = 100;
int rsp = 100;

int turnspeed;
// int lfspeed = 50;
// float Kp = 0.02;
// float Kd = 1.2;
// int Ki;


int FT = 250;
int const lfspeed = 50;//----------ok
float Kp = 0.54;
float Kd = 7.4;
float Ki = 0.00018;

// int FT = 100;
// int lfspeed = 120;
// float Kp = 1.95;
// float Kd = 37.0;
// float Ki = 0.00028;


// int FT = 177;
// int lfspeed = 160;//-------ok
// float Kp = 1.96;
// float Kd = 60.0;
// float Ki = 0.00028;

// int FT = 200;
// int lfspeed = 190; // ----------ok but late decision making //16s
// float Kp = 2.24;
// float Kd = 54.0;
// float Ki = 0.00042;

// int FT = 200;
// int lfspeed = 180; // ----------ok but late decision making
// float Kp = 2.24;
// float Kd = 57.6;
// float Ki = 0.0004;

// int FT = 169;
// int lfspeed = 200; // ----------ok but late decision making 
// float Kp = 2.25;
// float Kd = 59.5;
// float Ki = 0.00042;

const int startButton = 11;

bool l = 0;
bool r = 0;
bool s = 0;
bool u = 0;
bool x = 0;
bool y = 0;
int e = 0;
int paths = 0;
bool endFound = 0;

int blackValue = 900;
int whiteValue = 100;
//int threshold =
// float Ki = 0.00002;
// float Kp = 0.08;
// float Kd = 0.05;
// float Ki = 0 ;

String str;

unsigned long elapsedtime, startime;
#include <SparkFun_TB6612.h>

#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
#define STBY 5

          // these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

int minValues[8], maxValues[8], threshold[8];

void setup() {
  Serial.begin(9600);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(2, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(13, OUTPUT);

  // lfspeed = 55000 / analogRead(7); //arbitrary conversion to convert analogRead to speed. Need to check if this works for all voltage levels
  turnspeed = 48.5;
  // turnspeed=45;

}

void loop() {

  while (digitalRead(11)) {}
  delay(1000);
  calib();

  while (digitalRead(12)) {}
  delay(1000);

  while (endFound == 0)
  {
    linefollow();
    checknode();
    reposition ();
  }
  digitalWrite(13,HIGH);
  delay(2000);
  for (int m = 0; m < 4; m++)
  {
    str.replace("RUR", "S");
    str.replace("SUR", "L");
    str.replace("RUS", "L");
    str.replace("LUR", "U");
  }
  int endpos = str.indexOf('E');

  while (digitalRead(11)) {}
  delay(1000);

  for (int i = 0; i <= endpos; i++)
  {
    char node = str.charAt(i);
    paths = 0;
    while (paths < 2)
    {
      linefollow();
      checknode();
      if (paths == 1)
      {
        reposition();
      }
    }
    switch (node)
    {
      blue();
      case 'L':
        botstop();
        delay(50);
        botleft();
        break;

      case 'S':
        break;

      case 'R':
        botstop();
        delay(50);
        botright();
        break;

      case 'E':
        digitalWrite(13,HIGH);
        red();
        botstop();
        delay(5000);
        break;
    }//_end of switch
  }//_end of for loop

}

void showSensorData()
{
  Serial.print("Sensor 0-  ");
  Serial.print(analogRead(0));
  Serial.print("  Sensor 1-  ");
  Serial.print(analogRead(1));
  Serial.print("  Sensor 2-  ");
  Serial.print(analogRead(2));
  Serial.print("  Sensor 3-  ");
  Serial.print(analogRead(3));
  Serial.print("  Sensor 4-  ");
  Serial.println(analogRead(4));
}

void calib()
{
  for ( int i = 0; i < 5; i++)
  {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }
  
  for (int i = 0; i < 2500; i++)
  {
    motor1.drive(50);
    motor2.drive(-50);

    for ( int i = 0; i < 5; i++)
    {
      if (analogRead(i) < minValues[i])
      {
        minValues[i] = analogRead(i);
      }
      if (analogRead(i) > maxValues[i])
      {
        maxValues[i] = analogRead(i);
      }
    }
  }

  for ( int i = 0; i < 5; i++)
  {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("   ");
  }
  Serial.println();
  
  motor1.drive(0);
  motor2.drive(0);
}

void checknode ()
{
  lightsoff();
  l = 0;
  r = 0;
  x = 0;
  y = 0;
  s = 0;
  u = 0;
  e = 0;
  paths = 0;

  // checks whethere bot is on node and the number of exits possible

  if ((analogRead(0) > threshold[0] && (analogRead(4) > threshold[4]) && (analogRead(2) > threshold[2]))) {
    u = 1;
    botstraight();
    delay(100);
  }
//(analogRead(0) > threshold[0] // means black surface is detected

  if (u == 0)
  {
    for (int i = 0; i < FT; i++)
    {
      PID();
      if (analogRead(0) < threshold[0]) l = 1;
      if (analogRead(4) < threshold[4]) r = 1;
    }

    if (analogRead (2) < threshold[2]) s = 1;
    if ((analogRead(0) < threshold[0]) && (analogRead(4) < threshold[4]) && (analogRead(2) < threshold[2])) e = 2;
//is it incorrect??

  }

  paths = l + s + r;

}

void magenta ()

{ /*
    analogWrite (3, 200); //BLUE
    analogWrite(5, 0); //Green      // MAGENTA
    analogWrite(6, 150); //red
  */
}

void yellow ()
{
  /*
    analogWrite (3, 0); //BLUE
    analogWrite(5, 200); //Green      // Yellow
    analogWrite(6, 150); //red
  */
}
void cyan()
{
  /*
    analogWrite (3, 200); //BLUE
    analogWrite(5, 200); //Green      // Cyan
    analogWrite(6, 0); //red
  */
}
void green ()

{
  /*
    digitalWrite (3, LOW);
    digitalWrite(5, HIGH);          // GREEN
    digitalWrite(6, LOW);
  */
}

void red ()
{
    digitalWrite(8,HIGH);
  /*
    digitalWrite (3, LOW);
    digitalWrite(5, LOW);
    digitalWrite(6, HIGH);           //RED
  */
}
void blue ()
{
  digitalWrite(2,HIGH);

  /*
    digitalWrite (3, HIGH);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);           //BLUE
  */
}
void lightsoff()
{
  digitalWrite(2,LOW);
  digitalWrite(8,LOW);
  digitalWrite(13,LOW);
  /*
    digitalWrite (3, LOW);
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
  */
}


void linefollow()
{ //green () ;
  paths = 0;
  while ((analogRead(0) > threshold[0] ) && (analogRead(4) > threshold[4] )&& (analogRead(2) < threshold[2]))
  {
    PID();
  }
  lightsoff();
}
void PID()
{
  int error = analogRead(1) - analogRead(3);

  P = error; // present error 
  I = I + error; // Future 
  D = error - previousError; 

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  if (lsp > 200) {
    lsp = 200;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 200) {
    rsp = 200;
  }
  if (rsp < 0) {
    rsp = 0;
  }

  motor1.drive(rsp);
  motor2.drive(lsp);
}


void reposition()
{
  lightsoff();
  if (e == 2)
  {
    str += 'E';
    endFound = 1;
    botstop();
    red();
    delay(200);
    lightsoff();
  }

  else if (r == 1)
  {
    if (paths > 1) str += 'R';
    botright(); //take right
  }
  else if (s == 1)
  {
    if (paths > 1) str += 'S';
  }

  else if (l == 1)
  {
    if (paths > 1) str += 'L';
    botleft(); //take left
  }

  else if (u == 1)
  {
    str += 'U';
    botuturn(); //take left
  }
  lightsoff();

}

void botX ()
{
  motor1.drive(-1 * turnspeed);
  motor2.drive(turnspeed);
  delay(500);
  while (analogRead(2) > threshold[2])
  {
    motor1.drive(-1 * turnspeed);
    motor2.drive(turnspeed);
  }
  motor1.drive(0);
  motor2.drive(0);
  delay(50);
}
void botleft ()
{
  motor1.drive(-1 * turnspeed);
  motor2.drive(turnspeed);
  delay(150);
  while (analogRead(2) > threshold[2])
  {
    motor1.drive(-1 * turnspeed);
    motor2.drive(turnspeed);
  }
  motor1.drive(0);
  motor2.drive(0);
  delay(50);
}

void botright ()
{
  motor1.drive(turnspeed);
  motor2.drive(-1 * turnspeed);
  delay(150);
  while (analogRead(2) > threshold[2])
  {
    motor1.drive(turnspeed);
    motor2.drive(-1 * turnspeed);
  }
  motor1.drive(0);
  motor2.drive(0);
  delay(50);
}

void botstraight ()
{
  motor1.drive(lfspeed);
  motor2.drive(lfspeed);
}

void botinchforward ()
{
  motor1.drive(turnspeed);
  motor2.drive(turnspeed);
  delay(10);
}
void botstop ()
{
  motor1.drive(0);
  motor2.drive(0);
}
void botuturn ()
{
  motor1.drive(-1 * turnspeed);
  motor2.drive(turnspeed);
  delay(150);
  while (analogRead(2) > threshold[2])
  {
    motor1.drive(-1 * turnspeed);
    motor2.drive(turnspeed);
  }
  motor1.drive(0);
  motor2.drive(0);
  delay(50);
}
void forwardstep()
{
  int battV = analogRead(7)- 770;
}
// Serial.println(str.length());