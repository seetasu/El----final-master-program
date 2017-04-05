#include <Adafruit_NeoPixel.h>
#include <Stepper.h>
#include <QueueArray.h>
#include <unistep.h>
/*********define the appliances btns and sensors***********/
int hall_sensor = 0;
#define NOFIELD 505L
#define TOMILLIGAUSS 1953L 

#define fridge_light_pin            30
#define fridge_NUMPIXELS      1
Adafruit_NeoPixel fridge_light = Adafruit_NeoPixel(fridge_NUMPIXELS, fridge_light_pin, NEO_GRB + NEO_KHZ800);
int fridge_LDR = 1;
int fridge_counter = 0;

#define tv_light_pin    32
Adafruit_NeoPixel tv_light = Adafruit_NeoPixel(1, tv_light_pin, NEO_GRB + NEO_KHZ800);

#define tv_btn 33
int tv_state = 0;
int tv_state_new = 0; 
int tv_counter = 0;
int tv_standby_counter = 0;

#define lamp_light 34
#define lamp_btn 35
int lamp_state = 0;
int lamp_state_new = 0; 
int lamp_counter = 0;

int range_counter = 0;

int noonXmin = 1;
int noonXmax = 9;
int noonYmin = 4;
int noonYmax = 8;

/*********define timeslot btn***********/
#define NOFIELD 505L;
#define TOMILLIGAUSS 1953L  // For A1301: 2.5mV = 1Gauss, and 1024 analog steps = 5V, so 1 step = 1953mG

int morningBtn = 36;
int noonBtn = 37;
int nightBtn = 38;
int resetBtn = 50;

unsigned long timeStamp;
int morningBtnState = 0;
int noonBtnState = 0;
int nightBtnState = 0;
int resetBtnState = 0;

int timeSlot = 0;

/*********define left and right steppers***********/
unistep left(8,9,10,11,4096,900);
unistep right(4,5,6,7,4096,900);

#define ONE_REVOLUTION 4096;

#define DISPLAY_WIDTH 9;
#define DISPLAY_HEIGHT 14;

float WHEEL_DIAMETER = 3.5;
float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER*PI;

int currentPoint[]={1,3};
int stepperPos_L[]={-1,-1};
int stepperPos_R[]={10,-1};

double currentD_L = sqrt(20);
double currentD_R = sqrt(97);
int stepToTake_L = 0;
int stepToTake_R = 0;
int stepsRemaining_L = 0;
int stepsRemaining_R = 0;
int stepsTaken_L = 0;
int stepsTaken_R = 0;

boolean stepperDirection_L;
boolean stepperDirection_R;

boolean isReadyForNewPoint = true;


/*********define xy coorninates array***********/
QueueArray <int> xCoordinates;
QueueArray <int> yCoordinates;  

/*********abou random***********/
long randomNumber;


void setup()
{
  Serial.begin(9600);

  xCoordinates.setPrinter(Serial);
  yCoordinates.setPrinter(Serial);
  randomSeed(analogRead(A0));

  pinMode(morningBtn, INPUT);
  pinMode(noonBtn, INPUT);
  pinMode(nightBtn, INPUT);
  fridge_light.begin();
  fridge_light.setPixelColor(0,0,0,0);
  fridge_light.show();

  tv_light.begin(); 
  tv_light.setPixelColor(0,0,0,0);
  tv_light.show();

  digitalWrite(lamp_light,LOW);

  pinMode(hall_sensor, INPUT);
  
  pinMode(fridge_LDR, INPUT);
  pinMode(fridge_light_pin, OUTPUT);

  pinMode(tv_btn, INPUT);
  pinMode(tv_light_pin, OUTPUT);

  pinMode(lamp_btn, INPUT);
  pinMode(lamp_light, OUTPUT);
}

unsigned long currentMillis;

int incomingByte;

int randomSpeed;

void loop()
{
  int morning = 0;
  int noon = 0;
  int night = 0;
  int beginAgain = 1;
  catchFF();

  while(1)
  {
    currentMillis = millis();
    whatTimeSlot();
    switch(timeSlot)
    {
      case 1:
      {
        beginAgain = 0;
        morning = 1;
        noon = 0;
        night = 0;
        break;
      }
      case 2:
      {
        beginAgain = 0;
        morning = 0;
        noon = 1;
        night = 0;
        break;
      }
      case 3:
      {
        beginAgain = 0;
        morning = 0;
        noon = 0;
        night = 1;
        break;
      }
      case 0:
      {
        beginAgain = 1;
        morning = 0;
        noon = 0;
        night = 0;
        break;
      }
    }

    if(morning)
    {
      whatTimeSlot();
      int randomX = random(1,9);
      int randomY = random(7,12);
      addPoint(randomX,randomY);
      if (isReadyForNewPoint && !xCoordinates.isEmpty() && !yCoordinates.isEmpty())
      {
        int x = xCoordinates.dequeue(); 
        int y = yCoordinates.dequeue();
        nextPoint(x,y);
        whatTimeSlot();
        delay(100);
      }
    }
    if(noon)
    {
      whatRange();
      whatTimeSlot();
      tv_standby_killer_detection();
      switch(range_counter)
      {
        case 0:
        {
          noonXmin = 1;
          noonXmax = 9;
          noonYmin = 4;
          noonYmax = 8;
          randomSpeed = 0;
          giveRandomCoordinates();
          break;          
        }
        case 1:
        {
          noonXmin = 2;
          noonXmax = 8;
          noonYmin = 4;
          noonYmax = 8;
          randomSpeed = 300;
          giveRandomCoordinates();
          break;                    
        }
        case 2:
        {
          noonXmin = 3;
          noonXmax = 7;
          noonYmin = 5;
          noonYmax = 7;
          randomSpeed = 700;
          giveRandomCoordinates(); 
          break;         
        }
        case 3:
        {
          noonXmin = 4;
          noonXmax = 6;
          noonYmin = 5;
          noonYmax = 7;
          randomSpeed = 1000;
          giveRandomCoordinates();
          break;          
        }
        case 4:
        {
          addPoint(1, 11);
          if (isReadyForNewPoint && !xCoordinates.isEmpty() && !yCoordinates.isEmpty())
          
          {
            int x = xCoordinates.dequeue(); 
            int y = yCoordinates.dequeue(); 
            nextPoint(x,y);
            whatTimeSlot();
            delay(50);
          }
          addPoint(2,11);
          if (isReadyForNewPoint && !xCoordinates.isEmpty() && !yCoordinates.isEmpty())
          {
            int x = xCoordinates.dequeue(); 
            int y = yCoordinates.dequeue();
            nextPoint(x,y);
            whatTimeSlot();
            delay(50);
          }
          addPoint(1,10);
          if (isReadyForNewPoint && !xCoordinates.isEmpty() && !yCoordinates.isEmpty())
          {
            int x = xCoordinates.dequeue(); 
            int y = yCoordinates.dequeue(); 
            nextPoint(x,y);
            whatTimeSlot();
            delay(50);
          }
          
          tv_standby_killer_detection();
          if(tv_standby_counter == 1)
          {
            tv_light.setPixelColor(0,0,0,0);
            tv_light.show();
          }
          break;
        }
      }
      
    }
    if(night)
    {
      whatTimeSlot();
      int randomX = random(1,9);
      int randomY = random(0,4);
      if((randomX == 7 || randomX == 8)&&(randomY == 0 || randomY == 1))
      {
        
      }else
      {
        addPoint(randomX,randomY);                
      }
      if (isReadyForNewPoint && !xCoordinates.isEmpty() && !yCoordinates.isEmpty())
      {
        int x = xCoordinates.dequeue(); 
        int y = yCoordinates.dequeue(); 
        nextPoint(x,y);
        whatTimeSlot();
        delay(100);
      }
    }
    if(beginAgain)
    {
      whatTimeSlot();
      reset();
    }
  }
}



void giveRandomCoordinates()
{
  int randomX = random(noonXmin,noonXmax);
      int randomY = random(noonYmin,noonYmax);
      Serial.println("noonXmin" + String(noonXmin) + "noonXmax" + String(noonXmax));
      Serial.println("noonYmin" + String(noonYmin) + "noonYmax" + String(noonYmax));            
      
      addPoint(randomX,randomY);
      if (isReadyForNewPoint && !xCoordinates.isEmpty() && !yCoordinates.isEmpty())
      {
        int x = xCoordinates.dequeue();
        int y = yCoordinates.dequeue();
        nextPoint(x,y);
        whatTimeSlot();
//        whatRange();
        delay(randomSpeed);
      }
}

/*********detects if the appliance is on***********/
boolean doorclosed = true;
unsigned long previousMillis = 0;

const long interval = 1000;

void fridge_door_detection()
{
  delay(500);
  Serial.println(analogRead(fridge_LDR));
  if(analogRead(fridge_LDR)>400 && doorclosed == true)
  { 
    previousMillis = currentMillis;
    fridge_light.setPixelColor(0,255,255,255);
    fridge_light.show();
    Serial.println("turned on");
    doorclosed = false;
    fridge_counter = 1;
  }

  if(currentMillis - previousMillis >= interval && doorclosed == false)
  {
    previousMillis = currentMillis;
    fridge_light.setPixelColor(0,0,0,0);
    fridge_light.show();
    Serial.println("turned off");
  }

  if(analogRead(fridge_LDR)<400){
    doorclosed = true;
    fridge_counter = 0;
  }
}

void lamp_edge_detection()
{
  lamp_state_new = digitalRead(lamp_btn);
  if(lamp_state_new != lamp_state)
  {
    if(lamp_state_new == HIGH)
    {
      lamp_counter++;
      if(lamp_counter == 2)
      {
        lamp_counter = 0;
      }else
      {
      };
//      Serial.println("lamp counter"+String(lamp_counter));
    }
  }
  lamp_state = lamp_state_new;
  
 if(lamp_counter % 2 == 0)
  {
    digitalWrite(lamp_light, LOW);
  }else
  {
    digitalWrite(lamp_light, HIGH);
  }
}

void tv_edge_detection()
{
  tv_state_new = digitalRead(tv_btn);  

  if(tv_state_new != tv_state)
  {
    if(tv_state_new == HIGH)
    {
      tv_counter++;
      if(tv_counter == 2)
      {
        tv_counter = 0;
      }else
      {
      };
//      Serial.println("tv counter"+String(tv_counter));
    }
  }
  tv_state = tv_state_new;

  if(tv_counter % 2 == 0 && tv_standby_counter == 0)
  {
    tv_light.setPixelColor(0,10,10,10);
    tv_light.show();  
  }
  if(tv_counter%2 != 0)
  {
    tv_light.setPixelColor(0,255,255,255);
    tv_light.show();
//    Serial.println("TV on standby!!");
  }

  if(tv_counter == 1)
  {
    tv_standby_counter = 0;
  }
}

int gaussClass = 0;

void DoMeasurement()
{
// measure magnetic field
  int raw = analogRead(0);   // Range : 0..1024

  long compensated = raw - NOFIELD;                  
  long gauss = compensated * TOMILLIGAUSS / 1000;   

  if(gauss>0)
  {
    gaussClass = 1;
  }else
  {
    gaussClass = 0;
  }
  Serial.println("gauss "+ String(gauss));
}

void tv_standby_killer_detection()
{
  DoMeasurement();
  if(gaussClass == 1 && tv_counter == 0 && millis()-timeStamp > 2000)
  {
    timeStamp = millis();
    tv_standby_counter = 1; 
  } 
}

void whatTimeSlot()
{
   morningBtnState = digitalRead(morningBtn);
   noonBtnState = digitalRead(noonBtn);
   nightBtnState = digitalRead(nightBtn);
   resetBtnState = digitalRead(resetBtn);
//   Serial.println("reset btn " + String(resetBtnState));
    if(morningBtnState == HIGH && millis()-timeStamp > 2000)
    {
      timeStamp = millis();
      timeSlot = 1;
      Serial.println("timeslot" + String(timeSlot));
      catchFF();
    }
    if(noonBtnState == HIGH && millis()-timeStamp > 2000)
    {
      timeStamp = millis();
      timeSlot = 2;
      catchFF();
      Serial.println("timeslot" + String(timeSlot));
    }
    if(nightBtnState == HIGH && millis()-timeStamp > 2000)
    {
      timeStamp = millis();
      timeSlot = 3;
      catchFF();
      Serial.println("timeslot" + String(timeSlot));
    }
     if(resetBtnState == HIGH && millis()-timeStamp > 2000)
    {
      timeStamp = millis();
      timeSlot = 0;
      catchFF();
      Serial.println("timeslot" + String(timeSlot));
    }
}

void whatRange()
{
  lamp_edge_detection();
  tv_edge_detection();
  fridge_door_detection();
  if(tv_counter == 0 && tv_standby_counter == 0)
  {
    range_counter = 4;
  }else
  {
    range_counter = tv_counter + lamp_counter + fridge_counter;    
  }
  Serial.println("range_counter " + String(range_counter));
}

void addPoint(int x, int y)
{
  xCoordinates.push(x);
  yCoordinates.push(y);
//  setCurrentPosition(x,y);
//  Serial.println("Added point (" + String(x) + ", " + String(y) + ") to the list");
}

void nextPoint(int x, int y)
{
  int nextPoint[] = {x, y};
  
  double newD_L = calculateDistance(x, y, stepperPos_L[0], stepperPos_L[1]);
  double newD_R = calculateDistance(x, y, stepperPos_R[0], stepperPos_R[1]);
  
  double differenceD_L = newD_L - currentD_L;
  double differenceD_R = newD_R - currentD_R;
  
  if(differenceD_L>=0)
  {
    stepperDirection_L = 0; 
    //left go
  }else
  { stepperDirection_L = 1;
  } //left back

  if(differenceD_R>=0)     
  {
    stepperDirection_R = 1;
    //right go
    }else
  { 
    stepperDirection_R = 0;
  }//right back

  double leftTurns;
  double rightTurns;

  leftTurns = differenceD_L/WHEEL_CIRCUMFERENCE;
  rightTurns = differenceD_R/WHEEL_CIRCUMFERENCE;  

  stepToTake_L = (int)(leftTurns * 4096);
  stepToTake_R = (int)(rightTurns * 4096);  

  stepsRemaining_L = stepToTake_L;
  stepsRemaining_R = stepToTake_R;

  takeSteps((float)stepToTake_L, (float)stepToTake_R);


  currentD_L = newD_L;
  currentD_R = newD_R;  
  currentPoint[0] = nextPoint[0];
  currentPoint[1] = nextPoint[1];
  Serial.println("position update: "+ String(currentPoint[0]) + ", " + String(currentPoint[1]));
  Serial.println();
}

void takeSteps(float l_step, float r_step)
{
  float leftInc = abs(l_step)/4096.0;//0.5
  float rightInc = abs(r_step)/4096.0;//0.25

  float leftPos = 0;
  float rightPos = 0;


  for(int s = 0; s < 4096; s++)
  {
    
    leftPos = leftPos + leftInc;//1.2  0.2+1.2=1.4  0.4+1.2=1.6  0.6+1.2=1.8  0.8+1.2=2.0 1.2
    rightPos = rightPos + rightInc;//2.6  0.6+2.6=3.2  0.2+2.6=2.8  0.8+2.6=3.4  0.4+2.6=3.0 2.6
    int leftTempPos = (int)leftPos;//1 1 1 1 2 1
    int rightTempPos = (int)rightPos;//2 3 2 3 3
    
    stepsTaken_L = stepsTaken_L + leftTempPos;
    // 0+1=1, 1+1=2, 2+1=3, 3+1=4, 4+2=6, 6+1=7, 8 9 10 12 
    stepsTaken_R = stepsTaken_R + rightTempPos;
    //0+2=2, 2+3=5, 5+2=7, 7+3=10, 10+3=13, 15 18 20 23 26 
    stepsRemaining_L = stepsRemaining_L - stepsTaken_L;
    //4914 4913 4912 4911 4910...
    stepsRemaining_R = stepsRemaining_R - stepsTaken_R;
    //(10650)10648 10645 10643 10640 10637... 

    left.moves(leftTempPos,stepperDirection_L);//1
    right.moves(rightTempPos,stepperDirection_R);//3
    leftPos = leftPos - leftTempPos;//1.2-1=0.2  1.4-1=0.4
    rightPos = rightPos - rightTempPos;//2.6-2=0.6  2.6-2=0.6
  }
}

double calculateDistance(int x1, int y1, int x2, int y2)
{
  double distance = sqrt(sq(x2-x1)+sq(y2-y1));
  return distance;
}

void reset()
{
  isReadyForNewPoint = true;
  addPoint(1,3);
  if (isReadyForNewPoint && !xCoordinates.isEmpty() && !yCoordinates.isEmpty())
  {
    int x = xCoordinates.dequeue(); 
    int y = yCoordinates.dequeue(); //
    nextPoint(x,y);
    delay(0);
  }
  tv_light.setPixelColor(0,0,0,0);
  tv_light.show(); 
  fridge_light.setPixelColor(0,0,0,0);
  fridge_light.show();
  digitalWrite(lamp_light, LOW);
  Serial.println("the system is reset."); 
}

 

void catchFF()
{
  
  isReadyForNewPoint = true;
  addPoint(4,12);
  if (isReadyForNewPoint && !xCoordinates.isEmpty() && !yCoordinates.isEmpty())
  {
    int x = xCoordinates.dequeue(); 
    int y = yCoordinates.dequeue(); //
    nextPoint(x,y);
  }
}


