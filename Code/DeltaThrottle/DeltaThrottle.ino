const bool DEBUG = false;  // set to true to debug the raw values

//Geometry
const float handle_rad = 1.75;     // end effector
const float base_rad = 1.75;     // base
const float pushrod_lng = 3.5;
const float pivot_lng = 2.25;

//Configuration
const float deadzone = 0.1;  // smaller values will be set to 0
const int gain = 150;

 // trigonometric constants
const float sqrt3 = sqrt(3.0);
const float pi = 3.141592653;    // PI
const float sin120 = sqrt3/2.0;   
const float cos120 = -0.5;        
const float tan60 = sqrt3;
const float sin30 = 0.5;
const float tan30 = 1/sqrt3;

//Pins Used
const int btn1Pin = 1;
const int btn2Pin = 2;
const int btn3Pin = 3;
const int ADC1Pin = A0; //Bottom Pot
const int ADC2Pin = A1; //Top/Right Pot
const int ADC3Pin = A2; //Top/Left
const int enabledLED = 6;
const int enabledInterrupt = 7;

//Global Values
volatile bool enabled = false;
volatile bool needInit = false;
float xZero, yZero, zZero;
float xValue, yValue, zValue;

void setup() {
  pinMode(ADC1Pin, INPUT);
  pinMode(ADC2Pin, INPUT);
  pinMode(ADC3Pin, INPUT);
  pinMode(btn1Pin, INPUT);
  pinMode(btn2Pin, INPUT);
  pinMode(btn3Pin, INPUT);
  digitalWrite(btn1Pin, HIGH);
  digitalWrite(btn2Pin, HIGH);
  digitalWrite(btn3Pin, HIGH);
  attachInterrupt(enabledInterrupt, interrupt, FALLING);

  if (DEBUG) {
    Serial.begin(9600);
  }
}

void initialize() 
{
    xZero = 0;
    yZero = 0;
    zZero = zValue;
    needInit = false;
}

void loop() {
  int xVal = 0;
  int yVal = 0;
  int zVal = 0;
  int btn1 = LOW;
  int btn2 = LOW;
  int btn3 = LOW;
  
  if(enabled) 
  {
    getForwardKinematic();

    if(needInit) initialize();
  
    btn1 = !digitalRead(btn1Pin);
    btn2 = !digitalRead(btn2Pin);
    btn3 = !digitalRead(btn3Pin);
  
    xVal = GetAxisValue(xValue, xZero, gain, deadzone, -100, 100);
    yVal = GetAxisValue(yValue, yZero, gain, deadzone, -100, 100);
    zVal = GetAxisValue(yValue, zZero, gain, deadzone, -100, 100);    
     
    if (DEBUG) {
      Serial.print("X: ");
      Serial.println(xVal);
      Serial.print("Y: ");
      Serial.println(yVal);
      Serial.print("Z: ");
      Serial.println(zVal);
      Serial.print("B1: ");
      Serial.println(btn1);
      Serial.print("B2: ");
      Serial.println(btn2);
      Serial.print("B3: ");
      Serial.println(btn3);
    }
  }
  
  if(enabled) 
  {
    Joystick.SetAxis(Joystick_::XAXIS, map(xVal, -100, 100, 0, 255));
    Joystick.SetAxis(Joystick_::YAXIS, map(yVal, -100, 100, 0, 255));
    Joystick.SetAxis(Joystick_::ZAXIS, map(zVal, -100, 100, 0, 255));
    Joystick.SetButton(1, btn1 == HIGH);
    Joystick.SetButton(2, btn2 == HIGH);
    Joystick.SetButton(3, btn3 == HIGH);
    digitalWrite(enabledLED, HIGH);
  }
  else 
  {
    Joystick.SetAxis(Joystick_::XAXIS, 0);
    Joystick.SetAxis(Joystick_::YAXIS, 0);
    Joystick.SetAxis(Joystick_::ZAXIS, 0);
    Joystick.SetButton(1, false);
    Joystick.SetButton(2, false);
    Joystick.SetButton(3, false);
    digitalWrite(enabledLED, LOW);
  }

  // Send to USB
  Joystick.ReportState();

  if (DEBUG)
    delay(1000);
}

void interrupt() 
{
  if(enabled) 
  {
    enabled = false;
  }
  else
  {
    enabled = true;
    needInit = true;
  }  
}
//The delta kinematic math
//You shouldn't need to change anything here.
void getForwardKinematic()
{
  //p1 is the bottom joint, p2 is the top right, p3 is the top left
  int p1ADC = analogRead(ADC1Pin);
  int p2ADC = analogRead(ADC2Pin);
  int p3ADC = analogRead(ADC3Pin);


  //get the angle of each joint in radians
  float theta1 = (p1ADC - 60) * 0.003475;
  float theta2 = (p2ADC - 60) * 0.003475;
  float theta3 = (p3ADC - 60) * 0.003475;

  float t = base_rad - handle_rad;
  float y1 = -(t + pivot_lng * cos(theta1));
  float z1 = pivot_lng * sin(theta1);

  float y2 = (t + pivot_lng * cos(theta2)) * sin30;
  float x2 = y2 * tan60;
  float z2 = pivot_lng * sin(theta2);

  float y3 = (t + pivot_lng * cos(theta3)) * sin30;
  float x3 = -y3 * tan60;
  float z3 = pivot_lng * sin(theta3);

  float dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

  float w1 = y1 * y1 + z1 * z1;
  float w2 = x2 * x2 + y2 * y2 + z2 * z2;
  float w3 = x3 * x3 + y3 * y3 + z3 * z3;

  // x = (a1*z + b1)/dnm
  float a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
  float b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;

  // y = (a2*z + b2)/dnm;
  float a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
  float b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;

  // a*z^2 + b*z + c = 0
  float a = a1 * a1 + a2 * a2 + dnm * dnm;
  float b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
  float c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - pushrod_lng * pushrod_lng);

  // discriminant
  float d = b * b - 4.0 * a * c;

  zValue = 0.5 * (-b + sqrt(d)) / a;
  xValue = (a1 * zValue + b1) / dnm;
  yValue = (a2 * zValue + b2) / dnm;

  if (DEBUG) {
    Serial.print("T1: ");
    Serial.println(theta1);
    Serial.print("T2: ");
    Serial.println(theta2);
    Serial.print("T3: ");
    Serial.println(theta3);
    Serial.print("Z1: ");
    Serial.println(z1);
    Serial.print("Z2: ");
    Serial.println(z2);
    Serial.print("Z3: ");
    Serial.println(z3);
    Serial.print("DNM: ");
    Serial.println(dnm);

  }
}

float GetAxisValue(float value, float zero, float gain, float deadZone, float minValue, float maxValue) 
{
    float rtValue = value - zero;
   
    if (rtValue < -1 * deadZone)
    {
      rtValue += deadZone;
    }
    else if (rtValue > deadZone)
    {
      rtValue -= deadZone;
    }
    else
    {
      rtValue = 0;
    }
    
    //apply gain
    rtValue = rtValue * gain;
  
    //constrain outputs to +- 100
    rtValue = constrain(rtValue, minValue, maxValue);  
    
    return rtValue;
}
