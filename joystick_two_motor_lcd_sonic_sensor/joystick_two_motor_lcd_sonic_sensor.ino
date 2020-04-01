#include <MultiStepper.h>
#include <AccelStepper.h>
#include <LiquidCrystal.h>

////////////////////////////////////////////////////////////////////////////////////////////
// Defining namespace for motors
#define stepPin1 12
#define dirPin1 13

#define stepPin2 10
#define dirPin2 11

#define motorInterfaceType 1


////////////////////////////////////////////////////////////////////////////////////////////
//Setting up button and joystick-inputs
#define y_read A0

#define swPin 2


////////////////////////////////////////////////////////////////////////////////////////////
//Setting up LCD-pins
#define lcd_D4 5 //on pin 5 in example, skal ha pwm
#define lcd_D5 6//4
#define lcd_D6 7 //3, skal ha pwm
#define lcd_D7 9 //2
#define lcd_rs 3 //12
#define lcd_e 4 //11 skal ha pwm


////////////////////////////////////////////////////////////////////////////////////////////
//ultrasonic sensor
const int sonic_out = A1;
const int sonic_range = A2;


////////////////////////////////////////////////////////////////////////////////////////////
//declaring variables
int y;
int x;
int speedForMotor;
int maximumSpeed = 1000;
boolean SW;
boolean step1Enab = false;
boolean step2Enab = true;
float current;
int stepper_acceleration = 1000;
int motor_runtime = 10000;
unsigned int range_mm;  
unsigned int range_cm;
unsigned int range_dm;
float range_m;
float range_showed;


////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTION DECLARATIONS
int joyStickToSpeed(int x, int maximumSpeed){
  x = x-512;
  if(x>=-300){
    if(x<=300){
      x = 0;
    }
  }
  float x_scaled = float(x)/522;
  x_scaled = x_scaled*maximumSpeed;
  int speedForMotor = int(x_scaled);
  return speedForMotor;
}

////////////////////////////////////////////////////////////////////////////////////////////
//Creating Objects

LiquidCrystal lcd(lcd_rs, lcd_e, lcd_D4, lcd_D5, lcd_D6, lcd_D7);
AccelStepper stepper1 = AccelStepper(motorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, stepPin2, dirPin2);

//*******************************************************************************************
void setup() {
  //Initialize Serial
  Serial.begin(9600); 
  
  //Setting pins
  //For reading joystick and sensor
  pinMode(y_read, INPUT);
  pinMode(sonic_range, OUTPUT);
  pinMode(sonic_out, INPUT);
  analogWrite(sonic_range, 0);

  lcd.begin(16, 2);
  lcd.print("Roterer(2)");
  lcd.setCursor(0, 1);
  lcd.print("Avstand (m):  0");
  
  // Set the maximum steps per second:
  stepper1.setMaxSpeed(maximumSpeed);
  stepper1.setAcceleration(stepper_acceleration); 
  stepper2.setMaxSpeed(maximumSpeed);
  stepper2.setAcceleration(stepper_acceleration);
  
  //check pins
}
//*******************************************************************************************

void loop() { 
  /////////////////////////////////////////////////////////
  //Read inputdata
  y = analogRead(y_read);
  SW = digitalRead(swPin);
  range_mm = analogRead(sonic_out);
  range_cm = range_mm/10;
  range_dm = range_cm/10;
  range_m = float(range_dm)/10;
  
  range_showed = range_m;
  
  lcd.setCursor(13, 1);
  lcd.print(range_showed);
  ////////////////////////////////////////////////////////
  // Handle Buttonpress
  if(SW == true){
    step1Enab = !step1Enab;
    step2Enab = !step2Enab;
    lcd.setCursor(0, 0);
    if(step1Enab){
      lcd.print("Klemmer(1)          ");
    }
    else{
      lcd.print("Roterer(2)         ");
    }
    while(SW){
      SW = digitalRead(swPin);
    }
  }
  
  /////////////////////////////////////////////////////////
  //Calculating and setting motorspeed, updating lcd
  speedForMotor = joyStickToSpeed(y, maximumSpeed);
  lcd.setCursor(10, 0);
  lcd.print("    ");
  if(speedForMotor!=0){
    lcd.setCursor(10, 0);
    lcd.print(speedForMotor);
    if(step1Enab){
      stepper1.setSpeed(-speedForMotor);
      stepper2.setSpeed(0);
    }
    else{
      stepper2.setSpeed(speedForMotor);
      stepper1.setSpeed(0);
    }
    for(int i=0;i<motor_runtime;i++){
      stepper1.runSpeed();
      stepper2.runSpeed();
    }
  }
}
