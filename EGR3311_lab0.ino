//Department of Electrical Engineering
//Control and Soft-Computing Research Group
//Bayero University Kano.
//January, 2023
//DC MOTOR CONTROL MODULE

//ESP32 - DevKit V1 (30pins) PINS Configurations
const int VR_pin = 22; // Speed_pin variabily
const int motorPin1 = 4; // Direction_pin1
const int motorPin2 = 16; // Direction_pin2
const int speedIN = 33; // Input from Potentiometer

//Encoder Pins Settings
int Encoder_Pin1 = 03;
int Encoder_Pin2 = 21;
long  Encoder_Count;

//PWM SETTINGS
const int freq = 500;
const int resolution = 8;
const int VRChannel = 0;

//Timers
int samplingTime = 50; //in milliseconds
long currentTime = 0;
long timeLastCompute = 0;
long timeElapsed = 0;

//Speed Calculations
//Encoder Ticks; Pulse Per revolution PPR
//(120=600rpm, 240=300rpm, 408=150rpm, 540=130rpm, 900=80rpm)

const float PPR = 120; // I have fix it.
float rps, rpm, rpmKF = 0;
float numOfRevolution = 0;
float numPulse = 0;
int motorSpeed = 0;
float setpoint = 0;



void IRAM_ATTR Count_Pulses() 
{
 if (digitalRead(Encoder_Pin1) == digitalRead(Encoder_Pin2)) 
  {Encoder_Count--;} 
  else 
  {Encoder_Count++;}
}//End CountPulse

void setup() 
{
  Serial.begin(9600);

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);

  //Encoder Intrreput
  pinMode(Encoder_Pin2, INPUT_PULLDOWN);
  pinMode(Encoder_Pin1, INPUT_PULLDOWN);
  attachInterrupt(Encoder_Pin1, Count_Pulses, RISING);

  //PWM SETTINGS
  ledcSetup(VRChannel, freq, resolution);
  ledcAttachPin(VR_pin, VRChannel);

}//END SETUP

void loop() {

  currentTime = millis();
  timeElapsed = currentTime - timeLastCompute;
  if (timeElapsed >= samplingTime)
  {
    timeLastCompute = millis ();
    rpm = calculateOUTPUT(); //Get Speed

    //  ********INPUT VIA POT*******
    int  potValue = analogRead(speedIN);
    float uk = potValue/16;
      //  ****************************
    
    actuateMOTOR(uk);
    displayReadings(uk);
  }
}//END VOID LOOP

void actuateMOTOR(float uk)
{
  motorSpeed = (int) constrain(uk, 0, 255);
  digitalWrite(motorPin2, HIGH);
  digitalWrite(motorPin1, LOW);
  ledcWrite(VRChannel, motorSpeed);
  
}//END actuateMOTOR()

float calculateOUTPUT()
{
  //GET SPEED
  numPulse = Encoder_Count;
  numOfRevolution = numPulse / PPR;
  rps = (numOfRevolution / timeElapsed) * 1000;//convert millis to seconds
  float rpm1 = 60 * rps;
  
  Encoder_Count = 0;//RESET ENCOUNTER
  return rpm1;
}//END calculateSpeed()


void displayReadings(float uk)
{
  Serial.print(uk); Serial.print(" \t");
  Serial.println(rpm);
  convertToRpmAndDisplay(rpm);
}//END displayReadings()

void convertToRpmAndDisplay(float rpm){

  // float PI = 3.14159;
  float rad_per_sec = (rpm * 2 * PI) / 60;
  Serial.println(rad_per_sec);

}
