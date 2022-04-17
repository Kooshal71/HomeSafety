//Display
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

//LED and Buuzer
#define RED 13
#define GREEN 12
#define YELLOW 11
#define BLUE 10
#define BUZZER 9

//Distance
#define echoPin 2 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 3 //attach pin D3 Arduino to pin Trig of HC-SR04
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

//Flame
const int sensorMin = 0;     // sensor minimum
const int sensorMax = 1024;  // sensor maximum

//Gas
#define         MQ_PIN                       (0)     //define which analog input channel you are going to use
#define         RL_VALUE                     (5)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
                                      /*cablibration phase*/
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
#define         GAS_LPG                      (0)
#define         GAS_CO                       (1)
#define         GAS_SMOKE                    (2)
float           LPGCurve[3]  =  {2.3,0.21,-0.47};   //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent"
                                                    //to the original curve. 
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59) 
float           COCurve[3]  =  {2.3,0.72,-0.34};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
float           SmokeCurve[3] ={2.3,0.53,-0.44};    //two points are taken from the curve. 
                                                    //with these two points, a line is formed which is "approximately equivalent" 
                                                    //to the original curve.
                                                    //data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)                                                     
float           Ro           =  10;                 //Ro is initialized to 10 kilo ohms

void setup() {
  
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  
  //Display
  lcd.init();
  lcd.backlight();

  //LED and Buzzer
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(BLUE, OUTPUT);
  
  //Distance
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT

  //Gas
  lcd.print("Calibrating...");                
  Ro = MQCalibration(MQ_PIN); //Calibrating the sensor. Please make sure the sensor is in clean air 
  delay(1000);
  lcd.setCursor(0,0);
  lcd.print("Calibration done");
  delay(1000);
  lcd.setCursor(0,3);
  lcd.print("Ro = ");
  lcd.print(Ro);
  lcd.print(" kohm");
  delay(5000);

  //Flame contains no setup config
  
}

void loop() {
  //Distance
  /*
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2000);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10000);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  lcd.setCursor(0,0);
  lcd.print("Distance: ");
  lcd.print(distance);
  lcd.print(" cm");
  delay(1000);
  lcd.clear();
  */
  
  //Gas
  /*
  lcd.setCursor(0, 0);
  lcd.print("LPG:");
  lcd.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG) );
  //lcd.print( "ppm" );
  lcd.print("     ");  
  lcd.setCursor(9, 0);
  lcd.print("CO:"); 
  lcd.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO) );
  //lcd.print( "ppm" );
  lcd.print("       "); 
  lcd.setCursor(0, 1);  
  lcd.print("SMOKE:"); 
  lcd.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE) );
  //lcd.print( "ppm" );
  lcd.print("         ");
  delay(8000);
  lcd.clear();
  */

  //Flame
  int sensorReading = analogRead(A1);
  // map the sensor range (four options):
  // ex: 'long int map(long int, long int, long int, long int, long int)'
  int range = map(sensorReading, sensorMin, sensorMax, 0, 3);
  digitalWrite(GREEN, HIGH);
  // range value:
  switch (range) {
  case 0:    // A fire closer than 1.5 feet away.
    lcd.print("** Close Fire **");
    digitalWrite(RED, HIGH);
    tone(BUZZER, 1000);
    delay(500);
    noTone(BUZZER);
    digitalWrite(RED, LOW);
    break;
  case 1:    // A fire between 1-3 feet away.
    lcd.print("** Distant Fire **");
    digitalWrite(RED, HIGH);
    tone(BUZZER, 1000);
    delay(1000);
    noTone(BUZZER);
    digitalWrite(RED, LOW);
    break;
  case 2:    // No fire detected.
    //Insert Gas Program
    int LPG = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG);
    int CO = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO);
    int smoke = MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE);
    if(LPG || CO || smoke)
    {
      digitalWrite(YELLOW, HIGH);
      tone(BUZZER, 1000);
      lcd.setCursor(0, 0);
      lcd.print("LPG:");
      lcd.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_LPG) );
      //lcd.print( "ppm" );
      lcd.print("     ");  
      lcd.setCursor(9, 0);
      lcd.print("CO:"); 
      lcd.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_CO) );
      //lcd.print( "ppm" );
      lcd.print("       "); 
      lcd.setCursor(0, 1);  
      lcd.print("SMOKE:"); 
      lcd.print(MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_SMOKE) );
      //lcd.print( "ppm" );
      lcd.print("         ");
      delay(3000);
      noTone(BUZZER);
      lcd.clear();
    }
    else
    {
        digitalWrite(YELLOW, LOW);
        //Insert Distance Program
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2000);
        // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10000);
        digitalWrite(trigPin, LOW);
        // Reads the echoPin, returns the sound wave travel time in microseconds
        duration = pulseIn(echoPin, HIGH);
        // Calculating the distance
        distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
        if(distance < 5)
        {
          lcd.print("TANK FULL");
          digitalWrite(BLUE, HIGH);
          tone(BUZZER, 1000);
          delay(1000);
          digitalWrite(BLUE, LOW);
          lcd.clear();
          noTone(BUZZER);
        }
        else if(distance > 50)
        {
          lcd.print("TANK EMPTY");
          digitalWrite(BLUE, HIGH);
          delay(2000);
          lcd.clear();
        }
        else
        {
          digitalWrite(BLUE, LOW);
          lcd.setCursor(0,0);
          lcd.print("Water Left:");
          lcd.setCursor(0,3);
          int percent = 100 - distance*2;
          lcd.print(distance);
          lcd.print(" %");
          delay(1000);
          lcd.clear();
        }
        // Displays the distance on the Serial Monitor
    }
    break;
  }
  delay(1000);  // delay between reads
  lcd.clear();

  
}

float MQCalibration(int mq_pin)
{
  int i;
  float val=0;
 
  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value
 
  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according to the chart in the datasheet 
 
  return val; 
}

float MQRead(int mq_pin)
{
  int i;
  float rs=0;
 
  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}

int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_LPG ) {
     return MQGetPercentage(rs_ro_ratio,LPGCurve);
  } else if ( gas_id == GAS_CO ) {
     return MQGetPercentage(rs_ro_ratio,COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
     return MQGetPercentage(rs_ro_ratio,SmokeCurve);
  }    
 
  return 0;
}

int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}

float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}
