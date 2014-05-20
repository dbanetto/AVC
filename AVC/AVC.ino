/*
   Code for reading raw values from line sensor. 
   Sensor should be about 3 mm away from the surface and 
   not touch it.
   Sensor wires are connected to Arduino pins 2,4,5,6,7,8,9,10
*/

// libraries
#include <QTRSensors.h>
#define NUM_SENSORS	8   // constant value

// initialize sensor class/variable
QTRSensorsRC qtr((unsigned char[]) {2,4,5,6,7,8,9,10}, NUM_SENSORS);
// array to store signals from all 8 sensors
unsigned int line_sensor_values[NUM_SENSORS];

int pwm_a = 3;   //PWM control for motor outputs 1 and 2 is on digital pin 3
int pwm_b = 11;  //PWM control for motor outputs 3 and 4 is on digital pin 11
int dir_a = 12;  //dir control for motor outputs 1 and 2 is on digital pin 12
int dir_b = 13;  //dir control for motor outputs 3 and 4 is on digital pin 13

int pin_short_a = A0;
int pin_short_b = A1;
int pin_long = A5;


void setup()
{
  pinMode(pwm_a, OUTPUT);  //Set control pins to be outputs
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);
  
  pinMode(pin_short_a,INPUT);
  pinMode(pin_short_b,INPUT);
  pinMode(pin_long,INPUT);
  
  //Set all sensor pins to INPUT
  pinMode( 2, INPUT);
  for (int i = 4; i < 11; i++)
  {
    pinMode( i, INPUT);   
  }

  analogWrite(pwm_a, 0);  // set motor voltages 0
  analogWrite(pwm_b, 0);
  
  Serial.begin(9600);

  boadcastRadio( "QUAD 1" );
}

double last_meanx = 0;
bool TURN_DECTED = false;
int TURN_SIDE = 0;
#define TURN_RIGHT -1
#define TURN_LEFT 1

void loop()
{
  double wheel_left = 1.0 , wheel_right = 1.0;
  qtr.read(line_sensor_values,QTR_EMITTERS_OFF);
  
  double meanx = linesensor();
  //Print Line Position
  
  Serial.print(meanx);
  Serial.println(" ");
  
  line_wheel( &wheel_left , &wheel_right , meanx );
  
  if (linesensor_right())
  {
    wheel_left = 1.0;
    wheel_right = -1.0;
    boadcastRadio( "QUAD 2" );
    TURN_DECTED = true;
    TURN_SIDE = TURN_RIGHT;
  } else if ( linesensor_none() )
  {
    
    wheel_left = 1.0;
    wheel_right = 1.0;
    if (last_meanx > 0)
    {
      wheel_right *= -1;
    } else if (last_meanx < 0)
    {
     wheel_left *= -1; 
    }
    boadcastRadio( "NONE!" );
  } else 
  {
    last_meanx = meanx;
  }

  if (TURN_DECTED)
  {
    if (TURN_SIDE == TURN_RIGHT)
    {
      if (meanx > -1.0 && meanx < -1.0)
      {
        TURN_SIDE = 0;
        TURN_DECTED = false;  
      } else {
            wheel_left = 1.0;
            wheel_right = -1.0;
      }
    } else {
      TURN_SIDE = 0;
      TURN_DECTED = false;
    }

  }

  wheel_move(wheel_left , wheel_right);
  //wheel_move(0 , 0);
  
  delay(100);
}

double offset = 3.5;
double linesensor ()
{
  double mean = 0;
  double n = 0;
  double dt = 0;
  
  for (int i = 0; i < 8; i++)
  {
      mean += (double)(4000 - line_sensor_values[i] ) * (n - offset);
      n++;
      dt += (double)(4000 - line_sensor_values[i] );
  }
  
  return (mean/dt);
}

bool linesensor_none ()
{
	
  for (int i = 0; i < 8; i++)
  {
      if ( (4000 - line_sensor_values[i] ) > 1024 ) // ADJUST ME!
      	return false;
  }
  return true;
}


bool linesensor_right ()
{
	
  for (int i = 0; i < 5; i++)
  {
      if ( (4000 - line_sensor_values[i] ) < 2048 ) // ADJUST ME!
      	return false;
  }
  return true;	
}


bool linesensor_left ()
{
	
  for (int i = 3; i < 8; i++)
  {
      if ( (4000 - line_sensor_values[i] ) < 2048 ) // ADJUST ME!
      	return false;
  }
  return true;
}

// Short Range Function

bool short_range (int pin)
{
  //Note range ~= 1024 with no object infront
  //Then drops to ~150 when there is an object in its range
   int range =  analogRead(pin);
   if (range < 360)
     return true;
   else
     return false;
}

//Long range Sensor

int long_range_sensor_data [10] = {0,0,0,0,0,0,0,0,0,0};
double update_long_range_sensor ( )
{
  int tmp = analogRead( pin_long );
  int outavg = 0;
  
  for (int i =0; i < 10; i++)
  {
    int tmp2 = long_range_sensor_data[i];
    long_range_sensor_data[i] = tmp;
    tmp = tmp2;
    
    outavg += tmp;
  }
  
  return (double)outavg / 10.0;
}

double long_range_sensor ()
{
  int outavg = 0;
  for (int i =0; i < 10; i++)
  {
    outavg += long_range_sensor_data[i];
  }
  
  return (double)outavg / 10.0;
  
}

// Short range Sensors

bool short_left ()
{
	return ( analogRead(pin_short_a) < 1024 );
}

bool short_right ()
{
	return ( analogRead(pin_short_b) < 1024 );
}

// Wheel Functions
double TURN_OFF_SET = 2.0;
void line_wheel (double* left, double* right , double meanx )
{
   if (meanx < 0)
   {
     //Left
     *left =  (meanx + TURN_OFF_SET) / TURN_OFF_SET;
   } else if (meanx > 0)
   {
     //right
     *right = (meanx - TURN_OFF_SET) / -TURN_OFF_SET;
   }
    
}

//Apply power to wheels

void wheel_move (double left , double right)
{
  if (left < 0)
  {
    digitalWrite(dir_a , LOW);
    left *= -1.5;
  } else {
    digitalWrite(dir_a , HIGH);
  }
  if (right < 0)
  {
  digitalWrite(dir_b , LOW);
    right *= -1.5;
  } else {
    digitalWrite(dir_b , HIGH);
  }
  
   analogWrite(pwm_a, (int)(96*left) );     // set motor voltage to max
   analogWrite(pwm_b, (int)(96*right));
}

// Networking 

void boadcastRadio (char transmission [])
{
 char frame[34];
 
 int nFrame = 33;
 unsigned int checkSumTotal = 0;
 unsigned int crc = 0;
 
 // delimter
 frame[0] = 0x7e;

 //length
 frame[1] = 0x00;
 frame[2] = 0x1e;

 // API ID
 frame[3] = 0x10;
 frame[4] = 0x01;

 // destination 64 bit address - broadcast
 frame[5] = 0x00;
 frame[6] = 0x13;
 frame[7] = 0xa2;
 frame[8] = 0x00;
 frame[9] = 0x40;
 frame[10] = 0x6a;
 frame[11] = 0x40;
 frame[12] = 0xa4;

 // destination 16 bit address - broadcast
 frame[13] = 0xff;
 frame[14] = 0xfe;
 
 // no. of hops
 frame[15] = 0x00;

 // option
 frame[16] = 0x01;
 
 // data!
 for(int i = 17; i<33; i++)
 {
   frame[i] = transmission[i-17];
 }

 // check sum
 for(int i = 3; i<nFrame; i++)
 {
   checkSumTotal += frame[i];
 }

 checkSumTotal = checkSumTotal & 0xff;
 crc = 0xff - checkSumTotal;
 frame[nFrame] = crc;
 
 nFrame++;

 for(int i=0; i<nFrame; i++)
 {
   Serial.write(frame[i]);
 }
}


void print_debug ()
{
  
  Serial.print ( short_range(pin_short_a) ) ;
  Serial.print ( " , " ) ;
  Serial.println ( short_range(pin_short_b) ) ;
  
  
  Serial.print ( "None : " ) ;
  Serial.println ( linesensor_none() ) ;
  Serial.print ( "LR : " ) ;
  Serial.print ( linesensor_left() ) ;
  Serial.print ( " , " ) ;
  Serial.println ( linesensor_right() ) ;
  

  for (int i = 0; i < 8; i++ ) {
    Serial.print ( line_sensor_values[i] ) ;
    Serial.print ( " , " ) ;
  }
  Serial.println ( ""  );
  
}

