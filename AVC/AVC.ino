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

int SPEED = 108; // out of 255
// ~127 for the real course
// ~150 for secondary course w/ full bat 

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
  delay ( 1000 );
}

double last_meanx = 0;
bool TURN_DECTED = false;
int TURN_SIDE = 0;
bool TURN_PARTIAL = false;
#define TURN_RIGHT -1
#define TURN_LEFT 1

void loop()
{
  double wheel_left = 1.0 , wheel_right = 1.0;
  qtr.read(line_sensor_values,QTR_EMITTERS_OFF);
  update_long_range_sensor();

  double meanx = linesensor();
  
  line_wheel( &wheel_left , &wheel_right , meanx );
  
  if (linesensor_left())
  {
    wheel_left = -1.0;
    wheel_right = 1.0;
    boadcastRadio( "QUAD 2" );
    TURN_DECTED = true;
    TURN_SIDE = TURN_LEFT;
    TURN_PARTIAL = false;
  } else if ( linesensor_none() && !TURN_DECTED)
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
  } else if (!TURN_DECTED)
  {
    last_meanx = meanx;
  }

  if ( TURN_SIDE == TURN_RIGHT && linesensor_none())
  {
    TURN_DECTED = true;
    TURN_SIDE = TURN_RIGHT;
    TURN_PARTIAL = false;
    boadcastRadio( "RIGHT!" );
  }

  if ( short_left() && linesensor_none() && !TURN_DECTED)
  {
    TURN_SIDE = TURN_LEFT;
    wheel_left = -1.0;
    wheel_move(-1 , -1);
    delay(100);
    TURN_DECTED = true;
    TURN_PARTIAL = false;
  }

  if ( short_right() && (meanx > 1 || meanx < -1  ))
  {
    TURN_SIDE = TURN_LEFT;
    wheel_left = -1.0;
    wheel_move(-1 , -1);
    delay(300);
    TURN_DECTED = true;
    TURN_PARTIAL = false;
  }

    if ( long_range_sensor() > 350 )
  {
    TURN_SIDE = TURN_LEFT;
    wheel_left = -1.0;
    TURN_DECTED = true;
    TURN_PARTIAL = false;
  }

  if ( linesensor_right() && long_range_sensor() > 350 )
  {
    
    TURN_SIDE = TURN_RIGHT;
    //LOGIC!
    if (long_range_sensor() > 350)
    {
      TURN_SIDE = TURN_LEFT;
      wheel_left = -1.0;
      wheel_right = 1.0;
      boadcastRadio( "QUAD 3" );
      SPEED = 96;
    }

  }



  if (TURN_DECTED)
  {
    if (TURN_SIDE == TURN_RIGHT)
    {
      boadcastRadio( "RIGHT" );
      if (meanx > -1 && meanx < 1 && TURN_PARTIAL)
      {
        TURN_SIDE = 0;
        TURN_DECTED = false;  
      } else if (meanx > 2.5 || (linesensor_right_line() && !linesensor_left_line()) )
      {
      	TURN_PARTIAL = true;
      }
      wheel_left = 1.0;
      wheel_right = -1.0;
    } else if ( TURN_SIDE == TURN_LEFT )
    {
      boadcastRadio( "LEFT" );
      if (meanx > -1 && meanx < 1)
      {
        TURN_SIDE = 0;
        TURN_DECTED = false;  
      } else if (meanx < -2.5 || (!linesensor_right_line() && linesensor_left_line()) )
      {
      	TURN_PARTIAL = true;
      }
      wheel_left = -1.0;
      wheel_right = 1.0;
    } else {
      TURN_SIDE = 0;
      TURN_DECTED = false;
      TURN_PARTIAL = false;
    }
  }

  wheel_move(wheel_left , wheel_right);

  delay (10);
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
      if ( (4000 - line_sensor_values[i] ) > 2048 ) // ADJUST ME!
      	return false;
  }
  return true;
}


bool linesensor_left ()
{
	
  for (int i = 0; i < 4; i++)
  {
      if ( (4000 - line_sensor_values[i] ) < 1024 ) // ADJUST ME!
      	return false;
  }
  return true;	
}


bool linesensor_right  ()
{
	
  for (int i = 4; i < 8; i++)
  {
      if ( (4000 - line_sensor_values[i] ) < 1024 ) // ADJUST ME!
      	return false;
  }
  return true;
}

bool linesensor_right_line()
{
	  for (int i = 4; i < 7; i++)
  {
      if ( (4000 - line_sensor_values[i] ) > 1024 ) // ADJUST ME!
      	return true;
  }
  return false;
}

bool linesensor_left_line()
{
	  for (int i = 0; i < 3; i++)
  {
      if ( (4000 - line_sensor_values[i] ) > 1024 ) // ADJUST ME!
      	return true;
  }
  return false;
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
// Boolean sensor, ~150 when true
bool short_left ()
{
	return ( analogRead(pin_short_a) < 512 );
}

bool short_right ()
{
	return ( analogRead(pin_short_b) < 512 );
}

// Wheel Functions
double TURN_OFF_SET = 2.5;
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
	//If the value of left/right is negative
	// flip the polarity of the motor
  if (left < 0)
  {
    digitalWrite(dir_a , LOW);
    left *= -1;
  } else {
    digitalWrite(dir_a , HIGH);
  }
  if (right < 0)
  {
  digitalWrite(dir_b , LOW);
    right *= -1;
  } else {
    digitalWrite(dir_b , HIGH);
  }
	//Maximum is 255  
   analogWrite(pwm_a, (int)(SPEED*left) );     // set motor voltage to max
   analogWrite(pwm_b, (int)(SPEED*right));
}

// Networking (PACKET MAGIC)
// Thank you Aurther for giving us sample code to do this
void boadcastRadio (char transmission [])
{
 char packet[34];
 
 int packet_last = 33;
 unsigned int checkSumTotal = 0;
 unsigned int crc = 0;
 
 // XBee packet magic
 packet[0] = 0x7e;

 //length
 packet[1] = 0x00;
 packet[2] = 0x1e;

 // API ID
 packet[3] = 0x10;
 packet[4] = 0x01;

 // destination 64 bit address - broadcast
 packet[5] = 0x00;
 packet[6] = 0x13;
 packet[7] = 0xa2;
 packet[8] = 0x00;
 packet[9] = 0x40;
 packet[10] = 0x6a;
 packet[11] = 0x40;
 packet[12] = 0xa4;

 // destination 16 bit address - broadcast
 packet[13] = 0xff;
 packet[14] = 0xfe;
 
 // no. of hops
 packet[15] = 0x00;

 // option
 packet[16] = 0x01;
 
 // data!
 for(int i = 17; i<33; i++)
 {
   packet[i] = transmission[i-17];
 }

 // Sum all the bytes in the packet
 for(int i = 3; i<packet_last; i++)
 {
   checkSumTotal += packet[i];
 }

 //Limit the Checksum to 0-255
 checkSumTotal = checkSumTotal & 0xff;
 //Invert
 crc = 0xff - checkSumTotal;
 packet[packet_last] = crc;
 //Update to total length
 packet_last++;
 //Write out packet to Serial buffer
 for(int i=0; i<packet_last; i++)
 {
   Serial.write(packet[i]);
 }
}


void print_debug ()
{
  //Print Line Position
  
  Serial.print(last_meanx);
  Serial.println(" ");

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


