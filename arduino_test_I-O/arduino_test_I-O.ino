//////////////  by MOTOR_CONTROL_SERIAL (Mirko Ferrati)
// # Connection:
// #        M1 pin  -> Digital pin 4
// #        E1 pin  -> Digital pin 5
// #        M2 pin  -> Digital pin 7
// #        E2 pin  -> Digital pin 6
// #        Motor Power Supply -> Centor blue screw connector(5.08mm 3p connector)
// #        Motor A  ->  Screw terminal close to E1 driver pin
// #        Motor B  ->  Screw terminal close to E2 driver pin
// # 
// # Note: You should connect the GND pin from the DF-MD v1.3 to your MCU controller. They should share the GND pins.
// #

#include "CmdMessenger.h"
#include "motor_control.h"
#include <string.h>

#define LEFT 0
#define RIGHT 1
#define Tprint 1000
#define prn_enc 0
#define pwm_default 255

#define v_min -3
#define v_max 3
#define w_min -10
#define w_max 10

volatile int coder[2] = {0,0};
int lastSpeed[2] = {0,0};

char field_separator = ',';
char command_separator = ';';

int E2 = 6;
int M2 = 7;
int E1 = 5;                         
int M1 = 4;                           
unsigned long timer = 0;                //print manager timer

long now;
long time_last_cmd;

// Attach a new CmdMessenger object to the default Serial port
CmdMessenger cmdMessenger = CmdMessenger(Serial, field_separator, command_separator);

// Commands we send from the Arduino to be received on the PC
enum
{
  kCOMM_ERROR    = 000, // Lets Arduino report serial port comm error back to the PC (only works for some comm errors)
  kACK           = 001, // Arduino acknowledges cmd was received
  kARDUINO_READY = 002, // After opening the comm port, send this cmd 02 from PC to check arduino is ready
  kERR           = 003, // Arduino reports badly formatted cmd, or cmd not recognised

  // Now we can define many more 'send' commands, coming from the arduino -> the PC, eg
  // kICE_CREAM_READY,
  // kICE_CREAM_PRICE,
  // For the above commands, we just call cmdMessenger.sendCmd() anywhere we want in our Arduino program.

  kSEND_CMDS_END, // Mustnt delete this line
};

void stop_motors();
void gest_motors();
void led_blinking();
void v2pwm_cmd();

int pwm_serial();
void set_pwm(int value_left,int value_right);
void ferma();
double get_v();
double get_w();
void vw2pwm(double v, double w);
void v2pwm(double vl, double vr);

messengerCallbackFunction messengerCallbacks[] = 
{
  gest_motors,    // 004 in this example
  stop_motors,
  led_blinking,
  v2pwm_cmd,
};
/////////////////////////// END /////////////////



//Arduino code to do serial communication of imu and encoder data

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "digitalWriteFast.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;

#define LED_PIN 13
bool blinkState = false;

// ------------------ M P U    C O N T R O L / S T A T U S   V A R S -------------------------
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// ------------------ I M U   O R I E N T A T I O N / M O T I O N    V A R S -------------------------
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
int gyro[3];            // [wx,wy,wz]           angular velocity
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ----------------- E N C O D E R S ----------------------------
#define LeftEncoderPin 3
bool LeftActualEncoded;
bool LeftLastEncoded;
#define RightEncoderPin 4 // TODO (Check that it isn't used by motor_control_serial)
bool RightActualEncoded;
bool RightLastEncoded;
int left_wheel;
int right_wheel;


// ---------------- S T R U C T    T O   R O S  ------------------
#define message_size 50
struct arduino_data {
  byte start;
  float qx, qy, qz, qw;
  float ax, ay, az;
  float wx, wy, wz;
  int lw, rw;
  byte checksum;
};

union {
  byte byte_buffer[message_size];
  arduino_data sensor_data;
} arduino_msg;



/// ===============================================================
/// ===               INTERRUPT DETECTION ROUTINE                ==
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}



/// ===========================================================
/// ===   D E F A U L T    C A L L B A C K S                ===
/// ===========================================================
void arduino_ready()
{
  // In response to ping. We just send a throw-away Acknowledgement to say "im alive"
  //cmdMessenger.sendCmd(kACK,"Arduino ready");
}

void unknownCmd()
{
  // Default response for unknown commands and corrupt messages
  //cmdMessenger.sendCmd(kERR,"Unknown command");
}




/// ===========================================================
/// ===   C A L L B A C K S    M E T H O D S                ===
/// ===========================================================
//////////////////////////////
void gest_motors()
{
  // Message data is any ASCII bytes (0-255 value). But can't contain the field
  // separator, command separator chars you decide (eg ',' and ';')
  //cmdMessenger.sendCmd(kACK,"cmd received");
  if ( cmdMessenger.available() )
  {
      time_last_cmd=millis();
        set_pwm(pwm_serial(),pwm_serial());
      
    }
}

//////////////////////////////
void stop_motors()
{ 
  ferma();
  Serial.println("Command stop_motors");
}

//////////////////////////////
void led_blinking()
{
  double time_on, time_off;

  if ( cmdMessenger.available() )
  {
         char buf[350] = { '\0' };
        cmdMessenger.copyString(buf, 350);
  time_on = atof(buf);
  if (time_on < 0)
    time_on = 0;
  else if (time_on > 255)
    time_on = 255;

  cmdMessenger.copyString(buf, 350);
  time_off = atof(buf);
  if (time_off < 0)
    time_off = 0;
  else if (time_off > 255)
    time_off = 255;

  digitalWrite(13, HIGH);
  delay(10*time_on);
  digitalWrite(13, LOW);
  delay(10*time_off); 
    
  Serial.print("Command led_blinking:");
        Serial.print(time_on);
        Serial.print(",");
        Serial.println(time_off);
  } 
}
 
//////////////////////////////
void v2pwm_cmd()
{
    if ( cmdMessenger.available() )
    {
      time_last_cmd=millis();
      double v = get_v();
      double w = get_w();
      vw2pwm(v,w);
      
      Serial.println("Command v2pwm_cmd");
    }
} 



/// ===========================================================
/// ===                M E T H O D S                        ===
/// ===========================================================
//////////////////////////////
void set_pwm(int value_left,int value_right)
{
  digitalWrite(M2,HIGH); 
  if (value_left <0)
    digitalWrite(M2,LOW);

  digitalWrite(M1,LOW);
  if (value_right <0)
          digitalWrite(M1,HIGH);
  
  analogWrite( E2, abs(value_left) );
  analogWrite( E1, abs(value_right) );

        Serial.print("Command set_pwm:");
        Serial.print(value_right);
        Serial.print(",");
        Serial.println(value_left);
}

//////////////////////////////
void ferma()
{
  analogWrite(E1, 0);  
  analogWrite(E2, 0);
  Serial.println("Command ferma");
}

//////////////////////////////
int pwm_serial()
{
       int pwm_cur; 
       pwm_cur= pwm_default;
       if (cmdMessenger.available())
       {
          char buf[350] = { '\0' };
          cmdMessenger.copyString(buf, 350); 
          pwm_cur=atoi(buf);
       }
       if (pwm_cur< -pwm_default)
       pwm_cur= -pwm_default;
  
  if (pwm_cur > pwm_default)
       pwm_cur= pwm_default;

       Serial.println("Command pwm_serial");
       return pwm_cur;
}

//////////////////////////
void LwheelSpeed()
{
  coder[LEFT]++;  //count the left wheel encoder interrupts
  Serial.print("Command LwheelSpeed:");
  Serial.println(coder[LEFT]);
}

//////////////////////////
void RwheelSpeed()
{
  coder[RIGHT]++; //count the right wheel encoder interrupts
  Serial.print("Command RwheelSpeed:");
  Serial.println(coder[RIGHT]);
}  

//////////////////////////
double get_v()
{
  double v; 
  v = 0;
  if (cmdMessenger.available())
  {
    char buf[350] = { '\0' };
    cmdMessenger.copyString(buf, 350); 
    v=atof(buf);
  }
  if (v < v_min)
    v = v_min;
  
  if (v > v_max)
    v= v_max;

  Serial.print("Command get_v:");
  Serial.println(v);
  return v;
}

//////////////////////////
double get_w()
{
   double w; 
   w = 0;
   if (cmdMessenger.available())
   {
     char buf[350] = { '\0' };
     cmdMessenger.copyString(buf, 350); 
     w=atof(buf);
   }
   if (w < w_min)
    w = w_min;
  
  if (w > w_max)
   w= w_max;

  Serial.print("Command get_w:");
  Serial.println(w);
  return w;
}

//////////////////////////////
void vw2pwm (double v, double w)
{
  double b = 0.09;
  double vl = v-(w/2.0)*b;
  double vr = v+(w/2.0)*b;
  
  v2pwm(vl, vr) ;
}

//////////////////////////////
void v2pwm(double vl, double vr)
{
  int vl_sign = vl>0 ? 1 : -1;
  double vl_abs = abs(vl);
  int vr_sign = vr>0 ? 1 : -1;
  double vr_abs = abs(vr);
  int left = 0;
  int right = 0;

  
  /* LEFT */
  if (vl_abs < 0.082)
  {
    left = 0;
  }
  else if (vl_abs < 0.41)
  {
    left  = vl_sign*(55+(vl_abs-0.082)/0.0033);
  }
  else if (vl_abs < 0.5467)
  {
    left  = vl_sign*(155+(vl_abs-0.41)/0.0014);
  }
  else
  {
    left  = vl_sign*255;
  }
  
  /* RIGHT */
  if (vr_abs < 0.082)
  {
    right = 0;
  }
  else if (vr_abs < 0.41)
  {
    right = vr_sign*(55+(vr_abs-0.082)/0.0033);
  }
  else if (vr_abs < 0.5467)
  {
    right = vr_sign*(155+(vr_abs-0.41)/0.0014);
  }
  else
  {
    right = vr_sign*255;
  }
  
  set_pwm(right, left);
}






// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
//////////////////////////////
void attach_callbacks(messengerCallbackFunction* callbacks)
{
  int i = 0;
  int offset = kSEND_CMDS_END;
  while(callbacks[i])
  {
    cmdMessenger.attach(offset+i, callbacks[i]);
    i++;
  }
}
void setup() {
  //////////////// by MOTOR_CONTROL_SERIAL (Mirko Ferrati)
  // Listen on serial connection for messages from the pc
  // Serial.begin(57600);  // Arduino Duemilanove, FTDI Serial
  Serial.begin(115200); // Arduino Uno, Mega, with AT8u2 USB

  // cmdMessenger.discard_LF_CR(); // Useful if your terminal appends CR/LF, and you wish to remove them
  cmdMessenger.print_LF_CR();   // Make output more readable whilst debugging in Arduino Serial Monitor
  
  // Attach default / generic callback methods
  cmdMessenger.attach(kARDUINO_READY, arduino_ready);
  cmdMessenger.attach(unknownCmd);
 
  // Attach my application's user-defined callback methods
  attach_callbacks(messengerCallbacks);
  pinMode(3, INPUT);
  pinMode(2, INPUT);
  digitalWrite(3, HIGH);
  digitalWrite(2, HIGH);
  attachInterrupt(LEFT, LwheelSpeed, CHANGE);    //init the interrupt mode for the digital pin 2
  attachInterrupt(RIGHT, RwheelSpeed, CHANGE);   //init the interrupt mode for the digital pin 3 

  arduino_ready();
  
  pinMode(M1, OUTPUT);   
  pinMode(M2, OUTPUT);

  pinMode(13, OUTPUT);
  
  now=0;
  time_last_cmd=0;
  /////////////// END ////////////
  
  //encoder  TO CHECK WITH BEFORE
  attachInterrupt(digitalPinToInterrupt(LeftEncoderPin),left_encoder,CHANGE);
  pinMode(LeftEncoderPin, INPUT);
  LeftActualEncoded = digitalRead(LeftEncoderPin);
  LeftLastEncoded = digitalRead(LeftEncoderPin);
  attachInterrupt(digitalPinToInterrupt(RightEncoderPin),right_encoder,CHANGE);
  pinMode(RightEncoderPin, INPUT);
  RightActualEncoded = digitalRead(RightEncoderPin);
  RightLastEncoded = digitalRead(RightEncoderPin);
  

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();


  // accelerometer
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);// by default is +-2g
  // supply your own gyro offsets here, scaled for min sensitivity
  //TODO
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(0);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

}

//// ================================================================
//// ===                    MAIN PROGRAM LOOP                     ===
//// ================================================================
//
int t_start,t_stop;
void loop() {
  t_start = micros();
  // by MOTOR_CONTROL_SERIAL
  // Process incoming serial data, if any
  
  now=millis();
  
  if(time_last_cmd!=0)
  if (now - time_last_cmd > 500)
  {
    stop_motors();
    Serial.print(now - time_last_cmd);
    Serial.print(" - ");
    time_last_cmd=0;
  }
  
  cmdMessenger.feedinSerialData();

  if( ((millis() - timer) > Tprint) && prn_enc==1 ){                   
    Serial.print("Elapsed time: ");
    Serial.println(millis()-timer);

    lastSpeed[LEFT] = coder[LEFT];   //record the latest speed value
    lastSpeed[RIGHT] = coder[RIGHT];
    coder[LEFT] = 0;                 //clear the data buffer
    coder[RIGHT] = 0;
    timer = millis();
    //delay(10);
  }
  //////////  END /////////////
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 4096) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // COMPONING MSG (acc in m/s^2 e vel rad/sec)
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetGyro(gyro, fifoBuffer);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
   
    //IMU
    arduino_msg.sensor_data.qx = 1;//122.35;
    arduino_msg.sensor_data.qy = 2;//q.y;
    arduino_msg.sensor_data.qz = 3;//q.z;
    arduino_msg.sensor_data.qw = 4;//q.w;
    arduino_msg.sensor_data.wx = 5;//gyro[0];
    arduino_msg.sensor_data.wy = 6;//gyro[1];
    arduino_msg.sensor_data.wz = 7;//gyro[2];
    arduino_msg.sensor_data.ax = aaReal.x;//#include <digitalWriteFast.h>
    arduino_msg.sensor_data.ay = 9;//aaReal.y;
    arduino_msg.sensor_data.az = 10;//aaReal.z;

    arduino_msg.sensor_data.start = 90;
    arduino_msg.byte_buffer[message_size - 1] = CRC8(arduino_msg.byte_buffer, message_size - 2);
    Serial.write(arduino_msg.byte_buffer, message_size);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
  t_stop = micros();
  //Serial.println(t_stop-t_start);
  delay(25);
}


//// ================================================================
//// ===                    O U R    M E T H O D S                ===
//// ================================================================


void left_encoder()
{
   //  LEFT ENCODER
    LeftActualEncoded = digitalReadFast(LeftEncoderPin);
    if (LeftActualEncoded != LeftLastEncoded)
    {
      left_wheel++;
      LeftLastEncoded = LeftActualEncoded;
      arduino_msg.sensor_data.lw = left_wheel;
    }
}


void right_encoder()
{
   //  RIGHT ENCODER
    RightActualEncoded = digitalReadFast(RightEncoderPin);
    if (RightActualEncoded != RightLastEncoded)
    {
      right_wheel++;
      RightLastEncoded = RightActualEncoded;
      arduino_msg.sensor_data.rw = 12;//right_wheel;
    }
}





//CRC-8 - algoritmo basato sulle formule di CRC-8 di Dallas/Maxim
//codice pubblicato sotto licenza GNU GPL 3.0
byte CRC8(const byte *data, byte len) {
  byte crc = 0x00;
  while (len--) {
    byte extract = *data++;
    for (byte tempI = 8; tempI; tempI--) {
      byte sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}
