//Arduino code to do serial communication of imu and encoder data

#include "I2Cdev.h"
#include "digitalWriteFast.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define LED_PIN 13
bool blinkState = false;

int E2 = 6;
int M2 = 7;
int E1 = 5;                         
int M1 = 4;    

//encoder
#define LeftEncoderPin 2
bool LeftActualEncoded;
bool LeftLastEncoded;
#define RightEncoderPin 3 // TODO
bool RightActualEncoded;
bool RightLastEncoded;

int left_wheel;
int right_wheel;

#define message_size_output 10

// STRUCT
struct arduino_data {
  byte start;
  int lw, rw;
  byte checksum;
};

union {
  byte byte_buffer_output[message_size_output];
  arduino_data sensor_data;
} arduino_msg;

/////////////////////////////////////////// PER LEGGERE DATI DA SERIALE //////////////////////////////////////////////////////
#define message_size_input 18
struct motor_data {
  byte serial_start;
  double v, w;
  byte serial_checksum;
};

union {
  byte byte_buffer_input[message_size_input];
  motor_data serial_data;
} serial_msg;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


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

////////////////////////////////   LETTURA DATI DA SERIALE + COMANDO MOTORI   ///////////////////////////////////////////
void set_pwm(int value_left,int value_right);
void vw2pwm(double v, double w);
void v2pwm(double vl, double vr);

// ------------------  M E T H O D S -------------------------

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

//////////////////////////


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  //ENCODER
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

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

}

//// ================================================================
//// ===                    MAIN PROGRAM LOOP                     ===
//// ================================================================
//
int t_start,t_stop;
char incomingByte;   // for incoming serial data

void loop() {
  
    t_start = micros();

    

       //if (Serial.available() > 0) {
             // read the incoming byte:
             //incomingByte = Serial.read();
       //}else{
       // arduino_msg.sensor_data.lw = left_wheel;
       // arduino_msg.sensor_data.rw = right_wheel;
       // arduino_msg.sensor_data.start = 90;
       // arduino_msg.byte_buffer_output[message_size_output - 1] = CRC8(arduino_msg.byte_buffer_output, message_size_output - 2);
       // arduino_msg.sensor_data.checksum = arduino_msg.byte_buffer_output[message_size_output - 1];
       // Serial.write(arduino_msg.byte_buffer_output, message_size_output);
       //}
       
    if (Serial.read() == 5 ) {
      char local_buffer[17], checksum_input;
      Serial.readBytes(local_buffer,17);
      serial_msg.byte_buffer_input[0] = 5;
      int i;
      for (i=0;i<17;i++){
          serial_msg.byte_buffer_input[i+1] = local_buffer[i];
           }
      checksum_input = CRC8(serial_msg.byte_buffer_input, message_size_input - 2); 
      //if (checksum_input == local_buffer[16])
      //{
        double v,w;
        v = serial_msg.serial_data.v;
        w = serial_msg.serial_data.w;
        Serial.println(v);
        vw2pwm(v,w);
     // }           
    }
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    t_stop = micros();
  //Serial.println(t_stop-t_start);
  delay(25);
}

