#include <Wire.h>
#include "HX711.h"
#include <aJSON.h>
#include <PID_v1.h>
#include <Scheduler.h>
#include <Servo.h>


//Serial configuration
String serialBuffer = "";         // a string to hold incoming data
boolean serialComplete = false;  // whether the string is complete

//Last print time
int last_print = 0;

//Due Pin configuration
//Motor pins
int motorPWMPin = 5;
int motorHall1Pin = 7;
int motorHall2Pin = 8;
int motorHall3Pin = 9;
int motorDRPin = 25;
int motorENPin = 26;

boolean motorENStatus = HIGH;
boolean motorDRStatus = LOW;

//Fan pins
int fanPWMPin = 10;

//Relay pins
int relay1Pin = 51;
int relay2Pin = 52;
int relay3Pin = 53;

//throttle pins
Servo throttle;
int throttlePin = 27;

//Resistor pins
int resistorPWMPin = 11;
long resistorStopTime = 0;
int resistorDRPin = 30;
boolean resistorLastDR = LOW;
//EventFlags
const static int events_n = 24;
boolean eventFlags[events_n];

//HX711 force transducer pins
HX711 torqueSensor(32, 33);   
HX711 weightSensor(34, 35);

//output data initialize
boolean motorPIDEnableFlag = true;
boolean fanPIDEnableFlag = true;
boolean relay1Status = 1;
boolean relay2Status = 1;
boolean relay3Status = 1;
float motorSpeed = 0;
float motorSpeedSet = 360;
float motorPWM = 0;
float motorPIDp = 0.8;
float motorPIDi = 0.4;
float fanSpeed = 0;
float fanSpeedSet = 360;
float fanPWM = 0;
float fanPIDp = 0.8;
float fanPIDi = 0.4;
float tcTempWall = 0;
float tcTempExhaust = 0;
float ambientTemp1 = 0;
float ambientTemp2 = 0;
float fueltankWeight = 0;
float fueltankFS = 0;
float fueltankZero = 0;
float armForce = 0;
float armZero = 0;
float armFS = 0;
float resistorPosition = 0;
float resistorZero = 0;
float resistorFS = 0;
int ambientRH = 0;
int ambientP = 0;
int throttlePosition=0;
int throttleZero = 76;
int throttleFS = 131;
int serialNumber = 0;


PID fanPID(&fanSpeed, &fanPWM, &fanSpeedSet, fanPIDp, fanPIDi, 0, DIRECT);
PID motorPID(&motorSpeed, &motorPWM, &motorSpeedSet, motorPIDp, motorPIDi, 0, DIRECT);
aJsonStream serial_stream(&Serial);

void setup() {
  // initialize serial port
  Serial.begin(9600);
  serialBuffer.reserve(200);// reserve 200 bytes for the inputString;

  //initialize pins
  pinMode(motorHall1Pin, INPUT);
  pinMode(motorHall2Pin, INPUT);
  pinMode(motorHall3Pin, INPUT);
  pinMode(motorPWMPin, OUTPUT);
  pinMode(motorDRPin, OUTPUT);
  pinMode(motorENPin, OUTPUT);
  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);
  pinMode(relay3Pin, OUTPUT);

  //initialize fan pins
  pinMode(fanPWMPin,OUTPUT);
  Wire.begin();        // join i2c bus (address optional for master)

  //initialize resistor pins
  pinMode(resistorPWMPin,OUTPUT);
  pinMode(resistorDRPin, OUTPUT);

  //initialize throttle pins
  throttle.attach(throttlePin);
  throttlePosition = throttleZero;
  //initialize PID controller
  motorSpeedSet = 360;
  motorPID.SetMode(AUTOMATIC);  
  analogWriteResolution(12);

  //initialize torque sensor
  torqueSensor.set_scale(-2116.f);     // this value is obtained by calibrating the scale with known weights
  torqueSensor.tare(10);               // reset the scale to 0
  weightSensor.set_scale(6817.f);     // this value is obtained by calibrating the scale with known weights;
  weightSensor.tare(); 
  
  // Scheduling initializaiton
  // "loop" is always started by default.
  Scheduler.startLoop(inputLoop);
  Scheduler.startLoop(outputLoop);
  Scheduler.startLoop(motorLoop);
  Scheduler.startLoop(fanLoop);
  Scheduler.startLoop(throttleLoop);
  Scheduler.startLoop(resistorLoop);
  Scheduler.startLoop(forceLoop);
}

//Processing sensor data
void loop() {
  //calculating PID
  // motorSpeed = measureMotorFrequency();
  //  motorPID.Compute();
  //analogWrite(motorPWMPin, motorPWM);
  //Final delay to compensate for a whole second
  serialNumber = millis();
  yield();
}

//Serial read-in loop
void inputLoop() {
  serialEvent(); //call the serial handling function
  if (serialComplete) {
    char serialRecBuffer[256];
    serialBuffer.toCharArray(serialRecBuffer, 256);
    aJsonObject *msg = aJson.parse(serialRecBuffer);
    //determine whether the parsing is successful.
    if (msg != NULL) {
      processMessage(msg);
    } else {
      Serial.println("parsing error!");
      Serial.println(serialRecBuffer);
    }
    aJson.deleteItem(msg);

    //Clear the serial buffer
    serialBuffer = "";
    serialComplete = false;
    serialRecBuffer[0] = (char)0;//Clear the char array
  }
  yield();
}


//Serial output loop
void outputLoop() {
  //Write JSON to Serial Port
  //Reserve JSON memory
  if (millis() - last_print > 823) {
    /* One second elapsed, send message. */
    aJsonObject *msg = createMessage();
    aJson.print(msg, &serial_stream);
    Serial.println(); /* Add newline. */
    aJson.deleteItem(msg);
    last_print = millis();
  }
  yield();
}

//Motor LOOP
void motorLoop() {
  analogWrite(motorPWMPin, motorPWM);
  digitalWrite(motorENPin, motorENStatus);
  digitalWrite(motorDRPin, motorDRStatus);
  digitalWrite(relay1Pin, relay1Status);
  digitalWrite(relay2Pin, relay2Status);
  digitalWrite(relay3Pin, relay3Status);
  yield();
}

void fanLoop(){
  analogWrite(fanPWMPin, fanPWM);
  Wire.requestFrom(2, 8);    // request 8 bytes from slave device #2
    while(Wire.available())    // slave may send less than requested
  { 
    int c = Wire.read(); // receive a byte as character
    c=c<<8;
    c |= Wire.read();
    fanSpeed = c;
    
    int d = Wire.read(); // receive a byte as character
    d=d<<8;
    d |= Wire.read();

    int e = Wire.read(); // receive a byte as character
    e=e<<8;
    e |= Wire.read();

    int f = Wire.read(); // receive a byte as character
    f=f<<8;
    f |= Wire.read();
  motorSpeed = (d+e+f)/3.0;
  }
  yield();
  };

void forceLoop(){
  armForce = torqueSensor.get_units();
  fueltankWeight = weightSensor.get_units();
  yield();
  };

  void throttleLoop(){
    throttle.write(throttlePosition);
    yield();
    };
  void resistorLoop(){
  //  if (millis()-resistorStopTime > 0){analogWrite(resistorPWMPin,0);};
    yield();
    };

void processMessage(aJsonObject * msg)
{
  const static int booleans_n = 5;
  const static int floats_n = 14;
  const static int ints_n = 2;
  aJsonObject *booleanCMD = aJson.getObjectItem(msg, "booleanCMD");
  aJsonObject *floatCMD = aJson.getObjectItem(msg, "floatCMD");
  aJsonObject *intCMD = aJson.getObjectItem(msg, "intCMD");
  aJsonObject *eventCMD = aJson.getObjectItem(msg, "eventCMD");

  /*  boolean command parser
  Process message like: { "booleanCMD": [true,true,false,false,false]} */
  if (!eventCMD) {
  //  Serial.println("no event CMD");
  } else {
    if (eventCMD->type != aJson_Array) {
      Serial.print("invalid data type ");
      Serial.println(eventCMD->type, DEC);
    } else {
      int events_n_real = aJson.getArraySize(eventCMD);
      if (events_n != events_n_real) {
        Serial.print("invalid event array size ");
        Serial.println(events_n_real);
      } else {
        aJsonObject *c = eventCMD->child;
        for (int i = 0; i < events_n_real; i++) {
          eventFlags[i] = c->valuebool;
          c = c->next;
//          Serial.print("getting events ");
//          Serial.print(i);
//          Serial.println(eventFlags[i]);
        }
        //get these events working
        if (eventFlags[0]) {
          setBoolean();
        };
        if (eventFlags[1]) {
          setFloat();
        };
        if (eventFlags[2]) {
          setInt();
        };
        if (eventFlags[3]) {
          setMotorSpeed();
        };
        if (eventFlags[4]) {
          setMotorPWM();
        };
        if (eventFlags[5]) {
          startMotor();
        };
        if (eventFlags[6]) {
          stopMotor();
        };
        if (eventFlags[7]) {
          inverseMotorDR();
        };
        if (eventFlags[8]) {
          runOneRev();
        };
        if (eventFlags[9]) {
          startFan();
        };
        if (eventFlags[10]) {
          stopFan();
        };
        if (eventFlags[11]) {
          setFanPWM();
        };
        if (eventFlags[12]) {
          setFanSpeed();
        };
        if (eventFlags[13]) {
          feedInThrottle();
        };
        if (eventFlags[14]) {
          feedOutThrottle();
        };
        if (eventFlags[15]) {
          setThrottleZero();
        };
        if (eventFlags[16]) {
          setThrottleFS();
        };
        if (eventFlags[17]) {
          feedInResistor();
        };
        if (eventFlags[18]) {
          feedOutResistor();
        };
        if (eventFlags[19]) {
          setResistorZero();
        };
        if (eventFlags[20]) {
          setResistorFS();
        };
        if (eventFlags[21]) {
          burnGLowplug();
        };
        if (eventFlags[22]) {
          switch2ModeA();
        };
        if (eventFlags[23]) {
          switch2ModeB();
        };

      }
    }
  }

  if (!booleanCMD) {
  //  Serial.println("no boolean CMD");
  } else {
    if (booleanCMD->type != aJson_Array) {
      Serial.print("invalid data type ");
      Serial.println(booleanCMD->type, DEC);
    } else {
      int booleans_n_real = aJson.getArraySize(booleanCMD);
      const static int booleans_n = 5;
      if (booleans_n != booleans_n_real) {
        Serial.print("invalid bool array size ");
        Serial.println(booleans_n_real);
      } else {
        aJsonObject *c = booleanCMD->child;
        boolean bo[booleans_n_real];
        for (int i = 0; i < booleans_n_real; i++) {
          bo[i] = c->valuebool;
          c = c->next;
//          Serial.print("getting booleans ");
//          Serial.print(i);
//          Serial.println(bo[i]);
        }
        //get these values set to the table
        motorPIDEnableFlag = bo[0];
        fanPIDEnableFlag = bo[1];
        relay1Status = bo[2];
        relay2Status = bo[3];
        relay3Status = bo[4];
      }
    }
  }

  /*float command parser*/
  /* Process message like: { "floatCMD": [0.1,0.2,0.1,0.2,0.3],"booleanCMD": [true,true,false,false,false]} */
  if (!floatCMD) {
   // Serial.println("no float CMD");
  } else {
    if (floatCMD->type != aJson_Array) {
      Serial.print("invalid data type ");
      Serial.println(floatCMD->type, DEC);
    } else {
      int floats_n_real = aJson.getArraySize(floatCMD);
      const static int floats_n = 14;
      if (floats_n != floats_n_real) {
        Serial.print("invalid float array size ");
        Serial.println(floats_n_real);
      } else {
        aJsonObject *c = floatCMD->child;
        float fl[floats_n_real];
        for (int i = 0; i < floats_n_real; i++) {
          fl[i] = (c->valuefloat);
          c = c->next;
//          Serial.print("getting floats ");
//          Serial.print(i);
//          Serial.println(fl[i]);
        }
        //get these values set to the table
        motorSpeedSet = fl[0];
        motorPWM = fl[1];
        motorPIDp = fl[2];
        motorPIDi = fl[3];
        fanSpeedSet = fl[4];
        fanPWM = fl[5];
        fanPIDp = fl[6];
        fanPIDi = fl[7];
        fueltankFS = fl[8];
        fueltankZero = fl[9];
        armZero = fl[10];
        armFS = fl[11];
        resistorZero = fl[12];
        resistorFS = fl[13];
      }
    }
  }

  /*int command parser*/
  /* Process message like: { "floatCMD": [0.1,0.2,0.1,0.2,0.3,0.1,0.2,0.1,0.2,0.3,0.1,0.2,0.1,0.2],"booleanCMD": [true,true,false,false,false],"intCMD": [1251,222]} */
  if (!intCMD) {
   // Serial.println("no int CMD");
  } else {
    if (intCMD->type != aJson_Array) {
      Serial.print("invalid data type ");
      Serial.println(intCMD->type, DEC);
    } else {
      int ints_n_real = aJson.getArraySize(intCMD);
      const static int ints_n = 2;
      if (ints_n != ints_n_real) {
        Serial.print("invalid int array size ");
        Serial.println(ints_n_real);
      } else {
        aJsonObject *c = intCMD->child;
        int in[ints_n_real];
        for (int i = 0; i < ints_n_real; i++) {
          in[i] = c->valueint;
          c = c->next;
//          Serial.print("getting ints ");
//          Serial.print(i);
//          Serial.println(in[i]);
        }
        //get these values set to the table
        throttleZero = in[0];
        throttleFS = in[1];
      }
    }
  }
}


/* Create message like: { "floatdata": [2.2, 1.1, 2.2] } */
aJsonObject *createMessage()
{
  aJsonObject *msg = aJson.createObject();
  float floData[] = {
    motorSpeed,
    motorSpeedSet,
    motorPWM,
    motorPIDp,
    motorPIDi,
    fanSpeed,
    fanSpeedSet,
    fanPWM,
    fanPIDp,
    fanPIDi,
    tcTempWall,
    tcTempExhaust,
    ambientTemp1,
    ambientTemp2,
    fueltankWeight,
    fueltankFS,
    fueltankZero,
    armForce,
    armZero,
    armFS,
    resistorPosition,
    resistorZero,
    resistorFS
  };
  boolean booData[] = {
    motorPIDEnableFlag,
    fanPIDEnableFlag,
    relay1Status,
    relay2Status,
    relay3Status
  };
  int intData[] = {
    ambientRH,
    ambientP,
    throttlePosition,
    throttleZero,
    throttleFS,
    serialNumber
  };
  aJsonObject *booleandata = aJson.createBooleanArray(booData, 5);
  aJsonObject *floatdata = aJson.createFloatArray(floData, 23);
  aJsonObject *intdata = aJson.createIntArray(intData, 6);
  aJson.addItemToObject(msg, "SN", aJson.createItem(serialNumber));
  aJson.addItemToObject(msg, "floatdata", floatdata);
  aJson.addItemToObject(msg, "booleandata", booleandata);
  aJson.addItemToObject(msg, "intdata", intdata);
  return msg;
}

float measureMotorFrequency() {
  unsigned long duration = 0;
  float frequency;
  for (int i = 0; i < 5; i++) {
    duration += pulseIn(motorHall1Pin, HIGH, 100000);
  }
  for (int i = 0; i < 5; i++) {
    duration += pulseIn(motorHall2Pin, HIGH, 100000);
  }
  for (int i = 0; i < 5; i++) {
    duration += pulseIn(motorHall3Pin, HIGH, 100000);
  }
  if (duration > 0) {
    frequency = 15.0 / (duration / 15.0 * 2) * 1000000;
  } else {
    frequency = 0;
  }
  return frequency;
};

//A funciton used to read-in serial event
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    serialBuffer += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      serialComplete = true;
    }
  }
}
void setBoolean() {};
void setFloat() {};
void setInt() {};
void setMotorSpeed() {};
void setMotorPWM() {};

void startMotor() {
  motorPWM = 1200;
  };

void stopMotor() {
  motorPWM=0;
  };

void inverseMotorDR() {
  motorDRStatus = !motorDRStatus;
  };
void runOneRev() {};
void startFan() {
 fanPWM = 2000;
  };
void stopFan() {
  fanPWM = 0;
  };
void setFanPWM() {};
void setFanSpeed() {};
void feedInThrottle() {
if (throttlePosition +2 <= throttleFS){ throttlePosition+=2;}else{throttlePosition=throttleFS;};
};
void feedOutThrottle() {
  if (throttlePosition -2 >= throttleZero){ throttlePosition -=2;}else{throttlePosition=throttleZero;};
};
void setThrottleZero() {
 throttlePosition=throttleZero; };
void setThrottleFS() {};

void feedInResistor() {
  digitalWrite(resistorDRPin, LOW);
  analogWrite(resistorPWMPin,2048);
  delay(200);
      if (resistorLastDR == HIGH){
    delay(800);
      };
  analogWrite(resistorPWMPin,0);
  resistorStopTime = millis()+200;
  resistorLastDR = LOW;
  resistorPosition += 1.8;
  };
  
void feedOutResistor() {
  digitalWrite(resistorDRPin,HIGH);
  analogWrite(resistorPWMPin,2048);
    delay(200);
    if (resistorLastDR == LOW){
    delay(800);
      };
  analogWrite(resistorPWMPin,0);
  resistorStopTime = millis()+200;
  resistorLastDR = HIGH;
  resistorPosition -= 1.8;
  };
  
void setResistorZero() {};
void setResistorFS() {};
void burnGLowplug() {};
void switch2ModeA() {
  relay1Status = 0;
  relay2Status = 0;
  relay3Status = 0;
  };
void switch2ModeB() {
    relay1Status = 1;
  relay2Status = 1;
  relay3Status = 1;
  };
