# Line-Following-Bot
my final line following bot not only walk on black with 5 IR sensors but also can detect junction and corner. It has PID control and weighted Error Calculations.  
# Journey Behind IT  
## for basic idea  
-get to know about arduino and rsp32  
(https://www.canva.com/design/DAGx-6gTFAs/K8FZEMR6VaM-S2LGIcFcg/viewutm_content=DAGx6gTFAs&utm_campaign=designshare&utm_medium=link2&utm_source=uniquelinks&utlId=h53b6c2367f)  
-know the basis of robots(sensor and motors):  
   https://www.youtube.com/watch?v=NRj6gzah7JA&list=PL4g1oAdmuCfqmYvURLzVFkMMUI7839biN&index=1&pp=iAQB  
   https://www.youtube.com/watch?v=pwwVOpXrazs&list=PL4g1oAdmuCfqmYvURLzVFkMMUI7839biN&index=2&pp=iAQB  
   https://www.youtube.com/watch?v=gizihSJ63o4&list=PL4g1oAdmuCfqmYvURLzVFkMMUI7839biN&index=3&pp=iAQB  
   https://www.youtube.com/watch?v=BnzUXag1qx8&list=PL4g1oAdmuCfqmYvURLzVFkMMUI7839biN&index=4&pp=iAQB  
-know about micro-controllers, PCB, UARt in detail  
   https://www.canva.com/design/DAGx-6gTFAs/K8FZEMR6VaM--S2LGIcFcg/view?utm _content=DAGx6gTFAs&utm_campaign=designshare&utm_medium=link2&utm_source=uniquelinks&utlId=h53b6c2367f  
   https://youtu.be/IyGwvGzrqp8?si=2z77m25CC4BParkK  
   -know about PID:
    https://youtu.be/tFVAaUcOm4I?si=8vZWMEeyeW-29dxb
-know about PCBS  
https://youtu.be/Z2LgmIGE2nI?si=HIU0AeMGXXB0Q6Lt  
-How to code Arduino  
https://youtu.be/zJ-LqeX_fLU?si=WDGOsD4ynYU08i4c  
# learn how to make line-following robot using 2 IR  
## and understand the code  
#define IR_SENSOR_RIGHT 11  
#define IR_SENSOR_LEFT 12  
#define MOTOR_SPEED 180  

//Right motor  
int enableRightMotor=6;  
int rightMotorPin1=7;  
int rightMotorPin2=8;  
//Left motor  
int enableLeftMotor=5;  
int leftMotorPin1=9;  
int leftMotorPin2=10;  
void setup()  
{
  //The problem with TT gear motors is that, at very low pwm value it does not even rotate.
  //If we increase the PWM value then it rotates faster and our robot is not controlled in that speed and goes out of line.
  //For that we need to increase the frequency of analogWrite.
  //Below line is important to change the frequency of PWM signal on pin D5 and D6
  //Because of this, motor runs in controlled manner (lower speed) at high PWM value.
  //This sets frequecny as 7812.5 hz.
  TCCR0B = TCCR0B & B11111000 | B00000010 ;
  
  // put your setup code here, to run once:  
  pinMode(enableRightMotor, OUTPUT);  
  pinMode(rightMotorPin1, OUTPUT);  
  pinMode(rightMotorPin2, OUTPUT);  
  pinMode(enableLeftMotor, OUTPUT);  
  pinMode(leftMotorPin1, OUTPUT);  
  pinMode(leftMotorPin2, OUTPUT);      
  pinMode(IR_SENSOR_RIGHT, INPUT);  
  pinMode(IR_SENSOR_LEFT, INPUT);  
  rotateMotor(0,0);   
}  
void loop()
{

  int rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  int leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

  //If none of the sensors detects black line, then go straight
  if (rightIRSensorValue == LOW && leftIRSensorValue == LOW)
  {
    rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
  }
  //If right sensor detects black line, then turn right
  else if (rightIRSensorValue == HIGH && leftIRSensorValue == LOW )
  {
      rotateMotor(-MOTOR_SPEED, MOTOR_SPEED); 
  }
  //If left sensor detects black line, then turn left  
  else if (rightIRSensorValue == LOW && leftIRSensorValue == HIGH )
  {
      rotateMotor(MOTOR_SPEED, -MOTOR_SPEED); 
  } 
  //If both the sensors detect black line, then stop 
  else 
  {
    rotateMotor(0, 0);
  }
}


void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }

  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }
  else 
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  }
  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));    
}  
## -then learn to make simple line following robot with 5 IR through gemini and understand it
  // --- Motor Control Pins (Adjust these for your specific driver/wiring) ---
#define MOTOR_R_A 5  // Motor Right Direction Pin 1 (Digital or PWM)
#define MOTOR_R_B 6  // Motor Right Direction Pin 2 (Digital or PWM)
#define MOTOR_R_PWM 9  // Motor Right Speed (PWM Pin)

#define MOTOR_L_A 4  // Motor Left Direction Pin 1 (Digital or PWM)
#define MOTOR_L_B 7  // Motor Left Direction Pin 2 (Digital or PWM)
#define MOTOR_L_PWM 10 // Motor Left Speed (PWM Pin)

// --- Sensor Pins (Adjust these for your specific wiring) ---
// Order: S0 (Extreme Left) to S4 (Extreme Right)
const int sensorPins[5] = {A0, A1, A2, A3, A4}; 

// --- Constants for Control ---
// BASE_SPEED is the speed when going straight (0-255 for Arduino PWM)
const int BASE_SPEED = 100; 

// Kp (Proportional Gain) is the most critical tuning parameter.
// Start with 0.5 and tune up or down. A higher Kp means more aggressive corrections.
const float Kp = 0.8; 

// The 'black' value is set to 0. The sensor is considered 'over the line' when it reads LOW.
const int ON_LINE_VALUE = LOW; 

// Weighted values for each sensor (used to calculate the error)
// S0: -40, S1: -20, S2: 0, S3: +20, S4: +40
const int sensorWeights[5] = {-40, -20, 0, 20, 40}; 

// Variables for the control loop
int sensorValues[5];
int error = 0;
int correction = 0;
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;

// --- Function to set motor speeds and direction ---
void setMotors(int leftSpeed, int rightSpeed) {
    // Left Motor (Assuming positive speed is FORWARD)
    if (leftSpeed >= 0) {
        digitalWrite(MOTOR_L_A, HIGH);
        digitalWrite(MOTOR_L_B, LOW);
        analogWrite(MOTOR_L_PWM, leftSpeed);
    } else {
        // Optional: Brake/Reverse logic if needed
        digitalWrite(MOTOR_L_A, LOW);
        digitalWrite(MOTOR_L_B, LOW); // Stop/Brake
        analogWrite(MOTOR_L_PWM, 0); 
    }

    // Right Motor
    if (rightSpeed >= 0) {
        digitalWrite(MOTOR_R_A, HIGH);
        digitalWrite(MOTOR_R_B, LOW);
        analogWrite(MOTOR_R_PWM, rightSpeed);
    } else {
        // Optional: Brake/Reverse logic if needed
        digitalWrite(MOTOR_R_A, LOW);
        digitalWrite(MOTOR_R_B, LOW); // Stop/Brake
        analogWrite(MOTOR_R_PWM, 0); 
    }
}

// --- Main Setup Function ---
void setup() {
    // Set all motor pins as output
    pinMode(MOTOR_R_A, OUTPUT);
    pinMode(MOTOR_R_B, OUTPUT);
    pinMode(MOTOR_R_PWM, OUTPUT);
    pinMode(MOTOR_L_A, OUTPUT);
    pinMode(MOTOR_L_B, OUTPUT);
    pinMode(MOTOR_L_PWM, OUTPUT);

    // Set all sensor pins as input
    for (int i = 0; i < 5; i++) {
        pinMode(sensorPins[i], INPUT);
    }

    // Initialize Serial for debugging
    Serial.begin(9600);
    Serial.println("Line Follower Initialized!");
    
    // Brief pause before starting the movement
    delay(1000); 
}

// --- Main Loop Function ---
void loop() {
    
    // 1. READ SENSORS AND CALCULATE ERROR
    error = 0;
    
    for (int i = 0; i < 5; i++) {
        sensorValues[i] = digitalRead(sensorPins[i]);
        
        // If the sensor is over the black line, add its weighted value to the error
        if (sensorValues[i] == ON_LINE_VALUE) {
            error += sensorWeights[i];
        }
    }

    // 2. CALCULATE CORRECTION
    // The correction is the proportional response to the error
    // 'correction' is the value to be ADDED/SUBTRACTED from the base speed
    correction = (int)(error * Kp);

    // 3. APPLY CORRECTION TO MOTOR SPEEDS
    
    // Positive 'error' means line is on the RIGHT, so the robot needs to turn LEFT.
    // Turn LEFT means increasing LEFT motor speed and decreasing RIGHT motor speed.
    leftMotorSpeed = BASE_SPEED + correction;
    rightMotorSpeed = BASE_SPEED - correction;

    // 4. CLAMP SPEEDS (Prevent exceeding max/min PWM value 0-255)
    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

    // 5. EXECUTE MOVEMENT
    setMotors(leftMotorSpeed, rightMotorSpeed);
    
    // --- Debug Output (Uncomment the lines below to see values in Serial Monitor) ---
    /*
    Serial.print("S_Val: ");
    for(int i=0; i<5; i++) {
      Serial.print(sensorValues[i]);
      Serial.print(" ");
    }
    Serial.print("| Error: ");
    Serial.print(error);
    Serial.print(" | Correction: ");
    Serial.print(correction);
    Serial.print(" | L_Speed: ");
    Serial.print(leftMotorSpeed);
    Serial.print(" | R_Speed: ");
    Serial.println(rightMotorSpeed);
    */
}  
## -then make final bot
  
