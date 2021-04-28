//Motor Set A
#define ENCA_A 21 // YELLOW
#define ENCB_A 20 // WHITE
#define PWM_A 9
#define IN2_A 52
#define IN1_A 53
#define PWM_B 8
#define IN2_B 49
#define IN1_B 48

//Motor Set b
#define ENCA_D 19// YELLOW
#define ENCB_D 18 // WHITE
#define PWM_D 11
#define IN2_D 32
#define IN1_D 33  
#define PWM_C 10
#define IN2_C 30
#define IN1_C 31

#define trigPin 13
#define echoPin 12

int maxpower = 150;
int minpower = 100;

int pos_A = 0;
long prevT_A = 0;
float eprev_A = 0;
float eintegral_A = 0;
long pos_D = 0;
long prevT_D = 0;
float eprev_D = 0;
float eintegral_D = 0;

  // PID constants
float kp_A = 0.08;
float kd_A = 0.02;
float ki_A = 0;

float kp_D = 0.08;
float kd_D = 0.02;
float ki_D = 0; 

int count=0;
int stopcount = 0;
 
float duration, distance;

int StopPos = 0;
  
void setup() {
  Serial.begin(9600);
  pinMode(ENCA_A,INPUT_PULLUP);
  pinMode(ENCB_A,INPUT_PULLUP);
  pinMode(PWM_A, OUTPUT);
  pinMode(IN1_A, OUTPUT);
  pinMode(IN2_A, OUTPUT);
  pinMode(PWM_B, OUTPUT);
  pinMode(IN1_B, OUTPUT);
  pinMode(IN2_B, OUTPUT);

  pinMode(ENCA_D,INPUT_PULLUP);
  pinMode(ENCB_D,INPUT_PULLUP);
  pinMode(PWM_D, OUTPUT);
  pinMode(IN1_D, OUTPUT);
  pinMode(IN2_D, OUTPUT);
  pinMode(PWM_C, OUTPUT);
  pinMode(IN1_C, OUTPUT);
  pinMode(IN2_C, OUTPUT);  
  
  attachInterrupt(digitalPinToInterrupt(ENCA_A),readEncoder_A,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB_D),readEncoder_D,RISING);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}


void loop() {
  DriveStright(1910*3,1910*3); // stright for 3 meters
  delay(1000);
  DriveTurn(-1305,1305); // 90 degree left turn 
  delay(1000);
  DriveStright(1910,1910);
  delay(1000);
  DriveTurn(-1305,1305);
}

bool checkfront(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Measure the response from the HC-SR04 Echo Pin
 
  duration = pulseIn(echoPin, HIGH);
  
  // Determine distance from duration
  // Use 343 metres per second as speed of sound
  
  distance = (duration / 2) * 0.0343;
  
  // Send results to Serial Monitor
 
  if (distance >= 400 || distance <= 2) {
     Serial.println("Out of range");
     return false;
  }
  else if (distance < 50){
    return true;
    Serial.print(distance);
    Serial.println(" cm");
  }else{
    return false;
    }
    
  
  
  }
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}


void DriveStright (int target_a, int target_b){

  int e_A = pos_A-target_a;
  int e_D = pos_D-target_b;
  unsigned long currentTime = millis();
  
  
  while (fabs(e_A) >= 0 or fabs(e_D) >= 0 ){
    long currT_A = micros();
    float deltaT_A = ((float) (currT_A - prevT_A))/( 1.0e6 );
    prevT_A = currT_A;
      
    long currT_D = micros();
    float deltaT_D = ((float) (currT_D - prevT_D))/( 1.0e6 );
    prevT_D = currT_D;
          
    // error
    e_A = pos_A-target_a;
    e_D = pos_D-target_b;
          
    // derivative
    float dedt_A = (e_A-eprev_A)/(deltaT_A);
    float dedt_D = (e_D-eprev_D)/(deltaT_D);
          
    // integral
    eintegral_A = eintegral_A + e_A*deltaT_A;
    eintegral_D = eintegral_D + e_D*deltaT_D;
          
    // control signal
    float u_A = kp_A*e_A + kd_A*dedt_A + ki_A*eintegral_A;
    float u_D = kp_D*e_D + kd_D*dedt_D + ki_D*eintegral_D;
  
    // motor power
    float pwr_A  = fabs(u_A );
    if( pwr_A  > maxpower ){
      pwr_A  = maxpower;
    }
    else if ( pwr_A < minpower){
      pwr_A = minpower;
    }
    
          
    float pwr_D = fabs(u_D);
    if( pwr_D > maxpower ){
      pwr_D = maxpower;
    }
    else if ( pwr_D < minpower){
      pwr_D = minpower;
    }
         
    //motor direction
    int dir_A = 1;
    if(u_A<0){
      dir_A = -1;
    }
          
     int dir_D = 1;
     if(u_D<0){
      dir_D = -1;
     }

     if (checkfront()==true){
      Serial.print(stopcount);
      if(stopcount == 0 ){
        StopPos = pos_A+360;
        stopcount++;
        }
      Break(StopPos,StopPos);
      continue;
      }
     
     while (millis()- currentTime < 500){  //move at max speed to overcome static friction
        setMotor(dir_A,255,PWM_A,IN1_A,IN2_A);
        setMotor(dir_A,255,PWM_B,IN1_B,IN2_B);
        setMotor(dir_D,255,PWM_D,IN1_D,IN2_D);
        setMotor(dir_D,255,PWM_C,IN1_C,IN2_C);
        Serial.print(" ");
        Serial.print(target_a);
        Serial.print(" ");
        Serial.print(target_b);
        Serial.print(" ");
        Serial.print(pos_A);
        Serial.print(" ");
        Serial.print(pos_D);
        Serial.println(); 
     }
     setMotor(dir_A,pwr_A,PWM_A,IN1_A,IN2_A);
     setMotor(dir_A,pwr_A,PWM_B,IN1_B,IN2_B);
     setMotor(dir_D,pwr_D,PWM_D,IN1_D,IN2_D);
     setMotor(dir_D,pwr_D,PWM_C,IN1_C,IN2_C);   
     // store previous error
     eprev_A = e_A;
     eprev_D = e_D;

     Serial.print(" ");
     Serial.print(target_a);
     Serial.print(" ");
     Serial.print(target_b);
     Serial.print(" ");
     Serial.print(pos_A);
     Serial.print(" ");
     Serial.print(pos_D);
     Serial.println(); 

     if (fabs(e_A) < 10 or fabs(e_D) < 10 ){  
       break; 
     }

     //if (fabs(e_A) < fabs(target_a * 0.1) and fabs(e_D) < fabs(target_b * 0.1) ){  
      // break; 
     //}
          
  }
  pos_A=0;
  pos_D=0;
  
  setMotor(0,0,PWM_A,IN1_A,IN2_A);
  setMotor(0,0,PWM_B,IN1_B,IN2_B);
  setMotor(0,0,PWM_D,IN1_D,IN2_D);
  setMotor(0,0,PWM_C,IN1_C,IN2_C); 
}

void DriveTurn (int target_a, int target_b){

  int e_A = pos_A-target_a;
  int e_D = pos_D-target_b;
  unsigned long currentTime = millis();
  
  
  while (fabs(e_A) >= 0 or fabs(e_D) >= 0 ){
    long currT_A = micros();
    float deltaT_A = ((float) (currT_A - prevT_A))/( 1.0e6 );
    prevT_A = currT_A;
      
    long currT_D = micros();
    float deltaT_D = ((float) (currT_D - prevT_D))/( 1.0e6 );
    prevT_D = currT_D;
          
    // error
    e_A = pos_A-target_a;
    e_D = pos_D-target_b;
          
    // derivative
    float dedt_A = (e_A-eprev_A)/(deltaT_A);
    float dedt_D = (e_D-eprev_D)/(deltaT_D);
          
    // integral
    eintegral_A = eintegral_A + e_A*deltaT_A;
    eintegral_D = eintegral_D + e_D*deltaT_D;
          
    // control signal
    float u_A = kp_A*e_A + kd_A*dedt_A + ki_A*eintegral_A;
    float u_D = kp_D*e_D + kd_D*dedt_D + ki_D*eintegral_D;
         
    //motor direction
    int dir_A = 1;
    if(u_A<0){
      dir_A = -1;
    }
          
     int dir_D = 1;
     if(u_D<0){
      dir_D = -1;
     }
     

     setMotor(dir_A,255,PWM_A,IN1_A,IN2_A);
     setMotor(dir_A,255,PWM_B,IN1_B,IN2_B);
     setMotor(dir_D,255,PWM_D,IN1_D,IN2_D);
     setMotor(dir_D,255,PWM_C,IN1_C,IN2_C);   
     // store previous error
     eprev_A = e_A;
     eprev_D = e_D;

     Serial.print(" ");
     Serial.print(target_a);
     Serial.print(" ");
     Serial.print(target_b);
     Serial.print(" ");
     Serial.print(pos_A);
     Serial.print(" ");
     Serial.print(pos_D);
     Serial.println(); 

     if (fabs(e_A) < 10 or fabs(e_D) < 10 ){  
       break; 
     }


  }
  pos_A=0;
  pos_D=0;
  
  setMotor(0,0,PWM_A,IN1_A,IN2_A);
  setMotor(0,0,PWM_B,IN1_B,IN2_B);
  setMotor(0,0,PWM_D,IN1_D,IN2_D);
  setMotor(0,0,PWM_C,IN1_C,IN2_C); 
}

void Break (int target_a, int target_b){

  int e_A = pos_A-target_a;
  int e_D = pos_D-target_b;
  unsigned long currentTime = millis();
  
  
  while (fabs(e_A) >= 0 or fabs(e_D) >= 0 ){
    long currT_A = micros();
    float deltaT_A = ((float) (currT_A - prevT_A))/( 1.0e6 );
    prevT_A = currT_A;
      
    long currT_D = micros();
    float deltaT_D = ((float) (currT_D - prevT_D))/( 1.0e6 );
    prevT_D = currT_D;
          
    // error
    e_A = pos_A-target_a;
    e_D = pos_D-target_b;
          
    // derivative
    float dedt_A = (e_A-eprev_A)/(deltaT_A);
    float dedt_D = (e_D-eprev_D)/(deltaT_D);
          
    // integral
    eintegral_A = eintegral_A + e_A*deltaT_A;
    eintegral_D = eintegral_D + e_D*deltaT_D;
          
    // control signal
    float u_A = kp_A*e_A + kd_A*dedt_A + ki_A*eintegral_A;
    float u_D = kp_D*e_D + kd_D*dedt_D + ki_D*eintegral_D;
         
    //motor direction
    int dir_A = 1;
    if(u_A<0){
      dir_A = -1;
    }
          
     int dir_D = 1;
     if(u_D<0){
      dir_D = -1;
     }
     
     Serial.print(" Stopcount ");
     Serial.print(stopcount);
     Serial.print("Skip");
     Serial.print(" ");
     Serial.print(target_a);
     Serial.print(" ");
     Serial.print(target_b);
     Serial.print(" ");
     Serial.print(pos_A);
     Serial.print(" ");
     Serial.print(pos_D);
     Serial.println(); 
     
     if (fabs(e_A) < 20 and fabs(e_D) < 20 ){ 
       break; 
     }

     Serial.print("NOt BROKENN ");

     setMotor(dir_A,255,PWM_A,IN1_A,IN2_A);
     setMotor(dir_A,255,PWM_B,IN1_B,IN2_B);
     setMotor(dir_D,255,PWM_D,IN1_D,IN2_D);
     setMotor(dir_D,255,PWM_C,IN1_C,IN2_C);   
     // store previous error
     eprev_A = e_A;
     eprev_D = e_D;


     
     
          
  }
    
  setMotor(0,0,PWM_A,IN1_A,IN2_A);
  setMotor(0,0,PWM_B,IN1_B,IN2_B);
  setMotor(0,0,PWM_D,IN1_D,IN2_D);
  setMotor(0,0,PWM_C,IN1_C,IN2_C); 
}



void readEncoder_A(){
  int b_A = digitalRead(ENCB_A);
  if(b_A > 0){
    pos_A++;
  }
  else{
    pos_A--;
  }
}

void readEncoder_D(){
  int b_D = digitalRead(ENCA_D);
  if(b_D > 0){
    pos_D++;
  }
  else{
    pos_D--;
  }
}
