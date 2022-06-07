const int d = 5;
const int pwmPins[3] = {10,11,12};
const int enPins[3] = {14,15,16};

const int pwm[3] = {2,3,4}; //GPIO{4,3,2}
const int en[3] = {6,7,8};
const int pwmChannel[3] = {0,1,2}; //PWM channels (0-15)

/*Frequency: max 40MHz*/
const int PWMFreq = 20000; //Hz
/*Resolution: 1-16 bits*/
const int PWMResolution = 10; //1-16 bits (10 bits = 1024 bins)
/*Duty cycle dependent on resolution*/
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);
const int LIMIT_MAX_DUTY_CYCLE = (int) (0.49 * MAX_DUTY_CYCLE);

int pwmA[MAX_DUTY_CYCLE];

int pwmStatus[3];
int enStatus[3];

int duty = (int) (0.1*MAX_DUTY_CYCLE);

boolean forward = true;
boolean pwmSwitchState;
boolean enSwitchState;

String msg1;
String msg2;

void setup() {
  // put your setup code here, to run once:
  for(int i=0; i<MAX_DUTY_CYCLE; i++){
    float angleA = (2 * 3.14 * i) / MAX_DUTY_CYCLE;

    float sinAngle = 0.5 * (sin(angleA) + 1);
    
    pwmA[i] = (int) (LIMIT_MAX_DUTY_CYCLE * sinAngle);
    
  }

  for(int i=0; i<3; i++){
    pinMode(pwm[i], OUTPUT);
    pinMode(en[i], OUTPUT);
    pinMode(pwmPins[i], INPUT_PULLUP);
    pinMode(enPins[i], INPUT_PULLUP);
    ledcSetup(pwmChannel[i], PWMFreq, PWMResolution);
    ledcAttachPin(pwm[i], pwmChannel[i]);
  }

  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  msg1 = "";
  msg2 = "";
  for(int i=0; i<3; i++){
    pwmSwitchState = digitalRead(pwmPins[i]);
    enSwitchState = digitalRead(enPins[i]);

    msg1 += pwmSwitchState;
    msg2 += enSwitchState;

    if(pwmSwitchState != pwmStatus[i]){
      if(pwmSwitchState == 0){
      //analogWrite(pwm[i], 0);
      ledcWrite(pwmChannel[i], 0);
      }
      else{
      //analogWrite(pwm[i], 0.1*MAX_DUTY_CYCLE);
      ledcWrite(pwmChannel[i], duty);
      }
      pwmStatus[i] = pwmSwitchState;
    }

    if(enSwitchState != enStatus[i]){
      if(enSwitchState == 0){
        digitalWrite(en[i], LOW);
      }
      else{
        digitalWrite(en[i], HIGH);
      }
      enStatus[i] = enSwitchState;
    }
    
  }

  Serial.println(msg1+msg2);

}
