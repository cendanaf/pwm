
const int buttonPin = 0;
const int pwm[3] = {2,3,4}; //GPIO{4,3,2}
const int en[3] = {6,7,8};
const int pwmChannel[3] = {0,1,2}; //PWM channels (0-15)

/*Frequency: max 40MHz*/
const int PWMFreq = 20000; //kHz
/*Resolution: 1-16 bits*/
const int PWMResolution = 10; //1-16 bits (10 bits = 1024 bins)
/*Duty cycle dependent on resolution*/
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);
const int LIMIT_MAX_DUTY_CYCLE = (int) (0.49 * MAX_DUTY_CYCLE);

int pwmA[MAX_DUTY_CYCLE];

const int d = 5;
boolean forward = true;
boolean state = false;
boolean steadyState = false;
boolean transientState = false;
boolean buttonState;

int phase = 0;
int increment;
int p = 0;
int step = 0;
int dutyCycle;

unsigned long debounceTime = 50;
unsigned long lastDebounceTime = 0;

void setup() {
  // put your setup code here, to run once:
  for(int i=0; i<MAX_DUTY_CYCLE; i++){
    float angleA = (2 * 3.14 * i) / MAX_DUTY_CYCLE;

    float sinAngle = 0.5 * (sin(angleA) + 1);
    
    pwmA[i] = (int) (LIMIT_MAX_DUTY_CYCLE * sinAngle);
    
  }

  for(int i=0; i<3; i++){
    //pinMode(pwm[i], OUTPUT);
    pinMode(en[i], OUTPUT);
    //digitalWrite(en[i], HIGH);
    ledcSetup(pwmChannel[i], PWMFreq, PWMResolution);
    ledcAttachPin(pwm[i], pwmChannel[i]);
  }

  pinMode(buttonPin, INPUT_PULLUP);

  Serial.begin(9600);

}

void loop() {

  buttonState = digitalRead(buttonPin);

  // First check if in transient state 
  if(state != transientState){
    lastDebounceTime = millis();
    transientState = state;
  }

  // if enough time has passed then we're at steady state
  if((millis() - lastDebounceTime) > debounceTime){
    if(state == HIGH && buttonState == LOW){
      //button is pressed
      Serial.println(String(step));
      // Zero pwm and en pins
      for(int i=0; i<3; i++){
        digitalWrite(en[i], LOW);
        //analogWrite(pwm[i], 0);
        ledcWrite(pwmChannel[0], 0);
      }
      // Go through steps
      
      if(step == 1){
        digitalWrite(en[0], HIGH);
        //analogWrite(pwm[1], 0.1*MAX_DUTY_CYCLE);
        ledcWrite(pwmChannel[0], 0.11*MAX_DUTY_CYCLE);
      }
      else if(step == 2){
        digitalWrite(en[1], HIGH);
        //analogWrite(pwm[2], 0.1*MAX_DUTY_CYCLE);
        ledcWrite(pwmChannel[1], 0.11*MAX_DUTY_CYCLE);
      }
      else if(step == 3){
        digitalWrite(en[2], HIGH);
        //analogWrite(pwm[2], 0.1*MAX_DUTY_CYCLE);
        ledcWrite(pwmChannel[2], 0.11*MAX_DUTY_CYCLE);
      }
      else if(step == 4){
        digitalWrite(en[0], HIGH);
        //analogWrite(pwm[2], 0.1*MAX_DUTY_CYCLE);
        ledcWrite(pwmChannel[2], 0.11*MAX_DUTY_CYCLE);
      }
      else if(step == 5){
        digitalWrite(en[1], HIGH);
        //analogWrite(pwm[2], 0.1*MAX_DUTY_CYCLE);
        ledcWrite(pwmChannel[2], 0.11*MAX_DUTY_CYCLE);
      }
      else{
        digitalWrite(en[1], HIGH);
        //analogWrite(pwm[1], 0.1*MAX_DUTY_CYCLE);
        ledcWrite(pwmChannel[0], 0.11*MAX_DUTY_CYCLE);
      }

      step++;
      if(step > 5) step = 0;
    }
    else if(state == LOW && buttonState == HIGH){
      //button is released
    }
  }
  
  state = buttonState;
  
  

}
