
const int buttonPin = 10;
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

unsigned long debounceTime = 50;
unsigned long lastDebounceTime = 0;

void setup() {
  // put your setup code here, to run once:
  for(int i=0; i<MAX_DUTY_CYCLE; i++){
    float angle = (2 * 3.14 * i) / MAX_DUTY_CYCLE;

    float sinAngle = 0.5 * (sin(angleA) + 1);
    
    pwmA[i] = (int) (LIMIT_MAX_DUTY_CYCLE * sinAngle);
    
  }

  for(int i=0; i<3; i++){
    pinMode(pwm[i], OUTPUT);
    pinMode(en[i], OUTPUT);
    digitalWrite(en[i], HIGH);
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
      analogWrite(pwm[p], 0); // zero the current pin
      p++;
      if(p > 2){
        p = 0;
      }
      
      
      
      Serial.println(String(p));
    }
    else if(state == LOW && buttonState == HIGH){
      //button is released
    }
  }

  else if(state == LOW && buttonState == HIGH){
    // button is released
  }

  state = buttonState;
  

  analogWrite(pwm[p], pwmA[phase]);  // run next pin
  if(forward) increment = 1;
  else increment = -1;

  phase = phase + increment;
  if(phase > MAX_DUTY_CYCLE) phase = 0;
  if(phase < 0) phase = MAX_DUTY_CYCLE;
  delay(d);

  
  

}
