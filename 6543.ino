const int buttonPin = 0;
const int pwm[3] = {2,3,4}; //GPIO{4,3,2}
const int en[3] = {6,7,8};
const int pwmChannel[3] = {0,1,2}; //PWM channels (0-15)

const int PWMResolution = 10; //1-16 bits (10 bits = 1024 bins)
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);

boolean steadyState = false;
boolean transientState = false;
boolean buttonState;

int phase = 0;
int increment;
int p = 0;
int step = 0;
int dutyCycle = (int) (0.1*MAX_DUTY_CYCLE);

unsigned long debounceTime = 50;
unsigned long lastDebounceTime = 0;

/*Frequency: max 40MHz*/
const int PWMFreq = 20000; //5 KHz

const int d = 500;

boolean state = false;

void setup()
{
  /*Setup enable pins*/
  for(int i=0; i<3; i++)
  {
    pinMode(en[i], OUTPUT);
    digitalWrite(en[i], LOW);
    ledcSetup(pwmChannel[i], PWMFreq, PWMResolution); ///*Initialize channel*/
    ledcAttachPin(pwm[i], pwmChannel[i]); ///* Route PWM channel signal to GPIO pins */
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
      Serial.println("Step " + String(step));
      // Go through steps
      if(step == 0){
        digitalWrite(en[1], HIGH);
        digitalWrite(en[0], HIGH);
        ledcWrite(pwmChannel[0], dutyCycle);
        //step++;
      }
      else if(step == 1){
        digitalWrite(en[2], LOW);
        ledcWrite(pwmChannel[2], 0);

        digitalWrite(en[0], HIGH);
        ledcWrite(pwmChannel[0], dutyCycle);
      }
      else if(step == 2){
        digitalWrite(en[1], LOW);

        digitalWrite(en[2], HIGH);
      }
      else if(step == 3){
        digitalWrite(en[0], LOW);
        ledcWrite(pwmChannel[0], 0);

        digitalWrite(en[1], HIGH);
        ledcWrite(pwmChannel[1], dutyCycle);
      }
      else if(step == 4){
        digitalWrite(en[2], LOW);

        digitalWrite(en[0], HIGH);
      }
      else if(step == 5){
        digitalWrite(en[1], LOW);
        ledcWrite(pwmChannel[1], 0);

        digitalWrite(en[2], HIGH);
        ledcWrite(pwmChannel[2], dutyCycle);
      }
      else{
        digitalWrite(en[0], LOW);
        digitalWrite(en[1], HIGH);
      }

      step++;
      if(step > 6) step = 1;
    }
    else if(state == LOW && buttonState == HIGH){
      //button is released
    }
  }
  
  state = buttonState;

 
}
