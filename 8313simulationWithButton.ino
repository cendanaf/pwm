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
int dutyCycle = 0.11*MAX_DUTY_CYCLE;

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
      Serial.println("Wave " + String(step));
      // Go through steps
      if(step == 0){
        digitalWrite(en[1], HIGH);
        digitalWrite(en[2], HIGH);
        digitalWrite(en[0], HIGH);
        ledcWrite(pwmChannel[0], dutyCycle);
      }
      else if(step == 1){
        ledcWrite(pwmChannel[0], 0);
        digitalWrite(en[1], HIGH);
        ledcWrite(pwmChannel[1], dutyCycle);
      }
      else{
        ledcWrite(pwmChannel[1], 0);
        digitalWrite(en[0], LOW);
        digitalWrite(en[2], HIGH);
        ledcWrite(pwmChannel[2], dutyCycle);
      }

      step++;
      if(step > 2) step = 0;
    }
    else if(state == LOW && buttonState == HIGH){
      //button is released
    }
  }
  
  state = buttonState;

  /*
  Serial.println("Wave 1");
  digitalWrite(en[0], HIGH);
  ledcWrite(pwmChannel[0], 0.11*MAX_DUTY_CYCLE);
  delay(d);

  Serial.println("Wave 2");
  ledcWrite(pwmChannel[0], 0);
  digitalWrite(en[1], HIGH);
  ledcWrite(pwmChannel[1], 0.11*MAX_DUTY_CYCLE);
  delay(d);

  Serial.println("Wave 3");
  ledcWrite(pwmChannel[1], 0);
  digitalWrite(en[0], LOW);
  digitalWrite(en[2], HIGH);
  ledcWrite(pwmChannel[2], 0.11*MAX_DUTY_CYCLE);
  delay(d);

  Serial.println("------");
  digitalWrite(en[1], LOW);
  */
}
