const int pwm[3] = {4,3,2}; //GPIO{4,3,2}
const int en[3] = {8,7,6};
const int pwmChannel[3] = {0,1,2}; //PWM channels (0-15)

int dutyCycle;
/*Frequency: max 40MHz*/
const int PWMFreq = 5000; //5 KHz
/*Resolution: 1-16 bits*/
const int PWMResolution = 10; //1-16 bits (10 bits = 0-1023)
/*Duty cycle dependent on resolution*/
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);



void setup()
{
  /*Setup enable pins*/
  for(int i=0; i<3; i++)
  {
    pinMode(en[i], OUTPUT);
  }
  
  /*Initialize channels*/
  for(int i=0; i<3; i++)
  {
    ledcSetup(pwmChannel[i], PWMFreq, PWMResolution);
  }
  /* Route PWM channel signals to GPIO pins */
  for(int i=0; i<3; i++)
  {
    ledcAttachPin(pwm[i], pwmChannel[i]);
  }
}

void loop()
{
  for(int i=0; i<3; i++)
  {
    runPhase(en[i], pwmChannel[i], 50);
    delay(1);
    runPhase(en[i], pwmChannel[i], 0);
  }
}


void runPhase(int en, int pwmChannel, int dutyCycle){
  digitalWrite(en, HIGH);
  ledcWrite(pwmChannel, dutyCycle);
  delay(20);
  digitalWrite(en, LOW);
}
