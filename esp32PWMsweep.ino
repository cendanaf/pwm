const int pwm[3] = {4,3,2}; //GPIO{4,3,2}
const int en[3] = {8,7,6};

int dutyCycle;
/*Frequency: max 40MHz*/
const int PWMFreq = 5000; //5 KHz
/*Resolution: 1-16 bits*/
const int PWMResolution = 10;
/*Duty cycle dependent on resolution*/
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);
/*PWM channels: 0-15*/
const int pwmChannel[3] = {0,1,2};
void setup()
{
  /*Setup enable pins*/
  for(int i=0; i<3; i++){
    pinMode(en[i], OUTPUT);
  }
  
  /*Initialize channels*/
  for(int i=0; i<3; i++){
    ledcSetup(pwmChannel[i], PWMFreq, PWMResolution);
  }
  /* Attach the PWM Channels to the GPIO Pin */
  for(int i=0; i<3; i++){
    ledcAttachPin(pwm[i], pwmChannel[i]);
  }
}
void loop()
{
  for(int i=0; i<3; i++){
    digitalWrite(en[i], HIGH);
    for(dutyCycle = 0; dutyCycle <= MAX_DUTY_CYCLE; dutyCycle++)
    {
      ledcWrite(pwmChannel[i], dutyCycle);
      delay(1);
    }
    for(dutyCycle = MAX_DUTY_CYCLE; dutyCycle >= 0; dutyCycle--)
    {
      ledcWrite(pwmChannel[i], dutyCycle);
      delay(1);
    }
    digitalWrite(en[i], LOW);
  }
}
