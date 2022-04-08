const int pwma = 2;  /* GPIO2 */
const int pwmb = 4; 
const int pwmc = 6; 
const int ena = 8;
const int enb = 10;
const int enc = 12;

int dutyCycle;
/*Frequency: max 40MHz*/
const int PWMFreq = 5000; //5 KHz
/*Resolution: 1-16 bits*/
const int PWMResolution = 10;
/*Duty cycle dependent on resolution*/
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);
/*PWM channels: 0-15*/
const int pwmaChannel = 0;
const int pwmbChannel = 1;
const int pwmcChannel = 2;
void setup()
{
  /*Setup enable pins*/
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(enc, OUTPUT);
  
  /*Initialize channels*/  
  ledcSetup(pwmaChannel, PWMFreq, PWMResolution);
  ledcSetup(pwmbChannel, PWMFreq, PWMResolution);
  ledcSetup(pwmcChannel, PWMFreq, PWMResolution);
  /* Attach the PWM Channels to the GPIO Pin */
  ledcAttachPin(pwma, pwmaChannel);
  ledcAttachPin(pwmb, pwmbChannel);
  ledcAttachPin(pwmc, pwmcChannel);
}
void loop()
{
  /*Phase A*/
  digitalWrite(ena, HIGH);
  for(dutyCycle = 0; dutyCycle <= MAX_DUTY_CYCLE; dutyCycle++)
  {
    ledcWrite(pwmaChannel, dutyCycle);
    delay(3);
    //delayMicroseconds(100);
  }
  for(dutyCycle = MAX_DUTY_CYCLE; dutyCycle >= 0; dutyCycle--)
  {
    ledcWrite(pwmaChannel, dutyCycle);
    delay(3);
    //delayMicroseconds(100);
  }
  digitalWrite(ena, LOW);

  /*Phase B*/
  digitalWrite(enb, HIGH);
  for(dutyCycle = 0; dutyCycle <= MAX_DUTY_CYCLE; dutyCycle++)
  {
    ledcWrite(pwmbChannel, dutyCycle);
    delay(3);
    //delayMicroseconds(100);
  }
  for(dutyCycle = MAX_DUTY_CYCLE; dutyCycle >= 0; dutyCycle--)
  {
    ledcWrite(pwmbChannel, dutyCycle);
    delay(3);
    //delayMicroseconds(100);
  }
  digitalWrite(enb, LOW);

  /*Phase C*/
  digitalWrite(enc, HIGH);
  for(dutyCycle = 0; dutyCycle <= MAX_DUTY_CYCLE; dutyCycle++)
  {
    ledcWrite(pwmcChannel, dutyCycle);
    delay(3);
    //delayMicroseconds(100);
  }
  for(dutyCycle = MAX_DUTY_CYCLE; dutyCycle >= 0; dutyCycle--)
  {
    ledcWrite(pwmcChannel, dutyCycle);
    delay(3);
    //delayMicroseconds(100);
  }
  digitalWrite(enc, LOW);
}
