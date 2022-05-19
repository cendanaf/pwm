const int pwm[3] = {2,3,4}; //GPIO{4,3,2}
const int en[3] = {6,7,8};
const int pwmChannel[3] = {0,1,2}; //PWM channels (0-15)

int dutyCycle;
/*Frequency: max 40MHz*/
const int PWMFreq = 5000; //5 KHz
/*Resolution: 1-16 bits*/
const int PWMResolution = 10; //1-16 bits (10 bits = 0-1023)
/*Duty cycle dependent on resolution*/
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);

const int d = 500;

void setup()
{
  /*Setup enable pins*/
  for(int i=0; i<3; i++)
  {
    pinMode(en[i], OUTPUT);
    digitalWrite(en[i], LOW);
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

  Serial.begin(9600);
}

void loop() {
  Serial.println("Wave 1");
  digitalWrite(en[0], HIGH);
  ledcWrite(pwmChannel[0], 0.1*MAX_DUTY_CYCLE);
  delay(d);

  Serial.println("Wave 2");
  ledcWrite(pwmChannel[0], 0);
  digitalWrite(en[1], HIGH);
  ledcWrite(pwmChannel[1], 0.1*MAX_DUTY_CYCLE);
  delay(d);

  Serial.println("Wave 3");
  ledcWrite(pwmChannel[1], 0);
  digitalWrite(en[0], LOW);
  digitalWrite(en[2], HIGH);
  ledcWrite(pwmChannel[2], 0.1*MAX_DUTY_CYCLE);
  delay(d);

  Serial.println("------");
  digitalWrite(en[1], LOW);
}
