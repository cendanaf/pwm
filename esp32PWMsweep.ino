const int pwm[3] = {4,3,2}; //GPIO{4,3,2}
const int en[3] = {8,7,6};
const int pwmChannel[3] = {0,1,2}; //PWM channels (0-15)

int dutyCycle;
/*Frequency: max 40MHz*/
const int PWMFreq = 10000; //Hz
/*Resolution: 1-16 bits*/
const int PWMResolution = 10;
/*Duty cycle dependent on resolution*/
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);

const int LIMIT_MAX_DUTY_CYCLE = (int) (1 * MAX_DUTY_CYCLE);

int pwmA[MAX_DUTY_CYCLE];
int pwmB[MAX_DUTY_CYCLE];
int pwmC[MAX_DUTY_CYCLE];

float phaseB = 3.14 * (30./180.);
float phaseC = 3.14 * (60./180.);

const int d = 5;
boolean forward = true;
int phase = 0;
int increment;

void setup()
{
  /*Setup enable pins*/
  for(int i=0; i<3; i++){
    pinMode(en[i], OUTPUT);
    digitalWrite(en[i], HIGH);
  }
  
  /*Initialize channels*/
  for(int i=0; i<3; i++){
    ledcSetup(pwmChannel[i], PWMFreq, PWMResolution);
  }
  /* Attach the PWM Channels to the GPIO Pin */
  for(int i=0; i<3; i++){
    ledcAttachPin(pwm[i], pwmChannel[i]);
  }

  /*Generate table of values for sine function at different phases*/
  for(int i=0; i<MAX_DUTY_CYCLE; i++){
    float angleA = (2 * 3.14 * i) / MAX_DUTY_CYCLE;
    float angleB = angleA + phaseB;
    float angleC = angleA + phaseC;

    float sinA = 0.5 * (sin(angleA) + 1);
    float sinB = 0.5 * (sin(angleB) + 1);
    float sinC = 0.5 * (sin(angleC) + 1);

    pwmA[i] = (int) (LIMIT_MAX_DUTY_CYCLE * sinA);
    pwmB[i] = (int) (LIMIT_MAX_DUTY_CYCLE * sinB);
    pwmC[i] = (int) (LIMIT_MAX_DUTY_CYCLE * sinC);
    
  }

  Serial.begin(9600);
}

void loop()
{
  ledcWrite(pwmChannel[0], pwmA[phase]);
  ledcWrite(pwmChannel[1], pwmB[phase]);
  ledcWrite(pwmChannel[2], pwmC[phase]);

  if(forward) increment = 1;
  else increment = -1;

  phase = phase + increment;
  if(phase > MAX_DUTY_CYCLE) phase = 0;
  if(phase < 0) phase = MAX_DUTY_CYCLE;

  Serial.println(String(pwmA[phase]) + ", " + String(pwmB[phase]) + ", " + String(pwmC[phase]));

  delay(d);
  
}
