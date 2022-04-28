
const int pwm[3] = {4,3,2}; //GPIO{4,3,2}
const int en[3] = {8,7,6};
const int pwmChannel[3] = {0,1,2}; //PWM channels (0-15)

/*Frequency: max 40MHz*/
const int PWMFreq = 5000; //5 KHz
/*Resolution: 1-16 bits*/
const int PWMResolution = 10; //1-16 bits (10 bits = 1024 bins)
/*Duty cycle dependent on resolution*/
const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);

int pwmA[MAX_DUTY_CYCLE];
int pwmB[MAX_DUTY_CYCLE];
int pwmC[MAX_DUTY_CYCLE];

float phaseB = 3.14 * (30./180.);
float phaseC = 3.14 * (60./180.);

const int d = 10;
boolean forward = true;
int phase = 0;
int increment;

void setup() {
  // put your setup code here, to run once:
  for(int i=0; i<MAX_DUTY_CYCLE; i++){
    float angleA = (2 * 3.14 * i) / MAX_DUTY_CYCLE;
    float angleB = angleA + phaseB;
    float angleC = angleA + phaseC;

    pwmA[i] = (int)(((MAX_DUTY_CYCLE/2) * sin(angleA)) + (MAX_DUTY_CYCLE/2));
    pwmB[i] = (int)(((MAX_DUTY_CYCLE/2) * sin(angleB)) + (MAX_DUTY_CYCLE/2));
    pwmC[i] = (int)(((MAX_DUTY_CYCLE/2) * sin(angleC)) + (MAX_DUTY_CYCLE/2));

    
  }

  for(int i=0; i<3; i++){
    pinMode(pwm[i], OUTPUT);
    pinMode(en[i], OUTPUT);
    digitalWrite(en[i], HIGH);
  }

  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(pwm[0], pwmA[phase]);
  analogWrite(pwm[1], pwmB[phase]);
  analogWrite(pwm[2], pwmC[phase]);

  if(forward) increment = 1;
  else increment = -1;

  phase = phase + increment;
  if(phase > MAX_DUTY_CYCLE) phase = 0;
  if(phase < 0) phase = MAX_DUTY_CYCLE;

  delay(d); 

}
