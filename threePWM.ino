// pwm at about 490Hz

int pwma = 9;
int pwmb = 10;
int pwmc = 11;
int ena = 5;
int enb = 6;
int enc = 7;
void setup() {
  pinMode(pwma, OUTPUT);
  pinMode(pwmb, OUTPUT);
  pinMode(pwmc, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(enc, OUTPUT);
}
void loop() {

  //phase a
  digitalWrite(ena, HIGH);
  for(int i=0; i<255; i++){
    analogWrite(pwma, i);
    delay(5);
  }
  for(int i=255; i>0; i--){
    analogWrite(pwma, i);
    delay(5);
  }
  digitalWrite(ena, LOW);

  //phase b
  digitalWrite(enb, HIGH);
  for(int i=0; i<255; i++){
    analogWrite(pwmb, i);
    delay(5);
  }
  for(int i=255; i>0; i--){
    analogWrite(pwmb, i);
    delay(5);
  }
  digitalWrite(enb, LOW);

  //phase c
  digitalWrite(enc, HIGH);
  for(int i=0; i<255; i++){
    analogWrite(pwmc, i);
    delay(5);
  }
  for(int i=255; i>0; i--){
    analogWrite(pwmc, i);
    delay(5);
  }
  digitalWrite(enc, LOW);
}
