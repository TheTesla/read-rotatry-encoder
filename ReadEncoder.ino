/* Size of the receive buffer and transmit buffer. */
#define BUFFER_SIZE     (100)
/* Reference voltage for ADC,in mv. */
#define VOLT_REF        (3300)
/* The maximal digital value */
#define MAX_DIGITAL     (4095)

int led = 13;

class RotaryEncoder {
  public:
    RotaryEncoder(void);
    void setIpin(int pin);
    void setQpin(int pin);
    void sample(void);
    int getValue(void);
    void reset(int value);

  private:
    int cnt;
    int state;
    int iPin;
    int qPin;

};

RotaryEncoder::RotaryEncoder(void) {

}

void RotaryEncoder::setIpin(int pin) {
  this->iPin = pin;
}

void RotaryEncoder::setQpin(int pin) {
  this->qPin = pin;
}

int RotaryEncoder::getValue(void) {
  return this->cnt;
}

void RotaryEncoder::reset(int value=0) {
  this->cnt = value;
}

void RotaryEncoder::sample(void) {
  const int lut[4] = {0, 1, 3, 2};
  int i = digitalRead(this->iPin);
  int q = digitalRead(this->qPin);
  int cur = lut[(i + 2 * q)%4];
  int diff = (cur - this->state + 4)%4;
  this->state = cur;
  if (1 == diff) {
    this->cnt++;
  } else if (3 == diff) {
    this->cnt--;
  }
}

RotaryEncoder enc1;

void setup() {
  enc1.setIpin(4);
  enc1.setQpin(5);
  enc1.reset();
  Serial.begin(115200);
  //pinMode(led, OUTPUT);
  for(int i=4;i<14;i++){
    pinMode(i, INPUT_PULLUP);
  }
  pinMode(3, OUTPUT);
  pinMode(2, OUTPUT);
  digitalWrite(3, HIGH);
  digitalWrite(2, HIGH);

}



void loop() {
  //int i_enc = digitalRead(51);
  //int i_enc = PIO_PD1;
  enc1.sample();
  Serial.print("Digital:");
  for(int i=4;i<14;i++){
    Serial.print(" ");
    Serial.print(digitalRead(i));
  }
  Serial.print(" # ");
  Serial.print(enc1.getValue());
  Serial.println("");


  //digitalWrite(led, HIGH);
  delay(10);
  //digitalWrite(led, LOW);
  delay(10);
}
