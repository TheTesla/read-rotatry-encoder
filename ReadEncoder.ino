/* Size of the receive buffer and transmit buffer. */
#define BUFFER_SIZE     (100)
/* Reference voltage for ADC,in mv. */
#define VOLT_REF        (3300)
/* The maximal digital value */
#define MAX_DIGITAL     (4095)

#include <avr/io.h>
#include <avr/interrupt.h>


int led = 13;

class RotaryEncoder {
  public:
    RotaryEncoder(void);
    //void setIpin(int pin);
    //void setQpin(int pin);
    void setIcb(int (*iSrcFn) (void) );
    void setQcb(int (*qSrcFn) (void) );
    void sample(void);
    int getValue(void);
    void reset(int value);

  private:
    int cnt;
    int state;
    //int iPin;
    //int qPin;
    int (*iSrcFn) (void);
    int (*qSrcFn) (void);

};

RotaryEncoder::RotaryEncoder(void) {

}

void RotaryEncoder::setIcb(int (*iSrcFn) (void) ) {
  this->iSrcFn = iSrcFn;
}

void RotaryEncoder::setQcb(int (*qSrcFn) (void) ) {
  this->qSrcFn = qSrcFn;
}
/*
void RotaryEncoder::setIpin(int pin) {
  this->iSrcFn = pin;
}

void RotaryEncoder::setQpin(int pin) {
  this->qPin = pin;
}
*/
int RotaryEncoder::getValue(void) {
  return this->cnt;
}

void RotaryEncoder::reset(int value=0) {
  this->cnt = value;
}

void RotaryEncoder::sample(void) {
  const int lut[4] = {0, 1, 3, 2};
  //int i = digitalRead(this->iPin);
  //int q = digitalRead(this->qPin);
  int i = this->iSrcFn();
  int q = this->qSrcFn();
  int cur = lut[(i + 2 * q)%4];
  int diff = (cur - this->state + 4)%4;
  this->state = cur;
  if (1 == diff) {
    this->cnt++;
  } else if (3 == diff) {
    this->cnt--;
  }
}

void setup_timer(void){
  // Timer1 konfigurieren
  cli(); // Alle Interrupts deaktivieren, um Konfiguration sicher zu machen

  TCCR1A = 0;            // Timer1 Control Register A auf 0 setzen
  TCCR1B = 0;            // Timer1 Control Register B auf 0 setzen
  TCNT1  = 0;            // Timerzähler auf 0 setzen

  // Vergleichswert berechnen für 1ms Interrupts:
  // Bei einem 16MHz Arduino-Takt und einem Prescaler von 64
  // ist 1ms = 16,000,000 / 64 / 1000 = 250 Zählschritte.
  OCR1A = 250;           // Vergleichswert setzen für 1ms

  TCCR1B |= (1 << WGM12); // CTC Modus (Clear Timer on Compare Match)
  TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler auf 64 setzen

  TIMSK1 |= (1 << OCIE1A); // Timer Compare Interrupt aktivieren

  sei(); // Interrupts global aktivieren
}


RotaryEncoder rotenc[5];

void setup_all_encoders(void) {
  for (int i=0; i<5; i++) {
    rotenc[i].setIcb([](int i) {return digitalRead(2*i+4);});
    rotenc[i].setQcb([](int i) {return digitalRead(2*i+5);});
    rotenc[i].reset();
  }

}

void sample_all_encoders(void) {
  for (int i=0; i<5; i++) {
    rotenc[i].sample();
  }
}

ISR(TIMER1_COMPA_vect) {
  // Diese Funktion wird alle 1 ms aufgerufen
  // Hier kann Code eingefügt werden, der alle 1ms ausgeführt werden soll.
  sample_all_encoders();
}


void setup() {
  setup_all_encoders();
  setup_timer();
//  enc1.setIpin(4);
//  enc1.setQpin(5);
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
  //enc1.sample();
  Serial.print("Digital:");
  for(int i=4;i<14;i++){
    Serial.print(" ");
    Serial.print(digitalRead(i));
  }
  Serial.print(" #");
  for (int i=0; i<5; i++) {
    Serial.print(" ");
    Serial.print(rotenc[i].getValue());
  }
  Serial.println("");


  //digitalWrite(led, HIGH);
  delay(10);
  //digitalWrite(led, LOW);
  delay(10);
}
