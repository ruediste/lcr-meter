#include <LiquidCrystal.h>

const int rs = 12, en = 13, d4 = 11, d5 = 10, d6 = 9, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

const int PIN_R0 = 2;
const int PIN_R1 = 3;
const int PIN_R2 = 4;
const int PIN_Rmask = (1 << PIN_R0) | (1 << PIN_R1) | (1 << PIN_R2);
const int PIN_L = 4;

const float valueR0 = 390;

enum class Mode
{
  INITIAL,
  WAIT_C,
  FINAL
};
Mode mode;

void setup()
{
  mode = Mode::INITIAL;
  DIDR1 = (1 << AIN0D); // disable digital input on AIN0

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  DDRC |= (1 << 2);
}

uint16_t timerBase;
boolean inputCaptured;
uint32_t capturedValue;

void startC()
{
  lcd.setCursor(0, 0);
  lcd.print("Waiting for C");

  PORTD &= ~PIN_Rmask;   // set r pins to 0
  DDRD &= ~PIN_Rmask;    // make r pins inputs
  DDRD |= (1 << PIN_R0); // set r0 output

  delay(500); // discharge any C

  noInterrupts();
  // setup timer 1
  TCCR1A = 0;
  TCCR1B =
      // (0 << ICES1) |  // rising edge to capture
      (1 << CS11); // /8 prescaler
  TIMSK1 =
      (1 << ICIE1)    // capture interrupt enable
      | (1 << TOIE1); // overflow interrupt

  // setup Ananalog Comparator: plus: ain0, minus: ADC0, input capture
  ADCSRB |= (1 << ACME);  // enable multiplexer
  ADCSRA &= ~(1 << ADEN); // disable ADC
  ADMUX &= ~0b1111;
  ACSR = (1 << ACIC); // enable input capture in T1

  // reset timer
  inputCaptured = false;
  timerBase = 0;
  TIFR1 = (1 << ICF1) | (1 << TOV1);
  TCNT1 = 0;
  PORTD |= (1 << PIN_R0); // set output to high
  interrupts();
}

void handleC()
{
  lcd.clear();
  uint32_t timeUs = capturedValue / 2;
  float tau = timeUs / 0.7 / 1e6;
  float c = tau / valueR0;
  
  lcd.setCursor(0, 0);
  lcd.print("T: ");
  lcd.print(timeUs);

  lcd.setCursor(0, 1);
  lcd.print("C: ");
  if (c < 1e-9)
  {
    lcd.print(c * 1e12);
    lcd.print(" pF");
  }
  else if (c < 1e-6)
  {
    lcd.print(c * 1e9);
    lcd.print(" nF");
  }
  else if (c < 1e-3)
  {
    lcd.print(c * 1e6);
    lcd.print(" uF");
  }
  else 
  {
    lcd.print(c * 1e3);
    lcd.print(" mF");
  }
}

boolean toggle;

void loop()
{
  switch (mode)
  {
  case Mode::INITIAL:
    startC();
    mode = Mode::WAIT_C;
    break;
  case Mode::WAIT_C:
    if (inputCaptured)
    {
      handleC();
      mode = Mode::FINAL;
    }
    else
    {
      if (toggle)
      {
        PORTC |= (1 << 2);
        //PORTD |= (1 << PIN_R0);
      }
      else
      {
        PORTC &= ~(1 << 2);
        //PORTD &= ~(1 << PIN_R0);
      }
      toggle = !toggle;
    }
    break;
  }
}

ISR(TIMER1_OVF_vect)
{
  timerBase++;
}

ISR(TIMER1_CAPT_vect)
{
  TIMSK1 &= ~(1 << ICIE1); // capture interrupt disabled
  capturedValue = (((uint32_t)timerBase) << 16) | ICR1;
  // if (ICR1 < 0x8000 && ((TIFR1 & (1 << TOV1)) != 0))
  {
    // low capture value and overflow still set, timer overflow has not been handled yet
    // capturedValue += ((uint32_t)1 << 16);
  }
  inputCaptured = true;
}