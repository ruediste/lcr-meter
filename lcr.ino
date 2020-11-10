#include <LiquidCrystal.h>

const int rs = 12, en = 13, d4 = 11, d5 = 10, d6 = 9, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

const int PIN_R0 = 2;
const int PIN_R1 = 3;
const int PIN_R2 = 4;
const int PIN_Rmask = (1 << PIN_R0) | (1 << PIN_R1) | (1 << PIN_R2);
const int PIN_L = 4;

const uint8_t PIN_analog_rc = A0;
const uint8_t PIN_analog_l = A1;

const float valueR0 = 390.;
const float valueR1 = 10. * 1000;
const float valueR2 = 100. * 1000;

const uint16_t LIMIT_LOW_VALUE = 5;
const uint16_t LIMIT_HIGH_VALUE = 1010;
const uint16_t MAX_RC_VALUE = 1014;
const unsigned long displayDelay = 2000;

void setup()
{
  DIDR1 = (1 << AIN0D); // disable digital input on AIN0

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  DDRC |= (1 << 2);
}

void printWithSiPrefix(float value)
{
  if (value < 1e-9)
  {
    lcd.print(value * 1e12);
    lcd.print(" p");
  }
  else if (value < 1e-6)
  {
    lcd.print(value * 1e9);
    lcd.print(" n");
  }
  else if (value < 1e-3)
  {
    lcd.print(value * 1e6);
    lcd.print(" u");
  }
  else if (value < 1)
  {
    lcd.print(value * 1e3);
    lcd.print(" m");
  }
  else if (value < 1e3)
  {
    lcd.print(value);
    lcd.print(" ");
  }
  else if (value < 1e6)
  {
    lcd.print(value / 1e3);
    lcd.print(" k");
  }
  else if (value < 1e9)
  {
    lcd.print(value / 1e6);
    lcd.print(" M");
  }
  else
  {
    lcd.print(value / 1e9);
    lcd.print(" G");
  }
}

uint16_t timerBase;
boolean inputCaptured;
uint32_t capturedValue;

void enableAdc()
{
  ADCSRA |= (1 << ADEN); // enable ADC
}

boolean startCMeasurement(uint8_t pin)
{
  // initialize
  PORTD &= ~PIN_Rmask; // set r pins to 0
  DDRD &= ~PIN_Rmask;  // make r pins inputs

  // discharge
  DDRD |= (1 << PIN_R0); // set r0 output
  enableAdc();
  {
    auto start = millis();
    while (true)
    {
      auto val = analogRead(PIN_analog_rc);
      if (val < LIMIT_LOW_VALUE)
        break;
      if (millis() - start > 1000)
      {
        return false;
      }
    }
  }

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

  DDRD = (DDRD & ~PIN_Rmask) | (1 << pin); // set pin output
  PORTD |= (1 << pin);                     // set output to high
  interrupts();
  return true;
}

uint16_t measureRValue()
{
  // let value settle
  enableAdc();

  while (true)
  {
    auto val = analogRead(PIN_analog_rc);
    delay(50);
    auto valNew = analogRead(PIN_analog_rc);
    if (abs(val - valNew) < 5)
      return valNew;
  }
}

void calculateAndPrintR(uint16_t value, float valueR)
{
  // valueR/(MAX_RC_VALUE-value) = R / value
  float divisor = (float)MAX_RC_VALUE - value;
  if (divisor < 1)
  {
    lcd.clear();
    lcd.print("R: out of range");
    delay(displayDelay);
    return;
  }
  float R = valueR * value / divisor;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("V: ");
  lcd.print(value);
  lcd.print(" ");
  lcd.print((uint32_t)valueR);

  lcd.setCursor(0, 1);
  lcd.print("R: ");
  printWithSiPrefix(R);
  lcd.print(" Ohm");

  delay(displayDelay);
}

void calculateAndPrintC(float time, float valueR)
{
  float tau = time / 0.7;
  float c = tau / valueR;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T: ");
  printWithSiPrefix(time);
  lcd.print("s");

  lcd.setCursor(0, 1);
  lcd.print("C: ");
  printWithSiPrefix(c);
  lcd.print("F");
  delay(displayDelay);
}

/* CR auto detection

        | low R  | middle R | high R
------------------------------------
low R   | middle | low      | low
middle R| high   | middle   | low
high R  | high   | high     | middle
short   | low    | low      | low
discon  | high   | high     | high
small C | fast   | fast     | 10ms
avg C   | fast   | 10 ms    | slow
large C | 10ms   | slow     | slow
huge C  | low    | low      | low

Algorithm: Start with low R, proceed with middle/higher R

1. Measure rise time (with next R)
2. let value settle and measure
-> timeout
  -> low value: short or too large C 
  -> middle value: => calculate R
-> high value:
   -> <= 1ms: large R or small C, go to 1
   -> > 1ms: => calculate C
-> middle value:
   -> <= 1ms: => calculate R
   -> >1ms: should not happen => error

*/

boolean measureC(uint8_t pin, float valueR)
{
  if (!startCMeasurement(pin))
  {
    // unable to discharge, there seems to be a resistor
    auto value = measureRValue();
    calculateAndPrintR(value, valueR);
    return true;
  }

  auto startTime = millis();
  while (true)
  {
    if (inputCaptured)
    {
      float time = capturedValue * (1. / 2) * 1.e-6;
      auto value = measureRValue();
      if (value > LIMIT_HIGH_VALUE)
      {
        if (time < 10e-6)
          return false;
        calculateAndPrintC(time, valueR);
        return true;
      }

      if (time < 10e-6)
      {
        calculateAndPrintR(value, valueR);
        return true;
      }

      lcd.clear();
      lcd.print(F("RC: parallel crct?"));
      delay(displayDelay);
      return true;
    }
    if ((millis() - startTime) > 1000)
    {
      // timeout
      auto value = measureRValue();
      if (value < LIMIT_LOW_VALUE)
      {
        lcd.clear();
        lcd.print(F("RC: short crct!"));
        delay(displayDelay);
        return true;
      }

      calculateAndPrintR(value, valueR);
      return true;
    }
  }
}

boolean measureC()
{
  if (measureC(PIN_R0, valueR0))
    return true;
  if (measureC(PIN_R1, valueR1))
    return true;
  if (measureC(PIN_R2, valueR2))
    return true;
  return false;
}

void loop()
{
  if (measureC())
    return;
  lcd.clear();
  lcd.print("not connected");
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