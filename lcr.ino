#include <LiquidCrystal.h>

const int rs = 12, en = 13, d4 = 11, d5 = 10, d6 = 9, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

const int PIN_L = 7;

const uint8_t PIN_analog_rc = A0;
const uint8_t PIN_analog_l = A1;


const uint16_t LIMIT_LOW_VALUE = 5;
const uint16_t LIMIT_HIGH_VALUE = 1010;
const uint16_t MAX_RC_VALUE = 1014;
const unsigned long displayDelay = 200;


struct CRPinInfo
{
  float valueR;
  uint8_t index;
  uint8_t pin;
  CRPinInfo(uint8_t index, uint8_t pin, float valueR)
  {
    this->index = index;
    this->pin = pin;
    this->valueR = valueR;
  }
};

CRPinInfo crPinInfos[] = {
    CRPinInfo(0, 2, 390.),
    CRPinInfo(1, 3, 10. * 1000),
    CRPinInfo(2, 4, 100. * 1000),
    CRPinInfo(3, 5, 750. * 1000)
    };

const int PIN_Rmask = (1 << 2) | (1 << 3) | (1 << 4)| (1 << 5);

const uint8_t crPinInfoSize = 4;

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
  DDRD |= (1 << crPinInfos[0].pin); // set r0 output
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
      (1 << CS10); // /1 prescaler
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

boolean calculateAndPrintR(uint16_t value, struct CRPinInfo &pinInfo)
{
  // valueR/(MAX_RC_VALUE-value) = R / value
  // R=valueR*value/(MAX_RC_VALUE-value)
  float divisor = (float)MAX_RC_VALUE - value;
  if (divisor < 1)
  {
    lcd.clear();
    lcd.print("R: out of range");
    delay(displayDelay);
    return true;
  }

  float ratio = value / divisor;

  // check if next pin would make more sense
  if (pinInfo.index < crPinInfoSize - 1 && ratio > 1)
  {
    float nextValue = MAX_RC_VALUE - divisor * crPinInfos[pinInfo.index + 1].valueR / pinInfo.valueR;
    float nextDivisor = MAX_RC_VALUE - nextValue;
    if (nextDivisor > 1)
    {
      float nextRatio;
      if (nextValue > nextDivisor)
        nextRatio = nextValue / nextDivisor;
      else
        nextRatio = nextDivisor / nextValue;
      if (nextRatio < ratio)
        return false;
    }
  }

  float R = pinInfo.valueR * ratio;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("V: ");
  lcd.print(value);

  lcd.setCursor(14, 0);
  lcd.print("R");
  lcd.print(pinInfo.index);

  lcd.setCursor(0, 1);
  lcd.print("R: ");
  printWithSiPrefix(R);
  lcd.print(" Ohm");

  delay(displayDelay);
  return true;
}

void calculateAndPrintC(float time, struct CRPinInfo &pinInfo)
{
  float tau = time / 0.7;
  float c = tau / pinInfo.valueR;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T: ");
  printWithSiPrefix(time);
  lcd.print("s");

  lcd.setCursor(14, 0);
  lcd.print("R");
  lcd.print(pinInfo.index);

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

boolean measureC(struct CRPinInfo &pinInfo)
{
  if (!startCMeasurement(pinInfo.pin))
  {
    // unable to discharge, there seems to be a resistor
    auto value = measureRValue();
    return calculateAndPrintR(value, pinInfo);
  }

  auto startTime = millis();
  while (true)
  {
    if (inputCaptured)
    {
      float time = capturedValue * (1. / 16) * 1.e-6;
      auto value = measureRValue();

      if (value > LIMIT_HIGH_VALUE)
      {
        // raise all the way to the high value, either a capacitor or a high R
        boolean wasFastRaise;
        if (pinInfo.index < crPinInfoSize - 1)
          wasFastRaise = time < 1e-3;
        else
          // for last R, accept slower raises as well
          wasFastRaise = time < 30e-6;

        if (wasFastRaise)
          // high R or low C, try larger resistor
          return false;
        calculateAndPrintC(time, pinInfo);
        return true;
      }

      // we got stuck in the middle
      if (time < 1e-3)
      {
        return calculateAndPrintR(value, pinInfo);
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

      return calculateAndPrintR(value, pinInfo);
    }
  }
}

boolean measureC()
{
  for (uint8_t i = 0; i < crPinInfoSize; i++)
  {
    if (measureC(crPinInfos[i]))
      return true;
  }

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