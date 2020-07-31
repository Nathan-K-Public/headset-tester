#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include "mariosongs.h"
#include "ringbuffer.h"

Adafruit_MCP4725 dac;

// Set this value to 9, 8, 7, 6 or 5 to adjust the resolution
#define DAC_RESOLUTION    (8)
#define ADC_DELAY 40
// Increase if ADC starts reading 0 vals due to sampling cap not filling

#define pinLED PC13
#define buzzerPin PB0
#define maxLimit 100
#define minLimit 10

#define RC_CONST 0.632120558828557678404476
#define ADC_TRIGGER_V 0.20
#define VCC_CORRECT (3210-3218)

#define BIAS_R 2200
#define BIAS_MV 2200

static int successLog = 0b1111;

typedef ringbuffer_t<int, 512> ringbuffer_med_t;
static ringbuffer_med_t message_buffer;

typedef ringbuffer_t<uint32_t, 512> ringbuffer_long_t;
static ringbuffer_long_t time_buffer;

int VCC_Self = 0;

void setup_vcc_sensor() {
  adc_reg_map *regs = ADC1->regs;
  regs->CR2 |= ADC_CR2_TSVREFE;    // enable VREFINT and temp sensor
  regs->SMPR1 =  ADC_SMPR1_SMP17;  // sample rate for VREFINT ADC channel
}



void setup() {
  Serial.begin(115200);
  pinMode(pinLED, OUTPUT);
  digitalWrite(pinLED, HIGH);
  pinMode(buzzerPin, OUTPUT);
  pinMode(PA0, INPUT_ANALOG);
  pinMode(PA1, INPUT_ANALOG);

  delay(500);

  Serial.println("START");
  dac.begin(0x60);

  Serial.print("\r\nPrint Internal VCC Voltage: ");
  setup_vcc_sensor();
  systick_uptime_millis = 0; // really sleazy way to reset millis;

  //uint32_t t0 = millis();
  //Serial.print(t0); Serial.print(" ");
  delay(100);
  VCC_Self = 1200 * 4096 / adc_read(ADC1, 17);  // ADC sample to millivolts

  //Serial.print(millivolts/1000,DEC);Serial.print(".");Serial.print(millivolts%1000,DEC);
  Serial.print(VCC_Self / 1000.0, 2);
  Serial.println("v");


  //110 is correction from predicted VCC to actual Multimeter at pin
  Serial.print("Vcc correction: ");
  Serial.print(VCC_CORRECT / 1000.0, 2);
  Serial.println("v");

  VCC_Self = VCC_Self - VCC_CORRECT;
  int cal = int(4095.0 * BIAS_MV / VCC_Self);
  Serial.print("Driving to 2.2v: "); Serial.print(" "); Serial.print(cal);
  Serial.println(" / 4095");
  dac.setVoltage(cal, false);

}

void loop() {
  digitalWrite(pinLED, LOW);
  sing(7, buzzerPin);
  /*
    sing(1, buzzerPin);

    sing(6, buzzerPin);
    sing(5, buzzerPin);
    sing(4, buzzerPin);
    sing(3, buzzerPin);
    sing(2, buzzerPin);
  */
/*
  message_buffer.push_back_nc(1234);
  time_buffer.push_back_nc(123);
  message_buffer.push_back_nc(5678);
  time_buffer.push_back_nc(456);
  timingCheck(5678);
*/


  Serial.println();

  //  static int i = minLimit;
  //  static int mod = 10;


  //  i = i + mod;
  //  if ( i > maxLimit || i < minLimit ) {
  //    mod *= -1;
  //  }




  //delay(i);
  digitalWrite(pinLED, HIGH);
  //Serial.println("Hello World");
  // delay(i);
  /*
    uint32 sumRead = 0;
    int n = 0;
    int tempRead = 0;
    for (n = 0; n < 100; n++) {
      delay(10);
      tempRead = analogRead(PA0);
      sumRead = sumRead + tempRead;
      //Serial.println(tempRead/4095.0 * VCC_Self/1000.0 ,4);
      //Serial.print(" ");
      //Serial.println(analogRead(PA1)*3300/4096);
    }
    Serial.print("Average over "); Serial.print(n); Serial.print(" samples is "); Serial.println( (float) sumRead / n *  (float) VCC_Self / 1000.0 / 4095.0 , 4);
  */



  flagTransition();
}


int resultIntegrate(int resistance, int timing)
{
  //Serial.println();
  if(resistance == -9){
    Serial.println("==== [Resetting] ====");
    sing(7, buzzerPin);
    return -9;
  }

  
  int summary = max (resistance, timing);
  switch (summary) {
    case 0:
      Serial.println("==== [Pass] ====");
      sing(5, buzzerPin);
      break;
    case 1:
      Serial.println("==== [Marginal pass] ====");
      sing(2, buzzerPin);
      break;
    case 2:
      Serial.println("==== [Fail!] ====");
      sing(4, buzzerPin);
      break;
    case 3:
      Serial.println("==== [MAJOR FAIL!] ====");
      sing(3, buzzerPin);
      break;
  }

return 0;
}

int timingCheck(int vf)
{
  Serial.print("Running timing check on "); Serial.print(message_buffer.available()); Serial.println(" buffer entries");
  //RC_CONST
  /*
    Serial.println("Dumping circular buffer...");
    for (int n = 0; message_buffer.available(); n++) {
      int val = message_buffer.pop_front();
      uint32_t time = time_buffer.pop_front();
      Serial.print(n);
      Serial.print(": ");
      Serial.print(time);
      Serial.print(" <-> ");
      Serial.println(val);
    }
  */

  if (message_buffer.empty()) {
    Serial.println("-- No data in buffer.");
    return 0;
  }

  uint32_t t0 = time_buffer.pop_front();
  uint32_t t1 = t0;
  int v0 = message_buffer.pop_front();
  int v1 = v0;

  Serial.print("-- t0:\t");
  Serial.print(t0);
  Serial.print("\tv0:\t");
  Serial.println(v0);

  Serial.print("-- tf:\t");
  Serial.print("---");
  Serial.print("\tvf:\t");
  Serial.println(vf);

  for (int n = message_buffer.available() + 1; message_buffer.available() > 0; n--) {
    static float ratio;
    int vtemp;
    uint32_t ttemp;
    vtemp = message_buffer.pop_back();
    ttemp = time_buffer.pop_back();

    ratio = ((float) (vtemp - v0)) / ((float)(vf - v0));
    /*
        Serial.print("-- ");
        Serial.print(n);
        Serial.print(":\t");
        Serial.print(ttemp);
        Serial.print("\t  \t");
        Serial.print(vtemp);
        Serial.print("\t \t");
        Serial.println(ratio * 100, 2);
    */
    //If % to goal is less  than 63.2% break
    if ( ratio <= RC_CONST) {

      ratio = ((float) (v1 - v0)) / ((float)(vf - v0));
      Serial.print("-- t");
      Serial.print(n);
      Serial.print(":\t");
      Serial.print(t1);
      Serial.print("\tv");
      Serial.print("n:\t");
      Serial.print(v1);
      Serial.print("\t");
      Serial.println(ratio * 100, 2);

      break;
    }
    v1 = vtemp;
    t1 = ttemp;
  }

  int trc = t1 - t0;
  Serial.print("-- Time to RC constant (mills):\t");
  Serial.println(trc);
  if (trc <= 100) {
    Serial.println("== [Pass] (<100ms)");
    return 0;
  }
  else if (trc <= 160) {
    Serial.println("== [Marginal pass] (<160ms)");
    return 1;
  }
  else if (trc <= 250) {
    Serial.println("== [Fail!] (<250ms)");
    return 2;
  }
  else {
    Serial.println("== [MAJOR FAIL!] (>250ms)");
    return 3;
  }

  Serial.println("SOMETHING WEIRD SLIPPED THROUGH");
  sing(9, buzzerPin);
  return -1;
}


int resistanceCheck(float functionR)
{
  /*
    Ear        16 ohms or higher  Recommend 32 - 300 ohms
    0   67.5   Function A [Play/Pause]
    135 187.5  Function D [Reserved/Nexus Voice command]
    240 355    Function B [Vol+]
    470 735    Function C [Vol-]
    Mic        1000 ohms minimum
  */
  float tolerance = 0;
  float target = 0;
  float drift = 0;
  float rmin = 0;
  float rmax = 0;

  Serial.print("Running resistance check on "); Serial.println(functionR);
  if (functionR < 0 || functionR > 100000){
    Serial.println("-- Open circuit");
    Serial.println("== [RESETTING]");
    return -9;
  }


  if (functionR >= 800) {
    target = 1000;
    tolerance = -1;
    rmin = 1000;
    rmax = -1;
    Serial.println("-- Microphone");
  }
  else if (functionR < 800 && functionR >= 355) {
    target = 470;
    tolerance = 1;
    rmin = 355;
    rmax = 800;
    Serial.println("-- Function C [Vol-]");
  }
  else if (functionR < 355 && functionR >= 187.5) {
    target = 240;
    rmin = 187.5;
    rmax = 355;
    tolerance = 1;
    Serial.println("-- Function B [Vol+]");
  }
  else if (functionR < 187.5 && functionR >= 67.5) {
    target = 135;
    tolerance = 1;
    rmin = 67.5;
    rmax = 187.5;
    Serial.println("-- Function D [Reserved]");
  }
  else if (functionR < 67.5 && functionR >= 0) {
    target = 0;
    tolerance = -1;
    rmin = -1;
    rmax = 67.5;
    Serial.println("-- Function A [Play/Pause]");
  }


  Serial.print("-- Drift of ");

  // DO NOT drift-check (a) 0 target or (b)No-tolerance measurements
  if (tolerance >= 0 && target > 0 ) {
    drift = abs(functionR - target) / target * 100.0;
    Serial.print(drift); Serial.print("%");
  }
  else {
    drift = -1;
    Serial.print(abs(functionR - target)); Serial.print("(abs)");
  }


  Serial.print(" from target ");
  Serial.print(target, 0);

  if (tolerance >= 0 && target > 0) {
    Serial.print("±"); Serial.print(tolerance, 0); Serial.println("%");
  }
  else {
    Serial.print(" (Limit");
    if (rmin >= 0) {
      Serial.print(" "); Serial.print(rmin); Serial.print(" min");
      if (rmax >= 0) Serial.print (" -");
    }
    if (rmax >= 0) {
      Serial.print(" "); Serial.print(rmax); Serial.print(" max");
    }
    Serial.println(")");
  }

  //  drift = abs(functionR - target);
  //  tolerance = target * tolerance;

  //PASS FAIL LOGIC
  //===============

  float toCompare = -1;

  if (drift >= 0) {
    if ( drift <= tolerance ) {
      Serial.println("== [Pass (tolerance)]");
      //sing(5, buzzerPin);
      return 0;
    }
  }

  if (functionR >= target)
    toCompare = (rmax >= 0) ? rmax : -1;
  if (functionR < target)
    toCompare = (rmin >= 0) ? rmin : -1;

  //If there's a checkbox
  if (toCompare >= 0) {
    //Get drift "to the limit that side"
    drift = abs(functionR - target) / abs(target - toCompare) * 100.0;


    Serial.print("-- Margin ");
    Serial.print(drift); Serial.print("% of threshold ");
    Serial.println(toCompare);


    if (drift <= 25 ) {
      Serial.println("== [Marginal pass]");
      return 1;
      //sing(2, buzzerPin);
    }
    else if (drift <= 50) {
      Serial.println("== [Fail!]");
      return 2;
      //sing(4, buzzerPin);
      ////sing(8, buzzerPin);
    }
    else if (drift > 50) {
      Serial.println("== [MAJOR FAIL!]");
      return 3;
      //sing(3, buzzerPin);
    }
    return 0;
  }
  else if (toCompare < 0) {

    Serial.println("== [Pass (no limit)]");
    //sing(5, buzzerPin);
    return 0;
  }

  Serial.println("SOMETHING WEIRD SLIPPED THROUGH");
  sing(9, buzzerPin);
  return -1;
}


void flagTransition()
{

  uint32_t t0 = millis();
  uint32_t t1 = t0;
  uint32_t x0 = t0;
  int tempRead = 0;
  int lastRead = 0;
  bool neverRead = true;
  bool quasiStable = false;
  bool longStable = false;
  float quasiVal;
  float equivR;

  int quasiRead = 0;
  uint32 sumRead = 0;
  int n = 0;

  float threshold = ADC_TRIGGER_V;//.040; //.045
  int adcThreshold = threshold * 1000.0 / (float) VCC_Self * 4095;
  static int x_count = 0;

  static bool logging = false;

  while (true) {
    Serial.print("Waiting for transition above "); Serial.print(threshold); Serial.print(" or ADC of "); Serial.println(adcThreshold);
    while (true) {
      delay(ADC_DELAY);
      lastRead = tempRead;
      tempRead = analogRead(PA0);


      if (neverRead) {
        neverRead = false;
        lastRead = tempRead;
        quasiRead = tempRead;
        message_buffer.clear();
        time_buffer.clear();
      }

      if (!logging) {
        x0 = t1;
        t1 = millis();
      }
      else {
        t1 = millis();
        message_buffer.push_back(tempRead);
        time_buffer.push_back(t1);
      }


      //Serial.print(tempRead); Serial.print(" -- "); Serial.println( ((float)tempRead)/4095.0 * (float) VCC_Self/1000.0 ,4);

      sumRead += tempRead;
      n++;

      //========================
      // QUASI-STABLE LOGIC

      if (quasiStable == false && t1 - t0 > 1000) {

        quasiStable = true;
        quasiRead = sumRead / n;
        quasiVal = ((float) sumRead) / ((float)n) *  ((float) VCC_Self) / 1000.0 / 4095.0;
        Serial.print("Quasi-stable over "); Serial.print(n); Serial.print(" samples at "); Serial.print( quasiRead ); Serial.print(" = "); Serial.println( quasiVal , 4);

        equivR = BIAS_R * quasiVal / ( BIAS_MV / 1000.0 - quasiVal);
        Serial.print("Equivalent resistance: "); Serial.println(equivR);
      }


      //========================
      // STABLE LOGIC

      if (longStable == false && t1 - t0 > 2000) {
        //Stop logging either here or in full stable
        if (logging) {
          logging = false;
          digitalWrite(pinLED, HIGH);
          Serial.print("±");
        }


        longStable = true;
        quasiRead = sumRead / n;
        quasiVal = ((float) sumRead) / ((float)n) *  ((float) VCC_Self) / 1000.0 / 4095.0;
        Serial.print("Stable over "); Serial.print(n); Serial.print(" samples at "); Serial.print( quasiRead ); Serial.print(" = "); Serial.println( quasiVal , 4);
        equivR = BIAS_R * quasiVal / ( BIAS_MV / 1000.0 - quasiVal);
        Serial.print("Equivalent resistance: "); Serial.println(equivR);

        Serial.print("Circular buffer has "); Serial.print(message_buffer.available()); Serial.println(" entries");

        Serial.println();
        Serial.println("================");
        int resistanceResult = resistanceCheck(equivR);
        int timingResult = timingCheck(quasiRead);
        resultIntegrate(resistanceResult, timingResult);

        message_buffer.clear();
        time_buffer.clear();

        systick_uptime_millis = 0;
        t1 = 0;
        t0 = 0;
        x0 = 0;
        sumRead = 0;
        n = 0;
        neverRead = true;
        //quasiStable=false; //optional
        //longStable=false; //optional
      }


      //===================
      // TRANSITION LOGIC


      if (abs(tempRead - lastRead) >= adcThreshold || (quasiStable && abs(quasiRead - ((lastRead + tempRead) / 2)) >= adcThreshold / 2) ) {

        if (quasiStable == true)
          Serial.println();
        if (!logging) {
          Serial.print("±");
          digitalWrite(pinLED, LOW);
          logging = true;
          message_buffer.clear();
          time_buffer.clear();
          //          systick_uptime_millis = 0;
          //          x0=0;
          //          t0=0;
          //          t1=0;
          message_buffer.push_back_nc(lastRead);
          time_buffer.push_back_nc(x0); //x0
          message_buffer.push_back_nc(tempRead);
          time_buffer.push_back_nc(t1); //t1
        }

        x_count++;
        Serial.print("TRANSITION! "); Serial.println(x_count);

        if (abs(tempRead - lastRead) >= adcThreshold) {
          //Serial.println("Mode A");
          Serial.print (tempRead); Serial.print(" <=x= "); Serial.println(lastRead);
        }

        if (quasiStable && abs(quasiRead - tempRead) >= adcThreshold / 2) {
          //Serial.println("Mode B");
          Serial.print (tempRead); Serial.print(" <=q= "); Serial.println(quasiRead);
        }

        //CLEANUP post-transition detect
        //DO NOT reset millis() here!!! Logging disrupted! systick_uptime_millis = 0;
        t0 = t1;
        sumRead = 0;
        n = 0;
        quasiStable = false;
        longStable = false;

        break;
      }
    }
  }
  return;
}



