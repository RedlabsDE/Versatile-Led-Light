/*
   Versatile Led Light

   Created: 15.08.2020 11:55
    Author: Julian

  History:
  15.08.2020 initial version of battery powered Motion Light
  16.12.2022 move to git
*/

/*
  Description of hardware connection and firmware function - initial version of battery powered Motion Light

  Board: Arduino Mini (without USB)
  Arduino IO Setting:
    Arduino Pro or Pro Mini
    Atmega328P (5V, 16MHz)

  Supply
    The Project is designed to run on battery supply (4...6V with 4 AA Cells, 1.2V NiMh)
    The Battery voltage is connected directly to the "5V" net of Arduino
    The battery voltage is monitored. (100k/1k voltage divider on ADC pin)
      At battery level "low" the LEDs will show red instead of orange 
      At battery level "empty" the LEDs will blink red once, than the mcu is set to sleep without wakeup option

  Sleep Modes
    Sleep mode "normal" with motion sensor active and used as wakeup-source (~0.5mA)
    Sleep mode "battery empty" with motion sensor not supplied and no wakeup-source (~0.3mA)

  IR-Motion Sensor
    5V Sensor supply via uC pin
    Sensor output used to wake up from sleep mode via interrupt

  WS2812 LED strip
    5V LED supply with switched Battery Voltage
    Voltage is switched with a single PFET IRF4905 (to reduce quiscent current of 0.5mA/LED)
       
*/

//use serial output and onboard LED for debug
#define DEBUG 0


/////////////////////////////////////////////////////////////////////////////////////
#ifdef __AVR__
  #include <avr/power.h>
#endif

#include <avr/sleep.h>
/////////////////////////////////////////////////////////////////////////////////////
#include <Adafruit_NeoPixel.h>
/////////////////////////////////////////////////////////////////////////////////////
//Pin of WS2812 LED Strip DATA connection
#define PIN            8

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      15

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

/////////////////////////////////////////////////////////////////////////////////////
//WS2812 LED Supply switch
#define SUPPLY_SWITCH_PIN  12

#define SUPPLY_SWITCH_ENABLE   digitalWrite(SUPPLY_SWITCH_PIN, LOW) //active low
#define SUPPLY_SWITCH_DISABLE  digitalWrite(SUPPLY_SWITCH_PIN, HIGH) //active low

/////////////////////////////////////////////////////////////////////////////////////
// Battery Voltage Measurement
#define CELL_COUNT 4
#define VBAT_MV_0_PERCENT 1060 * CELL_COUNT //4.24V
#define VBAT_MV_5_PERCENT 1130 * CELL_COUNT //4.52V

#define BAT_U_SENS_PIN  0 //A0

/////////////////////////////////////////////////////////////////////////////////////
// IR Motion Sensor
#define MOTION_SUPPLY_PIN   3 // Motion sensor is attached to GND and two GPIOs, this for supply
#define MOTION_PIN          2 // Motion sensor status pin 
#define MOTION_INTERRUPT_CH 0 // Motion sensor interrupt channel 

#define SENSOR_SUPPLY_ENABLE  digitalWrite(MOTION_SUPPLY_PIN, HIGH) //active high
#define SENSOR_SUPPLY_DISABLE digitalWrite(MOTION_SUPPLY_PIN, LOW) //active high

/////////////////////////////////////////////////////////////////////////////////////
// DEBUG LED
#define ONBOARD_LED_PIN     13 //Arduino Mini onboard LED
/////////////////////////////////////////////////////////////////////////////////////

bool motionActive = 0; //state of motion sensor output pin
bool ledsOn = false; //current state of LEDs

bool batteryIsEmpty = true; // voltage below critical level - no operation
bool batteryIsLow = true;   // voltage below low level - show status

void fadeUp();    //call on rising edge of motion sensor
void fadeDown();  //call on falling edge of motion sensor
void indicateEmptyBattery(); // blink one LED red, to indicate low battery level

void WAKE_UP(); // called by motion sensor pin interrupt
void GO_TO_SLEEP(bool enableWakeup); // set mcu to sleep, enable/disable wakup-option

void checkBattery(); // set battery state: batteryIsEmpty & batteryIsLow
int CalculateVoltage(int adcValue); // calculate battery voltage in mV from raw ADC

int delayval = 100; //delay between each LED in ms

/////////////////////////////////////////////////////////////////////////////////////
/************************************************************************************************************************************************/
/* Setup
/************************************************************************************************************************************************/
void setup()
{
#if DEBUG
  Serial.begin(115200);
  Serial.write("Redlabs Motion Light - initial version");

  //ONBOARD DEBUG LED
  pinMode(ONBOARD_LED_PIN, OUTPUT);
#endif

  // This initializes the NeoPixel library.
  pixels.begin();
  pixels.setBrightness(150);

  // Motion Sensor Supply via uC pin
  pinMode(MOTION_SUPPLY_PIN, OUTPUT);
  SENSOR_SUPPLY_ENABLE;

  // Motion Sensor
  pinMode(MOTION_PIN, INPUT_PULLUP);

  // LED supply switch (5V switched via PFET)
  pinMode(SUPPLY_SWITCH_PIN, OUTPUT);

  SUPPLY_SWITCH_ENABLE;
  delay(10);
  
  checkBattery();
  
  if(!batteryIsEmpty)
  {
    fadeUp();
    fadeDown();
  }
}

/************************************************************************************************************************************************/
/* LOOP
/************************************************************************************************************************************************/
void loop()
{
  if (batteryIsEmpty)
  {
    indicateEmptyBattery();
    GO_TO_SLEEP(false);
  }

  // Motion is active
  if (digitalRead(MOTION_PIN))
  {
    motionActive = true;
    digitalWrite(ONBOARD_LED_PIN, HIGH);

    if (ledsOn == false) //motion start
    {
      fadeUp();
      ledsOn = true;
    }
  }
  else // no motion detected
  {
    motionActive = false;
    digitalWrite(ONBOARD_LED_PIN, LOW);

    if (ledsOn == true) // motion end
    {
      fadeDown();
      ledsOn = false;
    }
    else // no motion
    {
      GO_TO_SLEEP(true);
      // ... continue here on MCU wakeup (motion sensor event)
    }
  }

  checkBattery();
  delay(10);
}

/************************************************************************************************************************************************/
/** Set MCU to sleep mode

    @param bool enableWakeup - (true): use motion pin interrupt to wake up (false): no option to wake up mcu, only by reset
    @return /
*/
/************************************************************************************************************************************************/
void GO_TO_SLEEP(bool enableWakeup)
{
  #if DEBUG
    Serial.write(" go to sleep ... ");

    if(!enableWakeup)
    {
      Serial.write(" ...forever... ");
    }
    digitalWrite(ONBOARD_LED_PIN, LOW);

    Serial.end();
  #endif

  SUPPLY_SWITCH_DISABLE;
  sleep_enable();
  
  if (enableWakeup) // normal sleep - use motion sensor to wake up MCU
  {    
    attachInterrupt(MOTION_INTERRUPT_CH, WAKE_UP, RISING); 
  }
  else // sleep because battery is empty - no option to wake up again, disable motion sensor
  {
    detachInterrupt(MOTION_INTERRUPT_CH); // motion sensor will not trigger an interrupt to wake up MCU
    SENSOR_SUPPLY_DISABLE; // motion sensor is not used, disable sensor supply
  }
  
  delay(10); // important delay!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); //set full sleep mode
  sleep_cpu(); // got to sleep ...
}

/************************************************************************************************************************************************/
/** Disable sleep mode

    this function is called by pin interrupt
*/
/************************************************************************************************************************************************/
void WAKE_UP()
{
  sleep_disable();
  detachInterrupt(MOTION_INTERRUPT_CH);

  SUPPLY_SWITCH_ENABLE;
  delay(50);

#if DEBUG
  Serial.begin(115200);
  Serial.write(" ...just woke up ");
  digitalWrite(ONBOARD_LED_PIN, HIGH);
#endif
  
}

/************************************************************************************************************************************************/
/** Enable leds and dim up

    call on rising edge of motion sensor
*/
/************************************************************************************************************************************************/
void fadeUp()
{
  for (int i = 0; i < NUMPIXELS; i++)
  {
    if (batteryIsLow)
    {
      pixels.setPixelColor(i, pixels.Color(100, 0, 0)); // Set LEDs red
    }
    else
    {
      pixels.setPixelColor(i, pixels.Color(100, 50, 0)); // Set LEDs orange
    }

    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval); // Delay for a period of time (in milliseconds).
  }
}

/************************************************************************************************************************************************/
/** Dim down LEDs and disable LED supply

    call on falling edge of motion sensor
*/
/************************************************************************************************************************************************/
void fadeDown()
{
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, pixels.Color(0, 0, 0)); // set LEDs OFF
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval*2); // Delay for a period of time (in milliseconds).
  }
}

/************************************************************************************************************************************************/
/** Blink red on first LED

    Use to indicate that battery in empty
*/
/************************************************************************************************************************************************/
void indicateEmptyBattery()
{
  for (int i = 0; i < 5; i++)
  {
    pixels.setPixelColor(0, pixels.Color(100, 0, 0)); // Set first LED red
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval);
    pixels.setPixelColor(0, pixels.Color(0, 0, 0)); // Set first LED OFF
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval);
  }
}

/************************************************************************************************************************************************/
/* Set battery state dependant on battery voltage
  /************************************************************************************************************************************************/
void checkBattery()
{
  SUPPLY_SWITCH_ENABLE;
  delay(10);
  
  // use intern 1.1V ADC reference voltage
  analogReference(INTERNAL);
  
  /// Do ADC sampling and mean calculation
  int vBatADC = 0;
  for (int i = 0; i < 10; i++)
  {
    vBatADC += analogRead(BAT_U_SENS_PIN);
  }
  vBatADC = vBatADC / 10;

  /// Calculate battery voltage in mV (from ADC)
  int vBat = CalculateVoltage(vBatADC);

  if (vBat < VBAT_MV_0_PERCENT)
  {
    batteryIsEmpty = true;
    batteryIsLow = true;
  }
  else if ( vBat < VBAT_MV_5_PERCENT)
  {
    batteryIsEmpty = false;
    batteryIsLow = true;
  }
  else
  {
    batteryIsEmpty = false;
    batteryIsLow = false;
  }
}

/************************************************************************************************************************************************/
/* ADC -> Voltage calculation
/************************************************************************************************************************************************/
/* Calculate Battery Voltage in mV from ADC raw data

   Vref: 1.1V (intern)
   R_up:    10k
   R_down:  1k

   Vbat 0...10V (normal operation range 4...6V)
   Vpin 0...1V

   21.12.2020
   Calibration point #1: 6000 mV = 507 ADC (measured)
   Calibration point #2: 4000 mV = 332 ADC (measured)

   slope = (V_CAL_#1 - V_CAL_#2) / (ADC_CAL_#1 - ADC_CAL_#2)
   offset = (slope * ADC_CAL_#1) - V_CAL_#1
*/
int CalculateVoltage(int adcValue)
{
  int voltage_mV;

  // set slope & offset of the ADC transfer function
  float slope = (6000.0 - 4000.0) / (507.0 - 332.0); //10bit ADC, 10V range //2000/(175) = 11,42857
  float offset = -205.714;

  voltage_mV = slope * adcValue - offset;

#if DEBUG
/*
  //use adc output for calibration
  Serial.print("ADC: ");
  Serial.print(adcValue);
  Serial.print(" Vbat: ");
  Serial.print(voltage_mV);
  Serial.println();
  */  
#endif

  return voltage_mV;
}
