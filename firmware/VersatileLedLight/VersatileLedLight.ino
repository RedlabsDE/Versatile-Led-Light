/*
   Versatile Led Light

   Created: 15.08.2020 11:55
    Author: Julian

  History:
  15.08.2020 initial version of battery powered Motion Light (Tag 1.0)
  16.12.2022 move to git 
  17.12.2022 restructure to button-controlled light (Tag 2.0)
  26.02.2023 disable auto-off timer (Tag 2.1)
*/

/*
  Description of hardware connection and firmware function - initial version of battery powered Motion Light (Tag 1.0)

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
#define DEBUG 1


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
#define NUMPIXELS      144

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

/////////////////////////////////////////////////////////////////////////////////////
//WS2812 LED Supply switch
#define SUPPLY_SWITCH_PIN  9 //12

#define SUPPLY_SWITCH_ENABLE   digitalWrite(SUPPLY_SWITCH_PIN, LOW) //active low
#define SUPPLY_SWITCH_DISABLE  digitalWrite(SUPPLY_SWITCH_PIN, HIGH) //active low

/////////////////////////////////////////////////////////////////////////////////////
// Battery Voltage Measurement
#define USE_BATTERY_MEASUREMENT false

#if (USE_BATTERY_MEASUREMENT==true)
  #define CELL_COUNT 4
  #define VBAT_MV_0_PERCENT 1060 * CELL_COUNT //4.24V
  #define VBAT_MV_5_PERCENT 1130 * CELL_COUNT //4.52V

  #define BAT_U_SENS_PIN  0 //A0
#endif

/////////////////////////////////////////////////////////////////////////////////////
// IR Motion Sensor
#define USE_MOTION_SENSOR false

#define MOTION_SUPPLY_PIN   3 // Motion sensor is attached to GND and two GPIOs, this for supply
#define MOTION_PIN          2 // Motion sensor status pin 
#define MOTION_INTERRUPT_CH 0 // Motion sensor interrupt channel 

 #define MOTION_SENSOR_IS_ACTIVE digitalRead(MOTION_PIN) //active high

#define SENSOR_SUPPLY_ENABLE  digitalWrite(MOTION_SUPPLY_PIN, HIGH) //active high
#define SENSOR_SUPPLY_DISABLE digitalWrite(MOTION_SUPPLY_PIN, LOW) //active high

/////////////////////////////////////////////////////////////////////////////////////
// User push button
#define USE_USER_PUSHBUTTON true

#define BUTTON_PRESS_TIME_LONG_MS 800
#define BUTTON_PRESS_TIME_SHORT_MS 100

#define USER_BUTTON1_PIN 2//TODO
#define USER_BUTTON1_INTERRUPT_CH 0 // Motion sensor interrupt channel 

#define USER_BUTTON1_PRESSED (digitalRead(USER_BUTTON1_PIN)==0) //active low, return true if button is pressed

/////////////////////////////////////////////////////////////////////////////////////
// DEBUG LED
#define ONBOARD_LED_PIN     13 //Arduino Mini onboard LED
/////////////////////////////////////////////////////////////////////////////////////
// AUTO-OFF Timer
#define AUTO_OFF_TIMER_ENABLED false
#define TIMER_LED_ON_SEC  60

/////////////////////////////////////////////////////////////////////////////////////

struct struct_SystemTimerEvent
{
  bool timerIsRunning = false;
  unsigned long startTime = 0; //set to millis() at start
  unsigned long duration_ms = 0; //set to desired time

  bool timerDonePending = false;

};

enum systemStatus
{
  stat_invalid = -1,
  stat_powerup = 1,
  stat_go_to_sleep,
  stat_sleep,
  stat_wakeup,
  stat_on
};

enum ledStatus
{
  ledStat_invalid = -1,
  ledStat_allOff = 0,
  ledStat_on = 1
};

struct struct_SystemTime
{
  unsigned long now = 0;

  //automatic mode/color change
  /*
  unsigned long last_change_color = 0;
  unsigned long next_change_color = TIMER_MS_AUTO_CHANGE_MIN;
  unsigned long last_change_mode = 0;
  unsigned long next_change_mode = TIMER_MS_AUTO_CHANGE_MIN;
  */

  //Sound to Light
  //unsigned long Sound_last_trigger = 0; //sound to light trigger

  //Button
  bool Button_isReleased = false; // a not pressed button will set this true. Used to prevent multiple long-button-press-events.
  unsigned long Button_PressDuration = 0;
  unsigned long Button_last_falling_edge = 0;
  bool Button_Shortpress_event = false;
  bool Button_Longpress_event = false;

  //Timer
  struct struct_SystemTimerEvent SystemTimer_LED;

  //Status indication
  unsigned long status_indication_start = 0;


  int8_t systemStatus = stat_invalid;
  int8_t ledStatus = ledStat_allOff;

} SystemTime;

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

void checkButton();
void action_ButtonPressShort();
void action_ButtonPressLong();


int delayval = 5; //delay between each LED in ms

/////////////////////////////////////////////////////////////////////////////////////
/************************************************************************************************************************************************/
/* Setup
/************************************************************************************************************************************************/
void setup()
{
#if DEBUG
  Serial.begin(115200);
  Serial.write("Redlabs Versatile Led Light - nightlight");

  //ONBOARD DEBUG LED
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  digitalWrite(ONBOARD_LED_PIN, HIGH);
  
#endif


  SystemTime.systemStatus = stat_powerup;

  // This initializes the NeoPixel library.
  pixels.begin();
  pixels.setBrightness(50);

#if (USE_MOTION_SENSOR==true)
  // Motion Sensor Supply via uC pin
  pinMode(MOTION_SUPPLY_PIN, OUTPUT);
  SENSOR_SUPPLY_ENABLE;

  // Motion Sensor
  pinMode(MOTION_PIN, INPUT_PULLUP);
#endif

#if (USE_USER_PUSHBUTTON == true)
  pinMode(USER_BUTTON1_PIN, INPUT_PULLUP);
  //pinMode(USER_BUTTON1_PIN, INPUT);
#endif

  // LED supply switch (5V switched via PFET)
  pinMode(SUPPLY_SWITCH_PIN, OUTPUT);

  SUPPLY_SWITCH_ENABLE;
  delay(10);
  
  checkBattery();
  
  if(!batteryIsEmpty)
  {
    fadeUpColor(250,50,0);
    fadeDown();
  }
  SystemTime.systemStatus = stat_go_to_sleep;
}

/************************************************************************************************************************************************/
/* LOOP
/************************************************************************************************************************************************/
void loop()
{
  #if (USE_BATTERY_MEASUREMENT == true)
  if (batteryIsEmpty)
  {
    indicateEmptyBattery();
    GO_TO_SLEEP(false);
  }
  #endif

  if(SystemTime.systemStatus == stat_go_to_sleep)
  {
    GO_TO_SLEEP(true);
  }


  #if(USE_MOTION_SENSOR == true)
  // Motion is active
  if (MOTION_SENSOR_IS_ACTIVE)
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
  #elif(USE_USER_PUSHBUTTON == true)
  checkButton();

  #endif


  checkTimer();
  checkBattery();
  delay(10);
}

void checkTimer()
{
  SystemTime.now = millis();

  if(SystemTime.SystemTimer_LED.timerIsRunning)
  {
    if( (SystemTime.SystemTimer_LED.startTime + SystemTime.SystemTimer_LED.duration_ms) < SystemTime.now)
    {
      SystemTime.SystemTimer_LED.timerDonePending = true;
      SystemTime.SystemTimer_LED.timerIsRunning = false;
    }
  }


  /////
  if(SystemTime.SystemTimer_LED.timerDonePending == true)
  {
    #if DEBUG
    Serial.write("\n SystemTimer_LED.timerDonePending == true");
    #endif
    
    SystemTime.SystemTimer_LED.timerDonePending = false;
    fadeDown(); //TODO: fade down slower
    SystemTime.ledStatus = ledStat_allOff;
    SystemTime.systemStatus = stat_go_to_sleep;
  }

}


void checkButton()
{
  static bool firstEdge = true;

  if(USER_BUTTON1_PRESSED)
  {
    if(firstEdge == true)
    {
      SystemTime.Button_last_falling_edge = millis();
      firstEdge = false;
    } 

    SystemTime.Button_PressDuration = millis() - SystemTime.Button_last_falling_edge;

    if(SystemTime.Button_PressDuration > BUTTON_PRESS_TIME_LONG_MS && SystemTime.Button_isReleased == true)
    {
      SystemTime.systemStatus = stat_on;
      action_ButtonPressLong();
      SystemTime.Button_isReleased = false;
    }
  }
  else
  {
    //short: released after BUTTON_PRESS_TIME_SHORT_MS hold time
    if(SystemTime.Button_PressDuration > BUTTON_PRESS_TIME_SHORT_MS && SystemTime.Button_isReleased == true)
    {
      SystemTime.systemStatus = stat_on;
      action_ButtonPressShort();
    }

    SystemTime.Button_PressDuration = 0;
    SystemTime.Button_isReleased = true;
    firstEdge = true;

    if(SystemTime.systemStatus == stat_wakeup)
    {
      SystemTime.systemStatus = stat_go_to_sleep;
    }
  }
}
void action_ButtonPressShort()
{

  #if DEBUG
  Serial.write("\n action_ButtonPressShort()");
  #endif

  if(SystemTime.ledStatus != ledStat_on)
  {
    #if DEBUG
    Serial.write(" - set led on by timer");
    #endif

    SystemTime.ledStatus = ledStat_on;
    //set color1
    fadeUpColor(100,0,0);

    #if AUTO_OFF_TIMER_ENABLED
    //start timer to turn off LEDs
    
    SystemTime.SystemTimer_LED.startTime = millis();
    SystemTime.SystemTimer_LED.duration_ms = (unsigned long) TIMER_LED_ON_SEC*1000;
    SystemTime.SystemTimer_LED.timerIsRunning = true;

    Serial.write("\n Time start: ");
    Serial.println(millis());// Gibt die Zeit seit dem Programmstart aus
    Serial.write("\n duration_ms ");
    Serial.println(SystemTime.SystemTimer_LED.duration_ms);// Gibt die Zeit seit dem Programmstart aus
    #endif
  

  }
  else if (SystemTime.ledStatus == ledStat_on)
  {
    #if DEBUG
    Serial.write(" - set led off");
    #endif

    SystemTime.SystemTimer_LED.timerIsRunning = false;
    //turn off direct
    fadeDown();
    SystemTime.ledStatus = ledStat_allOff;
    SystemTime.systemStatus = stat_go_to_sleep;
  }
}

void action_ButtonPressLong()
{
  #if DEBUG
  Serial.write("\n action_ButtonPressLong()");
  #endif
  if(SystemTime.ledStatus != ledStat_on)
  {
    #if DEBUG
    Serial.write(" - set led on");
    #endif

    SystemTime.ledStatus = ledStat_on;
    //set color2
    fadeUpColor(250,50,0);
  }
  else if(SystemTime.ledStatus == ledStat_on)
  {
    #if DEBUG
    Serial.write(" - set led off");
    #endif
    SystemTime.SystemTimer_LED.timerIsRunning = false;
    //turn off direct
    fadeDown();
    SystemTime.ledStatus = ledStat_allOff;
    SystemTime.systemStatus = stat_go_to_sleep;
  }

}


/************************************************************************************************************************************************/
/** Set MCU to sleep mode

    @param bool enableWakeup - (true): use motion pin interrupt to wake up (false): no option to wake up mcu, only by reset
    @return /
*/
/************************************************************************************************************************************************/
//dummy function
/*
void GO_TO_SLEEP(bool enableWakeup)
{
  Serial.write(" go to sleep dummy... ");
  SUPPLY_SWITCH_DISABLE;
  SystemTime.systemStatus = stat_sleep;
}
*/

//first sleep causes direct wakeup. second sleep works ?!
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
    #if (USE_MOTION_SENSOR == true)
      attachInterrupt(MOTION_INTERRUPT_CH, WAKE_UP, RISING); 
    #endif

    #if (USE_USER_PUSHBUTTON == true)
      //pinMode(USER_BUTTON1_PIN, INPUT_PULLUP);
      attachInterrupt(USER_BUTTON1_INTERRUPT_CH, WAKE_UP, FALLING); //sollte falling sein

    #endif 
  }
  else // sleep because battery is empty - no option to wake up again, disable motion sensor
  {
    #if (USE_MOTION_SENSOR == true)
      detachInterrupt(MOTION_INTERRUPT_CH); // motion sensor will not trigger an interrupt to wake up MCU
      SENSOR_SUPPLY_DISABLE; // motion sensor is not used, disable sensor supply
    #endif
  }
  
  //SystemTime.systemStatus = stat_sleep;
  SystemTime.systemStatus = stat_go_to_sleep;
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

  //TODO: reset wakeup by interrupt??
  //EIFR = (1<<INTF0); 

  sleep_disable();

  #if (USE_MOTION_SENSOR == true)
    detachInterrupt(MOTION_INTERRUPT_CH);
  #endif

  #if (USE_USER_PUSHBUTTON == true)
    detachInterrupt(USER_BUTTON1_INTERRUPT_CH);
  #endif 

  SystemTime.systemStatus = stat_wakeup;
  //SystemTime.systemStatus = stat_go_to_sleep;

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
//fade up by turning on one led after each other
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
      pixels.setPixelColor(i, pixels.Color(250, 50, 0)); // Set LEDs orange
    }

    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval); // Delay for a period of time (in milliseconds).
  }
}

//fade up all leds at the same time
void fadeUpColor(uint8_t r, uint8_t g, uint8_t b)
{
  uint8_t red = 0;
  uint8_t green = 0;
  uint8_t blue = 0;
  for(int i=0; i<=255; i++)
  {
    red = r;
    green = g;
    blue = b;
    if(i<r)
    {
      red = i;
    }
    if(i<g)
    {
      green = i;
    }
    if(i<b)
    {
      blue = i;
    }

    for (int p = 0; p < NUMPIXELS; p++)
    {
      pixels.setPixelColor(p, pixels.Color(red, green, blue)); // Set LEDs red
    }

    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval); // Delay for a period of time (in milliseconds).
  }
}

//fade up by turning on one led after each other
void fadeUpColor2(uint8_t r, uint8_t g, uint8_t b)
{
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, pixels.Color(r, g, b)); // Set LEDs red
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
#if (USE_BATTERY_MEASUREMENT==true)
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
#else
  batteryIsEmpty = false;
  batteryIsLow = false;
#endif
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
