//--------------------------------------------------------------------------------------------
// ARDUINO MPPT SOLAR CHARGE CONTROLLER
//
// This code is for an arduino Pro-Micro based Solar MPPT charge controller.
// This code is a modified version of sample code from www.timnolan.com
// dated 08/02/2015
//
// Mods by KI0BK started 01-Feb-2018
/*
 * Version  1.1.1 1-May-2018 added factory setup for two sizes of lifepo4 packs
 * Version  1.1  30-apr-2018 Release for Novexcomm added support for 24v solar panels
 * Version  1.0   8-apr-2018 Release for first two boards to Novexcomm for use with 20AH LiFePO4
*/
// Size batteries and panels appropriately. Larger panels need larger batteries and vice versa.
//
//// Specifications : /////////////////////////////////////////////////////////////////////////
//
// 1.Solar panel power = 150W
//
// 2.Rated Battery Voltage= 14v (LiFePO4 type 4S)
//
// 3.Maximum solar current = 6A
//
// 4. Input Voltage = Solar panel with Open circuit voltage from 17 to 25V
//    Goal is use with 24 volt panels as well.
//
///////////////////////////////////////////////////////////////////////////////////////////////
#include <EEPROM.h>
#include "TimerOne.h" // using Timer1 library from http://www.arduino.cc/playground/Code/Timer1
// another Timer1 library from  https://github.com/PaulStoffregen/TimerOne

//----------------------------------------------------------------------------------------------------------
//////// Arduino pins Connections//////////////////////////////////////////////////////////////////////////
//----------------------------------------------------------------------------------------------------------
// A0 - Voltage divider (solar)
// A1 - Panel current sensor
// A2 - Voltage divider (battery)
// A3 - Battery current sensor
// A7 - Voltage divider (power supply)
// D0, D1   Reserved, 2nd serial
// D2 - Low Battery buzzer
// D3 - LED Red
// D4   LED Grn
// D5 - LED Blu
// D7   Unused
// D8 - Unused
// D9 - PWM output
// D10  Alarm Silence button

///////// Definitions ////////////////////////////////////////////////////////////////////////////////
// Analog ports An
#define SOL_VOLTS_CHAN 0  // defining the adc channel to read solar volts
#define SOL_AMPS_CHAN  1  // Defining the adc channel to read solar amps
#define BAT_VOLTS_CHAN 2  // defining the adc channel to read battery volts
#define BAT_AMPS_CHAN  3  // Defining the adc channel to read battery amps
#define PS_VOLTS_CHAN  7  // Defining the adc channel ro read Power Supply volts
#define AVG_NUM        8  // number of iterations of the adc routine to average the adc readings

// ACS723LLCTR-10AU Current Sensor 10A is used, 400mV/Amp.
// Zero current output Vcc*0.1 so ADC = 102
// ADC for 5 A = 0.5 + 0.4*5= 2.5, ADC = 512
// ADC for 1 A = 0.5 + 0.4*1= 1.0, ADC = 204
// Measured current = (ADC-102)*5/1024/0.4 Same as formula below arranged differently
// 1 ADC integer represents 0.0122 Amps
// Current Measured = (5/(1024 * 0.4)) * ADC - (0.5/0.4)

#define SOL_AMPS_SCALE  0.01200 // the scaling value for raw adc reading to get solar amps // 5/(1024*0.4)
#define SOL_AMPS_CAL    1.285   // datasheet 1.25
#define BAT_AMPS_SCALE  0.01250 // the scaling value for raw adc reading to get battery amps // 5/(1024*0.4)
#define BAT_AMPS_CAL    1.25    // datasheet 1.25
#define SOL_VOLTS_SCALE 0.0498  // the scaling value for raw adc reading to get solar volts // (5/1024)*(R1+R2)/R2 // R1=100k and R2=11k
#define BAT_VOLTS_SCALE 0.0197  // the scaling value for raw adc reading to get battery volts 
#define PS_VOLTS_SCALE  0.0197  // the scaling value for raw adc reading to get solar volts // (5/1024)*(R1+R2)/R2 // R1=100k and R2=33k2
#define PWM_PIN           9 // the output pin for the pwm (only pin 9 avaliable for timer 1 at 50kHz)
#define PWM_ENABLE_PIN    8 // pin used to control shutoff function of the IR2104 MOSFET driver (when hight the mosfet driver is on)
#define TURN_ON_MOSFETS  digitalWrite(PWM_ENABLE_PIN, HIGH) // enable MOSFET driver
#define TURN_OFF_MOSFETS digitalWrite(PWM_ENABLE_PIN, LOW)  // disable MOSFET driver
#define TIMER_PERIOD     20 // number of microseconds interrupt period
#define ONE_SECOND  1000000/TIMER_PERIOD // number of interrupts in 1 second
#define MAX_BAT_VOLTS  14.6 // we don't want the battery going any higher than this
#define MIN_BAT_AMPS   0.50 // min charging current
//#define MAX_BAT_AMPS   2.00 // max charging current during cc mode (C/6)
#define MIN_MPPT_AMPS  1.00 // min PV current for mppt mode to start (must be above MIN_BAT_AMPS)
#define BAT_FLOAT      14.4 // battery voltage we want to stop charging at
#define BAT_CHARGE     13.0 //12.5 // battery voltage we restart charging again
#define OFF_NUM           9 // number of iterations to enter charger off state
#define NO_BAT         10.0 // voltage below which battery is considered to be disconnected
#define SAMP_NUM         15 // number of samples to average analog reads, must be 1 or greater
#define LOW_BATT_ALARM 12.9 // voltage below which alarm sounds
#define LOW_BATT_DELAY 30000 // ms below low battery level before alarm sounds
#define EEADDRESS 0xAA

//------------------------------------------------------------------------------------------------------
//Defining led pins for indication Dn
#define LED_RED    3
#define LED_GREEN  4
#define LED_BLUE   5

//-------------------------------------------------------------------------------------------------------
// Defining Low Battery Alarm ports Dn
#define ALARM_BUZZER 2
#define ALARM_BUTTON 10

//-------------------------------------------------------------------------------------------------------
// global variables
float sol_amps;           // solar amps
float sol_volts;          // solar volts
float bat_volts;          // battery volts
float bat_amps;           // battery amps
float ps_volts;           // power supply volts
float sol_watts;          // solar watts
float old_sol_watts;      // solar watts from previous time through mppt routine
float maxBattAmps;        // max charging rate

volatile unsigned int seconds;      // seconds from timer routine
unsigned int prev_seconds;          // seconds value from previous pass
unsigned int interrupt_counter;     // counter for 20us interrrupt

bool alarm_enable = true;
enum alarm_state { off, low_batt, on } alarm_state = off;

int pulseWidth = 0;           // pwm duty cycle 0-1023
int pwm;                      // mapped value of pulseWidth in %
int trackDirection = 1;       // step amount to change the value of pulseWidth used by MPPT algorithm

enum charger_mode {Start, no_battery, sleep, mppt, cv, cc, error} charger_state; // enumerated variable that holds state for charger state machine

//------------------------------------------------------------------------------------------------------
// This routine is automatically called at powerup/reset
//------------------------------------------------------------------------------------------------------
void setup()                        // run once, when the sketch starts
{
  pinMode(LED_RED, OUTPUT);         // sets the digital pin as output
  pinMode(LED_GREEN, OUTPUT);       // sets the digital pin as output
  pinMode(LED_BLUE, OUTPUT);        // sets the digital pin as output
  pinMode(PWM_ENABLE_PIN, OUTPUT);  // sets the digital pin as output
  pinMode(PWM_PIN, OUTPUT);         // sets the digital pin as output
  leds_off_all();
  pinMode(ALARM_BUZZER, OUTPUT);       // sets the digital pin as output
  pinMode(ALARM_BUTTON, INPUT_PULLUP); // input w/ pullup enabled

  Timer1.initialize(TIMER_PERIOD);     // initialize timer1, and set period
  Timer1.attachInterrupt(callback);    // attaches callback() as a timer overflow interrupt
  Timer1.pwm(PWM_PIN, 0, TIMER_PERIOD);// start pwm
  Serial.begin(115200);                // open the serial port
  Serial1.begin(115200);               // open the serial port
  //while (!Serial);                     // wait for serial to open
  //Serial.println("Up and Running!");
  charger_state = no_battery;          // set charger state to no_battery
  
  EEPROM.get(EEADDRESS, maxBattAmps);  // read max charging rate from eeprom
  if (isnan(maxBattAmps) || maxBattAmps > 8.0 || maxBattAmps < 2.0){
    maxBattAmps = 3.0;
    EEPROM.put(EEADDRESS, maxBattAmps); // write default charging rate to eeprom
    Serial1.println("!! Wrote default charging rate 3.0 amps to EEprom !!");
  }
  
  if (!(digitalRead(ALARM_BUTTON))) factorySelect_batt(); //factory setup of battery size
}

//------------------------------------------------------------------------------------------------------
// This routine is called if the alarm button is held at power up to set
// the max charging rate depending on battery size used at Novexcomm
// 12ah = 3 amp rate // system powered from solar port (default)
// 20ah = 5 amp rate // system powered from power supply port
//------------------------------------------------------------------------------------------------------
void factorySelect_batt(void)
{
  float f;
  
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_RED,  HIGH); // signal battery set mode
  while (!(digitalRead(ALARM_BUTTON))); //wait for button release

  leds_off_all();
  read_data();

  if (ps_volts > 10.0)  // if powered from ps port set high rate
  {
    f = 5.0; //set high rate for 20ah batt
    digitalWrite(LED_BLUE, HIGH);
  }
  else                  // if powered from solar port set low rate
  {
    f = 3.0;  //set low rate for 12ah batt
    digitalWrite(LED_GREEN, HIGH);
  }

  EEPROM.put(EEADDRESS, f);  //write charging rate to eeprom
  while (1);  //wait for power down
}


//------------------------------------------------------------------------------------------------------
// This routine reads the analog inputs for this system, solar volts, solar amps and
// battery volts, battery amps, power supply volts.
//------------------------------------------------------------------------------------------------------
int read_adc(int channel) {

  int sum = 0;
  int temp;
  int i;

  for (i = 0; i < AVG_NUM; i++) { // loop through reading raw adc values AVG_NUM number of times
    temp = analogRead(channel);   // read the input pin
    sum += temp;                  // store sum for averaging
    delayMicroseconds(50);        // pauses for 50 microseconds
  }
  return (sum / AVG_NUM);         // divide sum by AVG_NUM to get average and return it
}

//------------------------------------------------------------------------------------------------------
// This routine averages the samples to smooth the data
//------------------------------------------------------------------------------------------------------
float ema(float ave, float sample)
{
  float alpha = 2.0 / (SAMP_NUM + 1);
  ave = alpha * sample + ((1.0 - alpha) * ave);
  return ave;
}

//------------------------------------------------------------------------------------------------------
// This routine reads all the analog input values for the system. Then it multiplies them by the scale
// factor to get actual value in volts or amps.
//------------------------------------------------------------------------------------------------------
void read_data(void) {
  old_sol_watts = ema(old_sol_watts, sol_watts);                                 // save the previous value of solar watts
  sol_amps  = ema(sol_amps, (read_adc(SOL_AMPS_CHAN)  * SOL_AMPS_SCALE - SOL_AMPS_CAL));  // input of solar amps  0.5/.4
  if (sol_amps < 0.0) sol_amps = 0.0;
  bat_amps  = ema(bat_amps, (read_adc(BAT_AMPS_CHAN)  * BAT_AMPS_SCALE - BAT_AMPS_CAL));  // input of battery amps 0.5/.4
  if (bat_amps < 0.0) bat_amps = 0.0;
  sol_volts = ema(sol_volts, read_adc(SOL_VOLTS_CHAN) * SOL_VOLTS_SCALE);        // input of solar volts
  bat_volts = ema(bat_volts, read_adc(BAT_VOLTS_CHAN) * BAT_VOLTS_SCALE);        // input of battery volts
  ps_volts  = ema(ps_volts, read_adc(PS_VOLTS_CHAN)   * PS_VOLTS_SCALE);         // input of power supply volts
  sol_watts = sol_amps * sol_volts;                             // calculates new solar watts
}

//------------------------------------------------------------------------------------------------------
// This is interrupt service routine for Timer1 that occurs every 20uS.
//------------------------------------------------------------------------------------------------------
void callback(void)
{
  if (interrupt_counter++ > ONE_SECOND) {   // increment interrupt_counter until one second has passed
    interrupt_counter = 0;          // reset the counter
    seconds++;                      // then increment seconds counter
  }
}

//------------------------------------------------------------------------------------------------------
// Charger mode select function
//------------------------------------------------------------------------------------------------------
void mode_select(void) {
  static long old_millis = 0;

  switch (charger_state) {

    case Start:
      // Serial.print("mode:Start ");
      if (sol_volts > bat_volts) {
        charger_state = cc;            //wait until pv panel has sun or power supply is on, start charging
        pulseWidth = (int)((bat_volts / sol_volts) * 1023);
      }
      else if (ps_volts > bat_volts) {
        charger_state = cv;
        pulseWidth = (int)((bat_volts / ps_volts) * 1023);
      }
      break;

    case sleep:
      //Serial.print("mode:sleep ");
      if (bat_volts < BAT_CHARGE) {    //restart charging
        charger_state = cc;
        pulseWidth = (int)(bat_volts / ps_volts) * 1023;
      } else if ((millis() - old_millis) > 1800000) //napping > 30 min
      {
        old_millis = millis();
        charger_state = cv;
      }
      break;

    case mppt:
    //Serial.print("mode:mppt  ");
    case cc:
    //Serial.print("mode:cc    ");
    case cv:
      //Serial.print("mode:cv    ");
      if (bat_volts < NO_BAT)
        charger_state = no_battery ;        // If battery voltage is below 10, there is no battery connected
      else if (bat_volts > MAX_BAT_VOLTS)
        charger_state = error;              // If battery voltage is over 14.5, there's a problem
      else if ((sol_volts < bat_volts) && (ps_volts < bat_volts))  // If there's no light on the panel, or power supply is off go to start
        charger_state = Start;
      else if ((bat_amps < MIN_BAT_AMPS) && (pulseWidth > 1000))   // if not enough charge current, go to sleep
        charger_state = sleep;                                     // we have charging power select mppt, cc or cv
      else if (bat_volts >= (BAT_FLOAT - 0.1)) {
        charger_state = cv;                // If battery voltage is >= 14.4, go into cv charging
        alarm_enable = true;               // re-enable low battery alarm
      }
      else if (bat_volts < (BAT_FLOAT - 0.3)) {
        if (sol_volts <= (ps_volts + 0.25)) // if power supply volts is greater then solar volts
          charger_state = cc;              // use cc charging from power supply
        else if (bat_amps < MIN_MPPT_AMPS)
          charger_state = cc;
        else charger_state = mppt;         // else use mppt solar charging
      }
      if ((charger_state == mppt) && (bat_amps > maxBattAmps))
        charger_state = cc;
      else if ((charger_state == cv) && (bat_amps < MIN_BAT_AMPS))
      {
        charger_state = sleep;
        old_millis = millis();
      }
      break;

    case error:
      //Serial.print("mode:error ");
      if (bat_volts <= BAT_FLOAT - 0.3)    //wait until battery needs charging again
      {
        charger_state = sleep;
        old_millis = millis();
      }
      break;

    case no_battery:
      //Serial.print("mode:noBat ");
      if (bat_volts > NO_BAT)              //wait until battery is present
        charger_state = Start;
      break;

    default:
      charger_state = sleep;
      old_millis = millis();
  }
}

//------------------------------------------------------------------------------------------------------
// Charger state machine
//------------------------------------------------------------------------------------------------------
void set_charger(void) {
  switch (charger_state) { // skip to the state that is currently set

    case no_battery:       // the charger is in the no battery state
      disable_charger();   // Disable the MOSFET driver
      break;

    case sleep:            // the charger is in the sleep state
      disable_charger();   // Disable the MOSFET driver
      break;

    case mppt:             // the charger is in the mppt state
      PerturbAndObserve(); // run the MPPT algorithm
      enable_charger();    // Enable the MOSFET driver
      break;

    case cc:               // the charger is in the constant current state
      if (bat_amps < maxBattAmps) {
        pulseWidth++;
        enable_charger();  //
      }
      if (bat_amps > maxBattAmps) {
        pulseWidth -= 5;
        //pulseWidth--;
        //pulseWidth--;
        enable_charger();  //
      }
      break;

    case cv:                 // the charger is in the constant voltage state, it uses PWM instead of MPPT
      if (bat_volts < BAT_FLOAT) {
        pulseWidth++;        // If below, increase pwm
        enable_charger();
      }
      if (bat_volts > BAT_FLOAT) {
        pulseWidth--;
        enable_charger();    // If above, decrease pwm
      }
      break;

    case error:           // if there's something wrong
      disable_charger();  // Disable the MOSFET driver
      break;

    case Start:
      disable_charger();  // Disable the MOSFET driver
      break;

    default:              // if none of the other cases are satisfied,
      disable_charger();  // Disable the MOSFET driver
      break;
  }
}

//------------------------------------------------------------------------------------------------------
// MPPT algorithm
//------------------------------------------------------------------------------------------------------
void PerturbAndObserve() {
  if ((pulseWidth <= 300) || (pulseWidth >= 940) || (sol_watts < old_sol_watts))
    trackDirection = -trackDirection;       // if pulseWidth has hit one of the ends reverse the track direction
  pulseWidth = pulseWidth + trackDirection; // add (or subtract) track Direction to(from) pulseWidth
}

//------------------------------------------------------------------------------------------------------
// PWM drivers
//------------------------------------------------------------------------------------------------------
void enable_charger() {
  if (charger_state == mppt)
    pulseWidth = constrain(pulseWidth, 300, 900);
  else
    pulseWidth = constrain(pulseWidth, 0, 1023);   // prevent over/underflow of pulseWidth
  pwm = map(pulseWidth, 0, 1023, 0, 100);          // use pulseWidth to get a % value and store it in pwm
  Timer1.setPwmDuty(PWM_PIN, pulseWidth);          // use Timer1 routine to set pwm duty cycle
  TURN_ON_MOSFETS;                                 // enable the MOSFET driver
}

void disable_charger() {
  pulseWidth = 0;                                  // disable pwm
  pwm = map(pulseWidth, 0, 1023, 0, 100);          // use pulseWidth to get a % value and store it in pwm
  Timer1.setPwmDuty(PWM_PIN, pulseWidth);          // use Timer1 routine to set pwm duty cycle
  TURN_OFF_MOSFETS;                                // disable MOSFET driver
}

//------------------------------------------------------------------------------------------------------
// This routine prints all the data out to the serial port.
//------------------------------------------------------------------------------------------------------
void graph_data(void) {
  Serial.print(sol_volts);
  Serial.print(" ");
  Serial.print(sol_amps);
  Serial.print(" ");
  Serial.print(bat_volts);
  Serial.print(" ");
  //Serial.print(bat_amps);
  //Serial.print(" ");
  //Serial.println(ps_volts);
  //Serial.print(" ");
  Serial.print(sol_watts);
  Serial.println(charger_state);
}

//------------------------------------------------------------------------------------------------------
// This routine prints all the data out to the serial port.
//------------------------------------------------------------------------------------------------------
void print_data(void) {

  Serial1.print(seconds, DEC);
  Serial1.print(" ");

  //no_battery, sleep, mppt, CC, CV, error
  Serial1.print("Charging = ");
  if (charger_state == no_battery) Serial1.print("noBat");
  else if (charger_state == sleep) Serial1.print("sleep");
  else if (charger_state == mppt)  Serial1.print("mppt ");
  else if (charger_state == cc) Serial1.print("cc   ");
  else if (charger_state == cv) Serial1.print("cv   ");
  else if (charger_state == error) Serial1.print("error");
  else if (charger_state == Start) Serial1.print("start");
  Serial1.print(" ");

  Serial1.print("pwm = ");
  //Serial1.print(pulseWidth, DEC);
  //Serial1.print(",");
  Serial1.print(pwm, DEC);
  if (pwm == 100)
    Serial1.print(" ");
  else
    Serial1.print("  ");

  Serial1.print("Vpv = ");
  Serial1.print(sol_volts);
  Serial1.print(" ");

  Serial1.print("Ipv = ");
  Serial1.print(sol_amps);
  Serial1.print(" ");

  Serial1.print("Vbatt = ");
  Serial1.print(bat_volts);
  Serial1.print(" ");

  Serial1.print("Ibatt = ");
  Serial1.print(bat_amps);
  Serial1.print("/");
  Serial1.print(maxBattAmps);
  Serial1.print(" ");

  Serial1.print("Vps = ");
  Serial1.print(ps_volts);
  Serial1.print(" ");

  Serial1.print("Wpv = ");
  Serial1.print(sol_watts);
  Serial1.print(" ");

  Serial1.print("\n\r");
}

//-------------------------------------------------------------------------------------------------
//---------------------------------Led Indication--------------------------------------------------
//-------------------------------------------------------------------------------------------------
void led_output(void)
{
  if (bat_volts > 14.5 ) {
    leds_off_all();
    digitalWrite(LED_RED, LOW);
  }
  else if ((bat_volts > 12.6) && (bat_volts < 14.4)) {
    leds_off_all();
    digitalWrite(LED_GREEN, LOW);
  }
  else if ((bat_volts > 10.1) && (bat_volts < 12.5)) {
    leds_off_all();
    digitalWrite(LED_BLUE, LOW);
  }
  else if (bat_volts < 10.0) {
    leds_off_all();
  }
}

//------------------------------------------------------------------------------------------------------
// This function is used to turn all the leds off
//------------------------------------------------------------------------------------------------------
void leds_off_all(void)
{
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED,   HIGH);
  digitalWrite(LED_BLUE,  HIGH);
}

//------------------------------------------------------------------------------------------------------
// Low Battery Alarm
//------------------------------------------------------------------------------------------------------
void low_batt_alarm(void)
{
  static long low_batt_ms = 0;

  if (!(digitalRead(ALARM_BUTTON)))      // if button pressed
    alarm_enable = false;                // disable alarm

  if ((bat_volts < NO_BAT) || (bat_volts > LOW_BATT_ALARM)) // Battery disconnected or over trigger voltage immediately disables alarm
    alarm_state = off;
  else if ((bat_volts > NO_BAT) && (bat_volts < LOW_BATT_ALARM)) { // Battery is within the alarm voltage range
	switch (alarm_state) {
      case off:
	    low_batt_ms = millis();
		alarm_state = low_batt;
        break;

      case low_batt:
	    if ((millis() - low_batt_ms) > LOW_BATT_DELAY)
		   alarm_state = on;
        break;
	 }
  }

  if (alarm_enable && (alarm_state == on)) {
    // toggle alarm buzzer (beep)
    if (digitalRead(ALARM_BUZZER))
      digitalWrite(ALARM_BUZZER, LOW);
    else
      digitalWrite(ALARM_BUZZER, HIGH);
  }
  else // silence alarm
    digitalWrite(ALARM_BUZZER, LOW);
}

//------------------------------------------------------------------------------------------------------
// Main loop
//------------------------------------------------------------------------------------------------------
void loop()
{
  read_data();         // read data from inputs

  mode_select();       // select the charging state
  if (seconds != prev_seconds)
  {
    prev_seconds = seconds;
    //graph_data();        //display graph data
    print_data();      // print data
    low_batt_alarm();  //test for low battery
  }
  set_charger();  // run the charger state machine
  led_output();   // show battery SOC
  delay(10);
}
