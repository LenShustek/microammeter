/**************************************************************************************

      "microammeter": a low-current power meter

  This is a box that uses the TI INA219 high-side current and
  power monitor to measure low current and voltage usage.
  This is particularly helpful for doing experiments to minimize
  power utilitization in battery-powered devices.

  The custom-built hardware contains the following:
   - Teensy 3.2 microcontroller from www.pjrc.com
   - TI INA219B current/power monitor
   - 4-line by 20-character LCD display
   - 2-pole 4-position switch to change the shunt resistors and inform the CPU
   - four pushbutton switches: "start/stop", "clear", "change rate", "export"

  The display shows:
   - instaneous voltage, current, and power
   - average voltage and current since last reset
   - cumulative current in mAH since last reset
   - elapsed time since last reset

  Other features:
   - The maximum current range can be 1500, 300, 30, or 3ma.
   - At the 3ma range, the resolution (not the accuracy!) is 0.1 uA.
   - Samples can be taken 1000, 500, 250, 100, 10, or 1 times per second.
   - The last 5000 samples can be exported to a PC over a 9600 baud serial port in
     spreadsheet CSV (comma separated values) format, for graphing or other analysis.
     (An easy way to import it is via the Arduino IDE's "Serial Monitor". )
   - A scaled real-time analog output for use with a scope or logic analyzer.

  This is what the startup screen looks like:

    L o w - c u r r e n t   D C   m e t e r
            V e r s i o n   1 . 0
      m a x   c u r r e n t :  3 0 0 m A
            r a t e :   1 0 0 / s e c

  Here's an example of a running screen:

    N o w   3 . 4 4 0 V     1 4 . 3 3 2 m A
            0 . 0 4 6 W           2 5 . 4 S
    A v g   3 . 1 9 7 V     1 2 . 5 0 1 m A
    C u m         0 . 0 8 8   m A H

  The design of the hardware and software is open source and
  posted at https://github.com/LenShustek/microammeter.

  -------------------------------------------------------------------------------
    (C) Copyright 2015, Len Shustek

    This program is free software: you can redistribute it and/or modify
    it under the terms of version 3 of the GNU General Public License as
    published by the Free Software Foundation at http://www.gnu.org/licenses,
    with Additional Permissions under term 7(b) that the original copyright
    notice and author attibution must be preserved, and under term 7(c) that
    modified versions be marked as different from the original.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
  -------------------------------------------------------------------------------
  Change log

  12 Dec 2015, V1.0, L. Shustek; first released version
  27 Dec 2015, V1.1, L. Shustek; add calibration correction for low currents
  28 Dec 2015, V1.2, L. Shustek; make small negative currents display better
  12 Mar 2016, V1.3, L. Shustek; add analog output for use with a scope or logic analyzer
  30 May 2017, V1.4, L. Shustek; fix export to format negative voltages and currents correctly

  TODO:
  - change to higher A-to-D resolution when sample rate is slow enough

*/

#define CURRENT_CALIBRATION_uA 31 // best average low-current calibration value from empirical measurements

#define VERSION "1.4"
#define DEBUG 0
#define TESTING 0  // for testing without the chip
#define ANALOG_OUTPUT 1 // analog output on pin A14?

#include <arduino.h>
#include <i2c_t3.h>   // Enhanced I2C library for Teensy 3.x; for current monitor
#include <LiquidCrystalFast.h>  // for LCD display
#include <IntervalTimer.h>

LiquidCrystalFast lcd(/*RS*/7, /*RW*/8, /*E*/9, /*D4-D7*/10, 11, 12, 13);  // LCD pins

#define PB1 16      // pushbuttons
#define PB2 15
#define PB3 14
#define PB4 17
#define PB_STARTSTOP PB1
#define PB_CLEAR PB2
#define PB_CHANGERATE PB3
#define PB_EXPORT PB4

#define ROTARY1 23  // rotary switch position inputs
#define ROTARY2 22
#define ROTARY3 21
#define ROTARY4 20

byte input_pins[] = {PB1, PB2, PB3, PB4, ROTARY1, ROTARY2, ROTARY3, ROTARY4};

#define INA219_ADDR 0x40         // I2C address of the INA219 current monitor
#define INA219_REG_CONFIG 0      // internal registers
#define INA219_REG_SHUNTVOLTAGE 1
#define INA219_REG_BUSVOLTAGE 2
#define INA219_REG_STATUS 2
#define INA219_STATUS_CNVR 0x02  // "conversion ready" bit in bus voltage register
#define INA219_STATUS_OVF  0x01  // "math overflow" bit in bus voltage register
#define INA219_REG_POWER 3
#define INA219_REG_CURRENT 4
#define INA219_REG_CALIBRATION 5

#define INA219_PG_VALUE 3       // PGA scaling for Vshuntmax: 0=40mV, 1=80mV, 2=160mV, 3=320mV
#define INA219_9bit  0b0000     // 9-bit A-to-D: 84 usec for each; really 265 usec for both
#define INA219_10bit 0b0001     // 10-bit A-to-D: 148 usec for each; really 390 usec for both
#define INA219_11bit 0b0010     // 11-bit A-to-D: 276 usec for each; really 650 usec for both
#define INA219_12bit 0b0011     // 12-bit A-to-D: 532 usec for each; really 1168 usec for both
// Those "really" numbers are measurements on a Teensy 3.1 at 72 Mhz and I2C bus at 2.4 Mhz in "immediate" mode.
// It is disappointing, but we can't sample every 500 usec in 10-bit mode. Just interrupt overhead?
#define RESOLUTION INA219_10bit // which one we choose
static uint16_t INA219_CONFIG =  // configuration register
  0 // BRNG=0: 16V full scale voltage range
  | (INA219_PG_VALUE << 11) // PGA scaling
  | (RESOLUTION << 7) // bus voltage resolution
  | (RESOLUTION << 3) // shunt voltage resolution
  | 0b011;  // mode: both shunt and bus, triggered

#define INA219_VSHUNT_MAX_MV 320     // 0.320V max shunt voltage is fixed
#define INA219_VBUS_MAX_MV 16000     // 16V max bus voltage
unsigned int usec_per_sample;
unsigned int sample_rate_choices[] = {1000, 500, 250, 100, 50, 10, 1, 0}; // samples/second
unsigned int sample_rate_index = 3;  // default: 100 samples/second

#define NUM_RES 4  // number of shunt resistors we can choose
byte rotary_pins[NUM_RES] = {ROTARY1, ROTARY2, ROTARY3, ROTARY4};
byte rotary_position; // 0..NUM_RES-1
int shunt_res_x10[NUM_RES]  = {2, 10, 100, 1000};  // shunt resistors in 10th of an ohm
int max_current_mA[NUM_RES] = {1500, 300, 30, 3};  // max current we advertise, in mA
//                           .320V/R = {1600, 320, 32, 3.2} // true max current in mA

#define DATALOGSIZE 5000
struct {  // the log of recorded values
  int voltage_mV;       // volts (.001 to 16) in mV, so 1 to 16000
  long int current_uA;  // amps (1 uA to 1.6A) in uA, so 1 to 1,600,000
} datalog[DATALOGSIZE];
int datalog_count = 0; // number of log entries stored
int datalog_index = 0; // next log array location to write

//-----------------------------------------------------------------------------------------------------------------
// display routines
//-----------------------------------------------------------------------------------------------------------------

void assert (boolean test, const char *msg) {
  if (!test) {
    lcd.clear();
    lcd.print("** INTERNAL ERROR **");
    lcd.setCursor(0, 1);
    lcd.print("assertion failed:");
    lcd.setCursor(0, 2);
    lcd.print(msg);
    while (true) ;
  }
}
void center_message (byte row, const char *msg) {
  byte len, nblanks;
  len = strlen(msg);
  assert (len <= 20, "big center_message");
  nblanks = (20 - len) >> 1;
  lcd.setCursor(0, row);
  for (byte i = 0; i < nblanks; ++i) lcd.print(" ");
  lcd.print(msg);
  nblanks = (20 - nblanks) - len;
  for (byte i = 0; i < nblanks; ++i) lcd.print(" ");
}

//-----------------------------------------------------------------------------------------------------------------
// button and switch routines
//-----------------------------------------------------------------------------------------------------------------

#define DEBOUNCE_DELAY 30      // button debounce in msec

bool button_noticed = false;  // this is a leading-edge (first push) detector
byte button_noticed_pin = 0;  // it works if only one button is pushed at a time

bool button_push (byte pin) {
  if (digitalRead(pin) == 0) {  // if it is pushed
    if (button_noticed) return false; // we know about it already
    delay(DEBOUNCE_DELAY); // otherwise: new push
    button_noticed = true;
    button_noticed_pin = pin;
    return true;
  }
  if (button_noticed && button_noticed_pin == pin) { // not pushed: recently released this one?
    delay(DEBOUNCE_DELAY);
    button_noticed = false;
  }
  return false;
}
void read_rotary_position(void) {  // record which position the rotary switch is in
  do {  // we assume only one of the rotary inputs is low
    delay(DEBOUNCE_DELAY);
    for (rotary_position = 0; rotary_position < sizeof(rotary_pins); ++rotary_position)
      if (digitalReadFast(rotary_pins[rotary_position]) == LOW)
        return;
  } while (true); // loop if we caught the (non-shorting) switch moving between positions
}

//-----------------------------------------------------------------------------------------------------------------
//  Initialization
//-----------------------------------------------------------------------------------------------------------------

void setup() {
  delay(1000);
#if DEBUG
  Serial.begin(115200); // prepare for debugging
  Serial.println("Microammeter started.\n");
#else
  Serial.begin(9600);  // prepare for data export
#endif

  lcd.begin(20, 4); // start LCD display
  lcd.noCursor();
  center_message(1, "initializing");
  for (byte i = 0; i < sizeof(input_pins); ++i)
    pinMode(input_pins[i], INPUT_PULLUP);  //configure all input pins

  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_2400, I2C_OP_MODE_IMM);

#if ANALOG_OUTPUT
  analogWriteResolution(12); // 0..4095 by hardware for A14
#endif

  delay(1000);
}

//-----------------------------------------------------------------------------------------------------------------
//  INA219 current/voltage monitor routines
//-----------------------------------------------------------------------------------------------------------------

IntervalTimer sample_timer;
bool measuring = false;

long int vshunt_uV;   // shunt volts (10 uV to 0.32V) in uV, so 10 to 320000
int vbus_mV;          // bus volts (.001 to 16) in mV, so 1 to 16000
long int current_uA;  // amps (1 uA to 1.6A) in uA, so 1 to 1,600,000
int power_mW;         // power (0 to 25.6W) in milliwatts, so 0 to 25,600

long int average_current_uA;    // average current in uA
long int remainder_current_uA;  // cumulative remainder current
long int average_vbus_mV;       // average voltage in millivolts
long int remainder_vbus_mV;     // cumulative remainder voltage
long int addendum;              // what we're adding to an average
long int average_count;         // how many measurements have gone into the averages

unsigned long long int cum_current_fAH; // cumulative current in femto amp-hours

/*      ********  Notes on our arithmetic  *********

  We need to be very careful about scaling here, particularly when adding small increments to large numbers.

  We really do need femto (10E-15) AH for the cumulative current, because each increment could be 1 uA x 500 uS,
  which is only 140 fAH. The total should be at least 13,000 mAH, or 1.3E16 fAH, which is the capacity of a D-cell
  battery. That means that the cumulative must be a long long, which tops out at 2^64 = 1.8E19. Note that 2**32 is
  only 4.2E9. And no, floating point wouldn't help: you can't add tiny increments to a big floating point number.

  Furthermore, incrementally computing averages isn't as simple as you think because of the roundoff errors.
  Here's the scoop on that:

  The naive formula for updating the average A(n) of n values with the next value V(n+1) is
      A(n+1) = (n*A(n) + V(n+1)) / (n+1)
  But that could cause overflows in the multiplication, or force us to use long long intermediate results.
  (Note that 24 hours of samples at a 500usec rate is 1.7E8 samples, so n itself just fits in a long.)

  To avoid the overflow we can recast the formula like this
      A(n+1) = A(n) + (V(n+1) - A(n)) / (n+1)
  But it still doesn't work, because it discards the fractional part of the division and the average
  will drift down. No clever rounding will work over the long term. This just isn't a stable calculation.

  The solution is to keep track of the discarded fractional part and incrementally include it in the
  calculation. That guarantees that the average is always the closest integer to the true average,
  no matter how many terms are incorporated.  To do this, we keep the running cumulated remainder R(n)
  in addition to the running current average A(n).
     addendum = (V(n+1) - A(n)) + R(n)
     A(n+1) = A(n) + addendum / (n+1)
     R(n+1) = R(n) + addendum % (n+1)
  This works perfectly, forever, and with no chance of overflow. For more information, see
  http://www.codeproject.com/Articles/807195/Precise-and-safe-calculation-method-for-the-averag.

  Note that this depends on the % modulus operator being negative if the dividend is negative, which is
  "implementation-defined" in C and C++. But it works right in the Arduino/Teensy compiler, so we use it.
*/

void wire_write16 (uint8_t reg, uint16_t value) {
#if !TESTING
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xff);
  Wire.write(value & 0xff);
  Wire.endTransmission();
#endif
}
uint16_t wire_read16(uint8_t reg) {
#if TESTING
  if (reg == INA219_REG_SHUNTVOLTAGE)
    return random(10000, 15000);   // .1V to .15V
  else if (reg == INA219_REG_BUSVOLTAGE)
    return (random(700, 900) << 3) | INA219_STATUS_CNVR; // 2.8V to 3.6V, "conversion complete"
  else return 0;
#else
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(INA219_ADDR, 2);
  return (Wire.readByte() << 8) | Wire.readByte();
#endif
}

#define START_CONVERSION wire_write16(INA219_REG_CONFIG, INA219_CONFIG)  // a macro for speed
// we don't write to the calibration register, because we do all our own
// computations based on the raw values of bus voltage and shunt current

void clear_history(void) {
  noInterrupts();
  average_current_uA = 0;
  remainder_current_uA = 0;
  average_vbus_mV = 0;
  remainder_vbus_mV = 0;
  average_count = 0;
  cum_current_fAH = 0;
  datalog_count = 0; // also clear the log
  interrupts();
}
/* We used to poll the current sensor every 500 usec from the main loop, but just writing
   to the damn LCD screen takes milliseconds, even with the new LiquidCrystalFast library!
   So now we do all the work in this periodic interrupt routine.
*/

void sample_interrupt(void) {
  assert(measuring, "bogus interrupt"); // logic error?
  // We expect the chip to be done, based on the timing for the A-to-D conversion.
  assert(wire_read16(INA219_REG_STATUS) & INA219_STATUS_CNVR, "sample not done");

  // Quickly read the raw data from the A-to-D converter and get the next conversion started.
  uint16_t shuntVraw = wire_read16(INA219_REG_SHUNTVOLTAGE);
  vshunt_uV = (long int)((int16_t)shuntVraw) * 10; // LSB is 10uV
  vbus_mV = ((int16_t)(wire_read16(INA219_REG_BUSVOLTAGE) & 0xfff8) >> 1); // mult by 4uv/LSB
  START_CONVERSION; // get a headstart on the next conversion

  // compute the instantaneous current and power, which might be negative
  current_uA = vshunt_uV * 10 / shunt_res_x10[rotary_position] - CURRENT_CALIBRATION_uA;
  power_mW = (abs(((long long)current_uA * vbus_mV))) / 1000000LL;

  // maybe output an analog voltage, ignoring negatives and scaling to 0..4095
#if ANALOG_OUTPUT
  analogWrite(A14, (int16_t)shuntVraw < 0 ? 0 : shuntVraw >> 3 );
#endif

  // make a log entry`
  datalog[datalog_index].voltage_mV = vbus_mV;
  datalog[datalog_index].current_uA = current_uA;
  if (datalog_count < DATALOGSIZE) ++datalog_count;
  if (++datalog_index >= DATALOGSIZE) datalog_index = 0;

  // Accumulate averages of voltage and current, and cumulative current, all with positive values only.
  // For insight on how this works, see the learned disquisition above about arithmetic.
  ++average_count;  // n+1:  new number of averaged samples
  addendum = max(current_uA, 0) - average_current_uA + remainder_current_uA; // average the current
  average_current_uA += addendum / average_count;
  remainder_current_uA = addendum % average_count;
  addendum = max(vbus_mV, 0) - average_vbus_mV + remainder_vbus_mV; // average the voltage
  average_vbus_mV += addendum / average_count;
  remainder_vbus_mV = addendum % average_count;
  cum_current_fAH += ((unsigned long long)max(current_uA, 0)  * usec_per_sample) * 1000 / 3600;
}

//-----------------------------------------------------------------------------------------------------------------
//  The main loop
//-----------------------------------------------------------------------------------------------------------------

void loop (void) {
  char string[50];
  unsigned long last_display_time = 0;

  usec_per_sample = 1000000L / sample_rate_choices[sample_rate_index];

  lcd.clear();  // display the welcome screen
  center_message(0, "Low-current DC meter");
  sprintf(string, "Version %s", VERSION);
  center_message(1, string);
  read_rotary_position();
  sprintf(string, "max current: %dmA", max_current_mA[rotary_position]);
  center_message(2, string);
  sprintf(string, "rate: %u/sec", sample_rate_choices[sample_rate_index]);
  center_message(3, string);
  clear_history();
  measuring = false;

#if DEBUG && !TESTING
  Serial.println("INA219 registers:");
  for (int i = 0; i < 6; ++i)  Serial.println(wire_read16(i), HEX);
  for (int i = 0; i < 10; ++i) { // do some timing tests
    unsigned long int timestamp;
    START_CONVERSION;
    timestamp = micros();
    while ((wire_read16(INA219_REG_STATUS) & INA219_STATUS_CNVR) == 0) ;
    timestamp = micros() - timestamp;
    sprintf(string, "%2d: %5lu.%03lu msec", i + 1, timestamp / 1000, timestamp % 1000);
    Serial.println(string);
  }
  Serial.println("starting loop");
#endif


  while (digitalReadFast(rotary_pins[rotary_position]) == LOW) { // as long as rotary switch doesn't change

    if (button_push(PB_CHANGERATE)) { //***  "change rate" button pushed
      if (sample_rate_choices[++sample_rate_index] == 0) sample_rate_index = 0;
      break; // go back to startup screen
    }

    if (button_push(PB_STARTSTOP)) { //***  "startstop" button pushed
      if (measuring) { // **** STOP
        sample_timer.end();  // stop the interrupts
        //delay(5); // need to do this?
        measuring = false;
        //lcd.setCursor(0, 0); lcd.print("Was");
      }
      else {          // **** START
        measuring = true;
        lcd.clear();
        START_CONVERSION; // bootstrap the first A-to-D conversion
        delay(2); // give it a head start
        assert(sample_timer.begin(sample_interrupt, usec_per_sample), "interval timer fail");  // start the periodic interrupt
      }
    }

    if (button_push(PB_CLEAR)) {  //***  "clear" pushed
      clear_history();
      // blank elapsed seconds and averages, but leave the last V, mA, and W values
      lcd.setCursor(12, 1); lcd.print("        ");
      center_message(2, "");
      center_message(3, "");
    }

    if (button_push(PB_EXPORT) && !measuring) { //***  "export" button pushed: output to the serial port
      int index, count;
      Serial.println("\"count\", \"sec\", \"volts\", \"milliamps\"");
      index = datalog_index - datalog_count;
      count = 0;
      if (index < 0) index += DATALOGSIZE;
      while (count < datalog_count) {
        unsigned long time_msec_x10;
        time_msec_x10 = (count + 1) * (usec_per_sample / 100);
        sprintf(string, "%d, %lu.%04lu, ", ++count, time_msec_x10 / 10000, time_msec_x10 % 10000);
        Serial.print(string);
        if (datalog[index].voltage_mV >= 0)
          sprintf(string, "%d.%03d, ", datalog[index].voltage_mV / 1000, datalog[index].voltage_mV % 1000);
        else
          sprintf(string, "-%d.%03d, ", (-datalog[index].voltage_mV) / 1000, (-datalog[index].voltage_mV) % 1000);
        Serial.print(string);
        if (datalog[index].current_uA >= 0)
          sprintf(string, "%ld.%03ld", datalog[index].current_uA / 1000, datalog[index].current_uA % 1000);
        else
          sprintf(string, "-%ld.%03ld", (-datalog[index].current_uA) / 1000, (-datalog[index].current_uA) % 1000);
        Serial.println(string);
        if (++index >= DATALOGSIZE) index = 0;
      }
    }

    if (measuring && (millis() - last_display_time) > 250/*msec*/) {  // time between updates of the display
      last_display_time = millis();
      // There are race conditions with cosmetic effects here if the interrupt routine updates
      // values as they are being read. But the next update will clean it up, so who cares.

      lcd.setCursor(0, 0); lcd.print("Now"); // print current values, with negative signs if warranted
      if (vbus_mV >= 0)
        sprintf(string, "%2d.%03dV", vbus_mV / 1000, vbus_mV % 1000);
      else
        sprintf(string, "-%1d.%03dV", (-vbus_mV) / 1000, (-vbus_mV) % 1000);
      lcd.print(string);
      if (current_uA >= 0)
        sprintf(string, "%4ld.%03ldmA", current_uA / 1000, current_uA % 1000);
      else if (current_uA / 1000 == 0)
        sprintf(string, "  -0.%03ldmA", (-current_uA) % 1000);
      else
        sprintf(string, "%4ld.%03ldmA", current_uA / 1000, (-current_uA) % 1000);
      lcd.print(string);

      sprintf(string, "%5d.%03dW%7ld.%ldS", power_mW / 1000, power_mW % 1000,
              average_count / (1000000L / usec_per_sample),
              (average_count / (100000L / usec_per_sample)) % 10);
      lcd.setCursor(0, 1); lcd.print(string);
      sprintf(string, "Avg %ld.%03ldV %3ld.%03ldmA",
              average_vbus_mV / 1000, average_vbus_mV % 1000,
              average_current_uA / 1000, average_current_uA % 1000);
      lcd.setCursor(0, 2); lcd.print(string);
      sprintf(string, "Cum%5ld.%03ld mAH",
              (unsigned long)(cum_current_fAH / 1000000000000LL), // mAH
              (unsigned long)(cum_current_fAH / 1000000000LL) % 1000); // uAH
      lcd.setCursor(0, 3); lcd.print(string);
    }
  }
  sample_timer.end();  // stop the interrupts
  measuring = false;
}

