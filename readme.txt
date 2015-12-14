
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
   - The last 5000 samples can be exported to a PC over a serial port in
     spreadsheet CSV (comma separated values) format, for graphing or other analysis.

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

There is a photo of the box in operation and example graphs in this directory.
