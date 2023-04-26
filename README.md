# ocp_vulcan
OCP Vulcan LED Text Fixture (LTF)
Written by Richard Lewis rlewis@astlenterprises.com for Fusion Manufacturing Services
Posted January 15, 2023 at https://github.com/bentprong/ocp_vulcan
Initial Firmware Release: v1.2.1

## Release v1.3.0 Notes - April 26, 2023
1. Updated CLI and help to be clearer to the user
2. Completed the automatic calibration process function (green LED only)
3. Updated color & intensity equations 
4. Added the 'vers' command to show the firmware version and build date/time
5. Wrapped SerialUSB.print[ln] to address dropped characters on USB terminal
6. Replaced function headers with doxygen style blocks
7. Fixed compiler warnings (except in Arduino libraries those won't be fixed)
8. Changed ADC internal gain from ADC_INPUTCTRL_GAIN_DIV2 to ADC_INPUTCTRL_GAIN_2X which is what
it originally was, but it was changed during ADC debug and never changed back.  Now the firwmare
ADC gain of 1.0 is correct.  Gain necessary due to the voltage divider resistors R1/R2 and R3/R4
which divides the Spectra output range 5V-0 to 2.5V to 0.  
9. *** IMPORTANT NOTE *** The op amp U1 must be removed for proper operation of the board.  There
is an issue when the input to the op amp falls below 1.0V - the output goes up.  After removing
the op amp, short U1 pads 1-3 and 5-7 ie INx => OUTx (x=a,b) for both signals.
10. Added range checking on calculated K constant to detect if the sensor is misaligned or if
the user is using a different calibration LED than the required green LED.
11. Added a user-settable board ID parameter to the EEPROM.  Can be set with 'set bid <ID>'.
12. Added flag calibRequired that when true requires sensor calibration; flag is true if board
has never been programmed or if board was erased during programming/debugging. 
13. Added a flag sensorGood that when true, indicates that the sensor has been calibrated and 
the K constant validated.
14. Added a user-settable calibration date that is prompted during the calibration process.

## Known Issues
1. Sometimes the prompt fails to output; workaround: hit the ENTER key.

## Overview
These instructions have been tested on Windows and Mac.  If you are using a variant of Linux, the
instructions for Mac should apply except possibly for the "screen" terminal emulator program.  At
a minimum you need to select a Linux terminal emulation program and open the USB TTY with it.

This project was initially intended to be written in the Microchip Studio environment.  However,
issues were found in generating code for the CDC USB driver.  After much back and forth with
Microchip FAEs, I took their suggestion and switched to MPLAB IDE.  However, there were more
significant issues with MPLAB stability and code generation than with Studio.   Microchip was
unable to resolve these issues in the time we had.  This was in June 2022.

I have used Visual Studio Code with PlatformIO for other projects so I switched to that IDE.  That
also allowed me to leverage the Arduino drivers.  The good news was that the CDC (serial over USB)
driver worked out of the box, as did Wire (I2C).  So while much time was lost chasing Microchip
issues, the VSC/PIO IDE worked as expected and the project was completed.

I ran into issues with analogRead() so I wrote my own driver.  Part of the testing involved
exploring the ADC's automatic correction, which did not work as expected.  So you will see some
remnants of that code which I left in place but not active unless the EEPROM variable is true.

The issue with the Spectra DS LED sensor is that it takes quite a bit of time to settle down
to a stable output voltage for both color and intensity. Part of this is due to my lack of a
test fixture to hold the sensor in place, and part of it I suspect is inherent in the sensor.
With ADC oversampling/averaging, the ADC was finishing way faster than the time for the sensor
voltage to stabilize.  So software averaging over a longer period solves that problem.  Same
applied to the auto correction - ADC was too fast for the sensor.

## Operating Instructions
Important Note:  The LED sensor must be calibrated using the 'calib' cmd before using the
Led Test Fixture to measure LEDs.  When the calibration is complete, the K constant from the 
calibration will automatically be saved to EEPROM.  

To save the K constant manually, at the "ltf>" prompt, enter this command to set and save 
the K constant manually (not recommended unless the board was erased and the EEPROM K
value erased):
    set k 9.999<ENTER>

To confirm that the constant has been stored in EEPROM, enter this command:
    xdebug eeprom<ENTER>

Here is an example setting and confirming the K constant to a value of 99.89 (<ENTER> not shown):
ltf> set k 99.89  

ltf> xdebug eeprom 
EEPROM Contents:
Signature:     DE110C01
K Constant:     11.0220    <== NOTE: This constant varies between boards/sensors
Board ID:      Vulcan 1
Calib Date:    04/26/23    <== user is prompted for this date during calibration process
ADC Correct:   Disabled

The signature should always be DE110C01.  The ADC correction is disabled because it is no longer
required.

To set the board ID, use this command:
    set bid <text_here_with_underscores_not_spaces>

Example: set bid LAB_TEST_1

Note that if you don't use and underscore you will get a CLI error for too many arguments. The
default value for board ID is 'Vulcan 1'.

--------------------------------------------------------------------------------------------------
WARNING: Loading the board with (new) firmware WILL erase the EEPROM and you will need to re-enter
some parameters afterwards.  The board may display "EEPROM Validation Failed..." to indicate that
this has happened, but it is a good practice to always enter the K constant after flashing the
board's firmware.  The default value of K is 10.0 which may not work with an uncalibrated LED
sensor.  Tip: write the K constant on the board in permanent marker.
--------------------------------------------------------------------------------------------------

## Tips
Backspace and delete are implemented and erase the previous character typed.
Up arrow executes the previous command.

## Getting Started
Follow the Wiring and Terminal Instructions below to get started using the Vulcan board.

The board firmware prompt is "ltf>" and when you see that, you can enter "help<ENTER>" for help on the
available commands.

The purpose of the board is to measure a LEDs color and intensity.  Two commands are available for
that purpose: "read" (single read) and "check" (continuous loop of reads).  The terminal output is
the same for both commands.

Here is an example of the output for the read command:
    ltf> read 
    Intensity:  964 mcd  482 0.299 V
        Color:  578 nm  2872 1.781 V

You may also want to read the temperature of the board. To do that enter "temp<ENTER>".
    ltf> temp 
    Board temp: 23 C/73 F

## Development Environment Setup

This varies slightly between platforms including Mac, Linux and Windows.

1. Download and install Visual Studio Code.

2. In VS Code ("VSC"), go to Extensions and seach for "platformio" and install it.  This will take some time,
watch the status area in the bottom right of VSC for progress.  Note that PlatformIO ("PIO") also installs the C/C++ 
extension which is needed.

When finished you will see a prompt "Please restart VSCode" so do that.

Windows only: It is recommended that you install cygwin so that you have bash terminal.  There are other 
options such as GitBash.  Point is, command-line git needs to be run to clone the source code repo.

3. Set up a Projects folder if you don't already have one.  For these instructions it is assumed that this is
<home>/Documents/Projects.  VSC "may" be able to accomodate other directory structures, but of course, those
cannot and have not been tested.

    Windows:  <home> = C:/Users/<username>
    Mac:      <home> = Users/<username>

4. Log into GitHub.com using your own credentials then clone this repository: 
    bentprong/ocp_vulcan 
    
into your Projects folder.

    GitHub Requirements:
        a. SSH key generated and installed on this computer for YOU
        b. SSH key for YOU installed in YOUR GitHub.com account

5. In VSC, choose File | Open Folder... and navigate to <home>/<Projects>/ocp_vulcan then highlight that, and
click Select Folder.

6. In VSC, click the checkmark in the blue bar at the bottom to build.  This should install necessary files 
and tools.  It may take quite a bit of time.

7. In the repo folder platformio, open the README file and follow the instructions to configure PIO for the
Vulcan board.  There are 2 steps to this process explained in the README.

## Wiring Instructions
1. Connect Vulcan board to ATMEL-ICE and connect ATMEL-ICE to computer (debug only)
2. Connect Vulcan board USB-C port to computer USB port using a DATA CABLE (not a charging only cable).
3. Windows only: If not already installed, install a terminal emulator program such as TeraTerm.
 
LED BEHAVIOR: Once the Vulcan board has been powered up, there will be 2 LEDs that are on solid: one
for 3.2V and one for 5V.  These 2 LEDs are near the USB connector.  If either is off, a hardware issue
exists in one or both power supplies.  These 2 LEDs should always be on.

A third LED between the processor and debug connector should be fast blinking (4-5 times a second).  This 
means that the board has initialized OK and is waiting on a USB/serial connection.

If this third LED is on solid, an initialization error occured.  Best bet is to program the board again.
If all 3 LEDs are off, then the board is not receiving good 5V from the USB connector.  Check the USB
cable and that it is firmly in the USB connector.

Once a connection is made, the third LED will slow blink to indicate a heartbeat from the board and
firmware.

If the 3rd LED is off, the board firmware never started up.   The firmware turns this LED on when it
first starts, then it initializes itself and the harware, then it starts fast blinking unless errors
were encountered.

## Build/Debug Instructions
In VSC, click Run | Start Debugging.  The code will be built for debug and you should see the debugger
stop in main() at the init() call.   Click the blue |> icon in the debugger control area of VSC.

## Binary Executable Instructions
In VSC, click the checkmark in the blue line at the bottom to build a firmware release.  If no problems
are reported (there should be none), the executable is located here:
    <home>/Projects/ocp-vulcan/.pio/build/samd21g18a/firmware.bin

Note that in this same location is also the firmware.elf file which is the debug version of firmware.

Use any flash utility such as Microchip Studio to erase and burn this .bin file into the Vulcan board.

NOTE: PIO "upload" does not work, because there is intentionally no bootloader on the Vulcan board.

## Microchip Studio
1. Open Studio then select Tools | Device Programming.
2. Select Atmel-ICE from the Tool pulldown menu.  Verify that the Device is ATSAMD21G18A then click
the Apply button.  A menu of options will appear on the left of the screen now.
3. Select Memories then Erase Chip & Erase now.
4. In the Flash (256KB) section, use the ... button to navigate to the binary file described above.
That full path and filename should be shown in the long text area.
5. Click the Program button and wait for the status messages which should be:
    Erasing device...OK
    Programming Flash...OK
    Verifying Flash...OK
6. Click the Close button.

NOTE:  There may be a conflict between VSC and Microchip Studio if you are trying to use Studio to
flash the firmware.  Close out VSC and Studio, then restart Studio.  The conflict would be in these
2 software programs trying to use the Atmel-ICE at the same time.

## Terminal Instructions
Windows: In TeraTerm, open a new serial connection on the new COM port and press ENTER. You should see
the Vulcan welcome message and Vulcan prompt ltf> in the TeraTerm window.

Mac: Get a listing of TTYs like this:
    user@computer ~ % ls -l /dev/tty.usb*
    crw-rw-rw-  1 root  wheel    9,   2 Jan 17 14:02 /dev/tty.usbmodem146201 

Enter "screen /dev/tty.usbmodem146201" and you should see the Vulcan welcome message and Vulcan 
prompt ltf> in the terminal window.

Linux (eg Ubuntu): You can install screen or minicom using apt.  For screen, use this command:
screen /dev/ttyUSB0 115200 if the connection is on ttyUSB0. Check the /dev directory for any
matches to ttyUSB*.

** end **