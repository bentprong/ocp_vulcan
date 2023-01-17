# ocp_vulcan
Dell OCP Vulcan LED Text Fixture

Operating Instructions
TBD

Development Environment Setup

This varies slightly between platforms including Mac, Linux and Windows.

1. Download and install Visual Studio Code.

2. In VS Code ("VSC"), go to Extensions and seach for "platformio" and install it.  This will take some time,
watch the status area in the bottom right of VSC for progress.  Note that PlatformIO ("PIO") also installs the C/C++ 
extension which is needed.

When finished you will see a prompt "Please restart VSCode" so do that.

Windows only: It is recommended that you install cygwin so that you have bash terminal.  There are other 
options such as GitBash.  Point is, command-line git needs to be run to clone the source code repo.

3. Set up a Projects folder if you don't already have one.  For these instructions it is assumed that this is
<home>/Documents/Projects.

4. Log into GitHub.com and clone this repository: bentprong/ocp_vulcan into your Projects folder.

GitHub Requirements:
a. SSH key generated and installed on this computer for YOU
b. SSH key for YOU installed in YOUR GitHub.com account

5. In VSC, choose FIle | Open Folder... and navigate to <home>/<Projects>/ocp_vulcan then highlight and
click Select Folder.

6. In VSC, click the checkmark in the blue bar at the bottom to build.  This should install necessary files 
and tools.  It may take quite a bit of time.

7. In the repo folder platformio, open the README file and follow the instructions to configure PIO for the
Vulcan board.  There are 2 steps to this process explained in the README.

Connect Vulcan board to ATMEL-ICE and connect ATMEL-ICE to computer.
Connect Vulcan board USB-C port to computer USB port using a DATA CABLE (not a charging only cable).
Windows onoly: If not already installed, install a terminal emulator program such as TeraTerm.

8. In VSC, click Run | Start Debugging.  The code will be built for debug and you should see the debugger
stop in main() at the init() call.   Click the blue |> icon in the debugger control area of VSC.

Windows only: In TeraTerm, open a new serial connection on the new COM port and press ENTER. You should see
the Vulcan welcome message and Vulcan prompt ltf>.

Mac only: Get a listing of tty.usb* in /dev and enter "screen /dev/tty.usb<ID> 115200" and you should see
the Vulcan welcome message and Vulcan prompt ltf>.

Linux only: TBD.




