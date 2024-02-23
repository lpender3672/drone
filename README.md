# drone
Drone project using a raspberry pi pico

### Prerequisites
To install and set up cmake first have installed:
* ARM GCC Compiler (https://developer.arm.com/downloads/-/gnu-rm)
* CMake (https://cmake.org/download/)
* Build Tools for Visual Studio Code (visual studio 2022)
* Python
* Git

### Installation

Ensure these are added to the PATH environment variable. <br>
First install pico-sdk in the same directory as the project or elsewhere and set the PICO_SDK_PATH environment variable to the path of the pico-sdk directory. Also update the submodules to get some required build files <br>
```
git clone -b master https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init
```
Then using __Developer command prompt for VS2022__ in the project directory run the following commands: <br>
```
mkdir build
cd build
cmake -G “NMake Makefiles” ..
```
Then to build the project run, while still in the Developer command prompt for VS2022:
```
nmake
```
### Visual Studio Code Setup

To set up in Visual Studio Code: <br>
* Ensure that the CMake Tools extension is installed. <br>

In Cmake Extension settings:
* Add item to CMake.configureEnvironment: `PICO_SDK_PATH: (path to pico-sdk)`
* Add Cmake:Generator to: `NMake Makefiles` <br>


Then set kit to GCC <version> arm-none-eabi

### Debug Setup with PicoProbe in Visual Studio Code

This took a while for me to set up so I will outline the requirements and steps to set up debugging with the PicoProbe in Visual Studio Code. <br>

Requirements:
* PicoProbe or additional Pico with firmware installed(https://www.raspberrypi.com/products/debug-probe/)
* OpenOCD compiled for RP2040. This can be compiled from source from raspberrypi openocd repository (https://github.com/raspberrypi/openocd) or downloaded from the setup for windows releases here (https://github.com/raspberrypi/pico-setup-windows/releases)
* GDB which is already included in the ARM GNU toolchain.
* Visual Studio Code with CMake Tools and Cortex-Debug extensions installed.

All the debug configurations are set up in .vscode/launch.json. <br>

You will need to specify the path "cortex-debug.openocdPath" in .vscode/settings.json to the path of the openocd executable. <br>
Then run the Pico Debug configuration in Visual Studio Code. <br>

If the GDB times out, ensure that OpenOCD is being called in terminal.
If GDB disconnects unexpectedly, ensure that OpenOCD has started correctly, found the PicoProbe interface, and detects a connected pi<br>

When OpenOCD detects the PicoProbe the following message should appear in the terminal: <br>
```
Info : Using CMSIS-DAPv2 interface with VID:PID=0x2e8a:0x000c, serial=E6633861A34B8E2C
```
Otherwise check the PicoProbe firmware is installed correctly and the correct drivers are installed using Zadig (https://zadig.akeo.ie/) <br>

If the PicoProbe detects a connected Pico the following message should appear in the terminal: <br>
```
Info : [rp2040.core0] Cortex-M0+ r0p1 processor detected
```
If this message does not appear, ensure that the Pico is wired to the PicoProbe correctly and is powered on. <br>


