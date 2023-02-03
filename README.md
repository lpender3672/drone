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
