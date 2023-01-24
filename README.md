# drone
Drone project using a raspberry pi pico

To install and set up cmake first have installed:
* ARM GCC Compiler (https://developer.arm.com/downloads/-/gnu-rm)
* CMake (https://cmake.org/download/)
* Build Tools for Visual Studio Code (visual studio 2022)
* Python
* Git
Ensure these are added to the PATH environment variable.
<br/>
First install pico-sdk in the same directory as the project or elsewhere and set the PICO_SDK_PATH environment variable to the path of the pico-sdk directory.:
<br/>
    ```bash
    git clone -b master https://github.com/raspberrypi/pico-sdk.git
    ```
<br/>
Then in pico-sdk directory add the tinyUSB submodule
<br/>
    ```bash
    git submodule update --init
    ```
<br/>
Then using Developer command prompt for VS2022 in the project directory run the following commands: 
<br/>
    ```bash
    mkdir build
    cd build
    cmake -G “NMake Makefiles” ..
    ```
<br/>
Then to build the project run, still in the Developer command prompt for VS2022:
<br/>
    ```bash
    nmake
    ```
<br/>
To set up in Visual Studio Code:
Ensure that the CMake Tools extension is installed.
In Cmake Extension settings:
    Add item to CMake.configureEnvironment:
        PICO_SDK_PATH: <path to pico-sdk>
    Add Cmake:Generator to:
        NMake Makefiles

Then set kit to GCC <version> arm-none-eabi
