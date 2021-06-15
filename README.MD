
# Quick Start
##
### 1. Arduino IDE user prerequisites 
- Please follow this guide [Getting-started](https://github.com/stm32duino/Arduino_Core_STM32#getting-started) to complete the installation of `Arduino_Core_STM32` 
- Select the following picture in Arduino=> tools
    ![](image/setting.jpg)
- Copy all folders in the `libdeps` directory to the `~/Arduino/libraries` directory, and put them in `"Documents/Arduino/libraries"` for Windons users

### 2. How to upload the program to the board? 
- Before writing, press and hold the BOOT button on the board, then insert the USB, and then click upload 
- After writing, you need to unplug it again, and then plug it in again, select the port and open the serial port to view the output information

### 3. Why can't it be positioned? 
- Please ensure that the board and antenna are connected reliably, and the positioning area is outdoors, please do not perform positioning indoors 

### 4. My computer cannot recognize the port of the board ?

- Open the device manager, as shown in the figure, you will find `STM Device in DFU Mode`. At this time, the firmware cannot be uploaded. You need to use [zadig](https://zadig.akeo.ie/) to replace the original driver. The operation steps are as follows 
    ![](image/1.png)
- Choose `STM32 BOOLTOADER`
    ![](image/2.png)
- Click Replace Driver
    ![](image/3.png)
    ![](image/4.png)
- Click upload again. 
    ![](image/5.png)

# Description
##
### Use this example power consumption is 1.9 mA.

- If you need lower power consumption, you need to join the QQ group [984791243](https://jq.qq.com/?_wv=1027&k=VAh4pqTF) to get information.

- The limit sleep current of S76G chip can reach 157uA, we are still exploring

