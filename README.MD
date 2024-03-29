
<h1 align = "center">🌟LilyGo T-Impulse🌟</h1>

## Using Arduino IDE

1. Download [Arduino IDE](https://www.arduino.cc/en/software)

2. Open Arduino, open preferences-> add https://github.com/stm32duino/BoardManagerFiles/raw/master/package_stmicroelectronics_index.json to the board installation manager address list

3. Open the board installation manager, wait for the index update to complete, select 'STM32 MCU based boards' and click install

4. After the installation is complete, select 'Necleo-64' in the board list.Detailed configuration reference [Q&A](#Q&A)

5. Copy all the folders in the lib directory to `"C:\User\<YourName>\Documents\Arduino\libraries"`

## Using PlatformIO

1. Install [VSCODE](https://code.visualstudio.com/) and [Python](https://www.python.org/).

2. Search for the PlatformIO plug-in in the VSCODE extension and install it.

3. After the installation is complete and the reload is completed, there will be a small house icon in the lower left corner. Click to display the Platformio IDE home page

4. Go to file - > Open folder - > Select the LilyGO-T-Impulse folder and click the (√) symbol in the lower left corner to compile (→) for upload.

## Product 📷

|  Product   |                                                                                                Product  Link                                                                                                 |
| :--------: | :----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: |
| T-Impulse | [Taobao](https://item.taobao.com/item.htm?spm=a2oq0.12575281.0.0.50111debkzn8YY&ft=t&id=655131792555)   [AliExpress](https://www.aliexpress.com/item/1005003308747726.html)   |

## Application

- [T-Impulse SoftRF](https://github.com/lyusupov/SoftRF/wiki/Bracelet-Edition)
- [S7XG lib](https://github.com/xoseperez/s7xg)
- [T-Impulse Demo](./Example/T-Impulse%20Demo)

## Introduce

![](image/introduce.jpg)

## PinOut

![](image/PinOut.jpg)

## NOTICE

The version of SX1276 in the S76G is `0x13`. In most ARDUINO libraries the detection is judged to be `0X12`. Therefore, the sending is abnormal. It is recommended to annotate the version number verification or add `0x13` to register address `0x42` in different ARDUINO libraries

E.g:lora lib

```c
    // check version
    uint8_t version = readRegister(REG_VERSION);
    if (version != 0x12) {
        return 0;
    }
```

change into

```c
    // check version
    uint8_t version = readRegister(REG_VERSION);
    if (version != 0x12 && version != 0x13) {
        return 0;
    }
```

## Q&A

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

- Choose `STM32 BOOLTOADER`,Note not `NUCLEO_L073RZ CDC in FS Mode`.

    ![](image/2.png)

- Click Replace Driver

    ![](image/3.png)

    ![](image/4.png)

- Click upload again.

    ![](image/5.png)

### 5. What is the lowest sleep power consumption current ?

- When using the SoftRF firmware test, the lowest power consumption during sleep is about 124uA

![](image/current.jpg)
